#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_run.py —— 把一次抓取运行录成 CSV（旁路录制，不动主程序）

为什么单独做
------------
grasp_task.py 的日志是「叙述型」的：只有结论（成功/中止）、关键点的 xyz、
以及超过 5 秒的 movej 耗时。它能回答"成没成、卡在哪"，但回答不了
"轨迹是否走直线、关节抖不抖、每一步多快、感知给了什么中间量"。

所以这里做一个**只读旁路**：订阅关节角/目标/夹爪状态，自己算 FK，
按 /joint_states 的节奏写 CSV。它【不发布任何指令】，对主链路零风险。

用法
----
  # 终端 A：跑任务
  grasp 2>&1 | tee ~/grasp_9.log

  # 终端 B：同时录制（先起它，再起任务）
  python3 record_run.py --tag trial9

  Ctrl-C 结束，会打印一行汇总（时长/行数/TCP 轨迹总长）

输出
----
  ~/grasp_data/run_20261110-134500_trial9.csv

列含义
------
  t_rel        相对开始的秒数
  j1..j6       关节角（rad，实测反馈）
  tip_x/y/z    指尖(TCP)在基座系位置（FK + tcp_offset，单位 m）
  fl_x/y/z     法兰位置
  tgt_x/y/z    感知给出的目标顶面中心（没有则为空）
  gw,gang      夹爪开口(mm) / 电机角度(deg)
  greach,gstall 到位 / 堵转 标志
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main(argv=None):
    ap = argparse.ArgumentParser(description="旁路录制一次抓取运行（CSV）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=os.path.expanduser("~/grasp_data"))
    ap.add_argument("--tag", default="", help="文件名后缀，便于区分第几次")
    ap.add_argument("--rate", type=float, default=0.0,
                    help="最大记录频率 Hz（0=每条 /joint_states 都记）")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="多少秒后自动停（0=等 Ctrl-C）")
    a = ap.parse_args(argv)

    from common import load_config, setup_import_paths, find_urdf
    cfg = load_config(a.config)
    setup_import_paths(cfg)
    from ik_check import UrdfKinematics
    from planner import tip_position

    kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                         cfg["robot"]["tip_frame"])
    offset = np.asarray(cfg["robot"]["tcp_offset"], float)
    names = list(cfg["robot"]["joints"])

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
    except ImportError:
        print("❌ 没有 rclpy：先 source /opt/ros/foxy/setup.bash 和 install/setup.bash")
        return 2

    os.makedirs(a.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    fn = os.path.join(a.out, "run_%s%s.csv" % (stamp, ("_" + a.tag) if a.tag else ""))

    rclpy.init()
    node = Node("record_run")
    st = {"q": None, "tgt": None, "grip": None, "rows": 0,
          "t0": time.time(), "last": 0.0, "path": 0.0, "prev": None}

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in names):
            st["q"] = np.array([d[n] for n in names], dtype=float)

    def on_tgt(m):
        if len(m.data) >= 3:
            st["tgt"] = [float(x) for x in m.data[:3]]

    def on_grip(m):
        st["grip"] = [float(x) for x in m.data[:4]]

    node.create_subscription(JointState, cfg["arm"]["joint_states_topic"], on_js, 20)
    node.create_subscription(Float64MultiArray, cfg["perception"]["target_topic"], on_tgt, 20)
    node.create_subscription(Float64MultiArray, cfg["gripper"]["state_topic"], on_grip, 20)

    f = open(fn, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["t_rel", "j1", "j2", "j3", "j4", "j5", "j6",
                "tip_x", "tip_y", "tip_z", "fl_x", "fl_y", "fl_z",
                "tgt_x", "tgt_y", "tgt_z",
                "gw", "gang", "greach", "gstall"])

    print("=" * 62)
    print(" 录制中 → %s" % fn)
    print(" 订阅：%s / %s / %s" % (cfg["arm"]["joint_states_topic"],
                                   cfg["perception"]["target_topic"],
                                   cfg["gripper"]["state_topic"]))
    print(" Ctrl-C 结束（或 --duration 到点自动停）")
    print("=" * 62)

    min_dt = (1.0 / a.rate) if a.rate > 0 else 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if a.duration > 0 and time.time() - st["t0"] > a.duration:
                break
            q = st["q"]
            if q is None:
                continue
            now = time.time()
            if min_dt and (now - st["last"]) < min_dt:
                continue
            st["last"] = now

            p = tip_position(kin, q, offset)
            T = kin.fk(q)
            if st["prev"] is not None:
                st["path"] += float(np.linalg.norm(p - st["prev"]))
            st["prev"] = p

            tgt = st["tgt"] or [None, None, None]
            g = st["grip"] or [None, None, None, None]
            w.writerow([round(now - st["t0"], 4)] +
                       [round(float(v), 5) for v in q] +
                       [round(float(v), 5) for v in p] +
                       [round(float(v), 5) for v in T[:3, 3]] +
                       [None if v is None else round(float(v), 5) for v in tgt] +
                       [None if g[0] is None else round(float(g[0]), 2),
                        None if g[1] is None else round(float(g[1]), 1),
                        None if g[2] is None else int(float(g[2])),
                        None if g[3] is None else int(float(g[3]))])
            st["rows"] += 1
    except KeyboardInterrupt:
        pass

    f.close()
    node.destroy_node()
    rclpy.shutdown()

    dur = time.time() - st["t0"]
    print("\n" + "-" * 62)
    print(" 停止：%s" % fn)
    print(" 时长 %.1fs / %d 行 / 平均 %.1f Hz" %
          (dur, st["rows"], st["rows"] / dur if dur > 0 else 0))
    print(" TCP 轨迹总长 %.3f m（判断「有没有绕路」的基线）" % st["path"])
    print("-" * 62)
    return 0


if __name__ == "__main__":
    sys.exit(main())

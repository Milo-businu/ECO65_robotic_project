#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""示教放置点 —— 把「抓完往哪放」量成 3 个数字，写进 real_params.yaml

为什么需要它
------------
抓取的目标位置是感知算出来的，每帧都在变；而放置区是【固定】的，
所以放置点属于"标定结果"，不该靠视觉，也不该靠手估。

最省事的量法：用 jog.py（键盘示教，替代示教器）把夹爪移到放置区上方，
让**指尖（TCP）刚好碰到放置台面**，然后回车。脚本读当前 /joint_states，
做正运动学 + 加 tcp_offset，算出指尖在基座系里的 (x, y, z)：

    x, y  → place.xy        放置区中心
    z     → place.surface_z 台面高度

为什么不能只读 /joint_states 手算
--------------------------------
  · 指尖不等于法兰：真机夹爪沿末端轴伸出 17.7cm（robot.tcp_offset）
  · 夹爪不一定竖直向下：必须用 R·offset 而不是直接加 z
  这两条都容易错 1~2cm，而 1cm 就足以让释放时方块歪掉或压到台面。

用法
----
  # 真机（推荐）：订阅 /joint_states，摆好姿势后回车
  python3 teach_place_point.py

  # 离线算：已知关节角（弧度），不接硬件也能核对
  python3 teach_place_point.py --from-joints "0.0,0.2,0.3,0.0,0.0,0.0"

  # 连续采几个点对比（比如放置区四角），回车每次采一个，q 退出
  python3 teach_place_point.py --multi
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def _print_result(q, cfg, kin, offset, tag=""):
    from planner import tip_position
    p = tip_position(kin, q, offset)
    T = kin.fk(np.asarray(q, float))
    reach = float(np.hypot(p[0], p[1]))

    print("\n" + "-" * 58)
    if tag:
        print("  采样点：%s" % tag)
    print("  关节角   %s" % np.round(np.asarray(q, float), 4).tolist())
    print("  法兰位置 %s" % np.round(T[:3, 3], 4).tolist())
    print("  指尖(TCP) [%.4f  %.4f  %.4f]   水平距离 %.3fm" % (p[0], p[1], p[2], reach))
    print("-" * 58)
    print("  把下面两行填进 config/real_params.yaml 的 place 段：")
    print()
    print("    xy: [%.4f, %.4f]" % (p[0], p[1]))
    print("    surface_z: %.4f" % p[2])
    print()
    print("  ⚠️ 前提：采样时指尖是【刚好碰到放置台面】，不是悬空、不是压进去。")
    print("     悬空 1cm → surface_z 高 1cm → 释放时方块多掉 1cm（会歪）。")

    warn = []
    if not (0.05 <= reach <= 0.62):
        warn.append("水平距离 %.3fm 超出 ECO65 工作半径（0.627m）附近，"
                    "这个点大概率抓不到/放不到" % reach)
    if p[2] < -0.15 or p[2] > 0.40:
        warn.append("z=%.3fm 看起来不像桌面高度（-0.15~0.40 之外），"
                    "确认是不是把指尖停在了半空" % p[2])
    if abs(p[1]) > 0.55:
        warn.append("y=%.3fm 偏得很远，确认放置区真的在这个位置" % p[1])
    for w in warn:
        print("  ❌ %s" % w)
    if not warn:
        print("  ✅ 数值范围看起来合理（最终仍以真机试放一次为准）")
    print("-" * 58)
    return p


def main(argv=None):
    ap = argparse.ArgumentParser(description="示教放置点（指尖 → place.xy / surface_z）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--from-joints", default=None,
                    help='离线模式：逗号分隔的 6 个关节角（弧度），如 "0,0.2,0.3,0,0,0"')
    ap.add_argument("--multi", action="store_true",
                    help="连续采样：每次回车采一个点，q 退出（用来核对放置区四角）")
    ap.add_argument("--topic", default=None, help="关节角话题，默认取配置里的")
    ap.add_argument("--timeout", type=float, default=10.0,
                    help="等 /joint_states 的超时秒数（默认 10）")
    a = ap.parse_args(argv)

    from common import load_config, setup_import_paths, find_urdf, fmt
    cfg = load_config(a.config)
    setup_import_paths(cfg)
    from ik_check import UrdfKinematics
    from planner import tip_position

    kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                         cfg["robot"]["tip_frame"])
    offset = np.asarray(cfg["robot"]["tcp_offset"], float)

    print("=" * 58)
    print("  示教放置点   TCP 偏移 = %s（沿工具 z 轴）" % fmt(offset, 3))
    print("=" * 58)

    # ---------- 离线模式 ----------
    if a.from_joints:
        q = np.array([float(x) for x in a.from_joints.replace("，", ",").split(",")])
        if len(q) != 6:
            print("❌ --from-joints 需要 6 个数，收到 %d 个" % len(q))
            return 2
        _print_result(q, cfg, kin, offset, "离线计算")
        return 0

    # ---------- 真机模式 ----------
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        print("❌ 没有 rclpy：先 source /opt/ros/foxy/setup.bash，"
              "或改用 --from-joints 离线算")
        return 2

    topic = a.topic or cfg["arm"].get("joint_states_topic", "/joint_states")
    names = list(cfg["robot"]["joints"])

    rclpy.init()
    node = Node("teach_place_point")
    state = {"q": None}

    def _on_js(m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in names):
            state["q"] = np.array([d[n] for n in names], dtype=float)

    node.create_subscription(JointState, topic, _on_js, 10)
    print("\n等 %s 的第一帧（最多 %.0fs）…" % (topic, a.timeout))
    import time
    t0 = time.time()
    while state["q"] is None and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)

    if state["q"] is None:
        print("❌ %.0fs 内没收到关节角。依次查：" % a.timeout)
        print("   1) rm_driver 起着吗：ros2 topic list | grep joint_states")
        print("   2) udp_ip 填的是本机 IP 吗：hostname -I")
        print("   3) 机械臂使能了吗（控制器上电 / 用 rm_driver 的使能接口）")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print("✅ 收到关节角 %s" % fmt(state["q"]))
    print("\n现在用 jog.py 把【指尖】移到放置区中心、刚好碰到台面，然后回到这个终端按回车。")
    print("（随时按 Ctrl+C 退出；--multi 模式下每回车采一个点）\n")

    results = []
    while True:
        try:
            line = input("摆好后回车采样（q 退出）> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if line in ("q", "quit", "exit"):
            break
        # 用最新一帧（spin 一下，确保不是几秒前的旧值）
        rclpy.spin_once(node, timeout_sec=0.3)
        if state["q"] is None:
            print("⚠️ 暂时没有关节角，稍等再试")
            continue
        p = _print_result(state["q"], cfg, kin, offset,
                          "第 %d 个" % (len(results) + 1))
        results.append(p)
        if not a.multi:
            break

    node.destroy_node()
    rclpy.shutdown()

    if len(results) >= 2:
        arr = np.asarray(results, float)
        print("\n" + "=" * 58)
        print("  多次采样汇总（核对放置区尺寸 / 是不是同一个平面）")
        print("=" * 58)
        print("  x 范围 %.4f ~ %.4f  (%.1fmm)" %
              (arr[:, 0].min(), arr[:, 0].max(),
               (arr[:, 0].max() - arr[:, 0].min()) * 1000))
        print("  y 范围 %.4f ~ %.4f  (%.1fmm)" %
              (arr[:, 1].min(), arr[:, 1].max(),
               (arr[:, 1].max() - arr[:, 1].min()) * 1000))
        print("  z 范围 %.4f ~ %.4f  (%.1fmm)" %
              (arr[:, 2].min(), arr[:, 2].max(),
               (arr[:, 2].max() - arr[:, 2].min()) * 1000))
        if arr[:, 2].max() - arr[:, 2].min() > 0.005:
            print("  ⚠️ z 相差超过 5mm：放置台面不平，或某次指尖没真的碰到台面")
        print("  建议取中心的平均值：")
        print("    xy: [%.4f, %.4f]" % (arr[:, 0].mean(), arr[:, 1].mean()))
        print("    surface_z: %.4f" % arr[:, 2].mean())
    return 0


if __name__ == "__main__":
    sys.exit(main())

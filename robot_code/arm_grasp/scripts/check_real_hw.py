#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_real_hw.py —— 真机到场第一件事：在线硬件链路体检

只读订阅，**不下发任何运动/夹爪指令**，可以放心在接电后第一分钟跑。

为什么要有它：真机翻车最常见的不是算法，而是「某个话题根本没数据」——
/joint_states 没刷（udp_ip 填错）、对齐深度没开、相机前缀不对、夹爪串口被占。
这些都可以在动手标定前 8 秒内一次性查出来，而不是花两小时盲调。

用法：
  source /opt/ros/foxy/setup.bash && source ~/robot_vla_ws/install/local_setup.bash
  python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8

  # 不接 ROS 只看配置（到场前在 Mac/云机也能跑）
  python3 check_real_hw.py --config-only

检查分组：
  ① 配置   mode / allow_arm_motion / handeye / gripper / tcp_offset / udp_ip
  ② 串口   /dev/motor_serial 存在、权限、是否被占用
  ③ 相机   彩色图 / camera_info / 对齐深度是否在出数；深度有效比例与距离
  ④ 机械臂 /joint_states 是否在刷、6 个关节名是否齐
  ⑤ 夹爪   /gripper/state 是否在刷、开口是否已知
  ⑥ 感知   /grasp/target_xyz（仅当手眼已标定时要求）

输出 ✅ / ⚠️ / ❌ 与「下一步」；退出码 0=无致命项，1=有 ❌。
"""

from __future__ import annotations

import argparse
import subprocess
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import load_config   # noqa: E402

OK, WARN, FAIL = "✅", "⚠️ ", "❌"
_items = []


def rec(level, group, msg):
    _items.append((level, group, msg))
    print("  %s [%s] %s" % (level, group, msg))


# ---------------------------------------------------------------- ① 配置
def check_config(cfg):
    prof, robot = cfg["profile"], cfg["robot"]
    arm, cam, h, g = cfg["arm"], cfg["camera"], cfg["handeye"], cfg["gripper"]
    print("\n--- ① 配置 ---")
    mode = str(prof.get("mode", "dry-run"))
    rec(OK if mode == "real" else WARN, "配置",
        "profile.mode = %s%s" % (mode, "" if mode == "real" else "（演练中，不会动硬件）"))
    if not prof.get("allow_arm_motion", False):
        rec(WARN, "配置", "allow_arm_motion=false —— 第二道锁未开（标定阶段这是对的，抓取前才开）")
    else:
        rec(OK, "配置", "两道锁都已打开：真机运动会真实下发")

    off = robot.get("tcp_offset", [0, 0, 0.195])
    rec(OK, "配置", "tcp_offset = %s（真机实测 0.177；改动后必须重新验证下降终点）"
        % off)
    if abs(float(off[2]) - 0.177) > 0.005:
        rec(WARN, "配置", "tcp_offset 不是 0.177，确认是否换过夹爪/安装方式")

    rec(OK if h.get("calibrated") and h.get("matrix_4x4") else WARN, "配置",
        "手眼标定 %s" % ("已完成" if h.get("calibrated") else "未完成 → 感知节点会拒绝出坐标"))
    if h.get("calibrated") and not (h.get("tf_xyz") and h.get("tf_quat_xyzw")):
        rec(WARN, "配置", "tf_xyz / tf_quat_xyzw 未填 → RViz 里看不到相机（不影响抓取）")

    rec(OK if g.get("calibration_done") else WARN, "配置",
        "夹爪开口标定 %s（k=%.4f °/mm, 空程=%.1f°, max=%.1fmm）"
        % ("已完成" if g.get("calibration_done") else "未完成 → 开口只是粗估",
           float(g.get("deg_per_mm", 0)), float(g.get("deadzone_deg", 0)),
           float(g.get("max_width_mm", 0))))

    rec(OK if cam.get("serial_no") else WARN, "配置",
        "相机序列号 %s" % (cam.get("serial_no") or "未填（多设备时可能开错相机）"))
    rec(WARN, "配置", "camera.prefix = %s（以 ros2 topic list | grep d435i 实测为准）"
        % cam.get("prefix"))
    rec(WARN, "配置", "arm.udp_ip = %s —— 必须等于本机 IP（hostname -I），否则 /joint_states 全空"
        % arm.get("udp_ip"))
    return g, h, cam, arm


# ---------------------------------------------------------------- ② 串口
def check_serial(g):
    print("\n--- ② 串口 ---")
    p = g.get("port", "/dev/motor_serial")
    if os.path.exists(p):
        rec(OK, "串口", "%s 存在 → %s" % (p, os.path.realpath(p)))
    else:
        rec(FAIL, "串口", "%s 不存在。检查 udev 规则 / 拨码 2 号 ON / 供电 10~25V" % p)
        return
    try:
        import serial
        s = serial.Serial(p, int(g.get("baud", 115200)), timeout=0.2)
        s.close()
        rec(OK, "串口", "可正常打开（当前无人占用，标定脚本可独占）")
    except ImportError:
        rec(WARN, "串口", "pyserial 未装，跳过打开测试")
    except Exception as e:                          # noqa: BLE001
        rec(WARN, "串口", "打不开：%s —— 多半是 gripper_node 已占用（标定脚本前先停整机 launch）" % e)


# ---------------------------------------------------------------- 在线部分
def usb_health():
    """查内核日志里的 USB 复位/掉线 —— 相机线被拽过的第一手证据。

    为什么重要：USB3 线被拉直时，先出问题的不是机械结构，而是信号。
    典型表现是 dmesg 里出现 "USB disconnect" / "reset high-speed USB device" /
    "device descriptor read/64, error -71"，而 ROS 侧只表现为帧率掉、深度断流，
    很容易被误判成"感知算法坏了"。
    返回 (告警条数, 最后一条内容)，读不到 dmesg 就返回 (None, 原因)。
    """
    try:
        out = subprocess.run(["dmesg", "--ctime"], capture_output=True, text=True,
                             timeout=5)
        if out.returncode != 0 or not out.stdout:
            # 普通用户可能没权限；再试 journalctl
            out = subprocess.run(["journalctl", "-k", "-n", "400", "--no-pager"],
                                 capture_output=True, text=True, timeout=8)
        txt = out.stdout or ""
    except Exception as e:                         # noqa: BLE001
        return None, "读不到内核日志：%s" % e
    if not txt.strip():
        return None, "内核日志为空或无权限（sudo dmesg 可看）"

    bad = []
    for ln in txt.splitlines():
        low = ln.lower()
        if "usb" not in low:
            continue
        if ("disconnect" in low or "reset" in low or "error -71" in low
                or "device descriptor read" in low or "over-current" in low):
            bad.append(ln.strip())
    if not bad:
        return 0, ""
    return len(bad), bad[-1][:120]


def online_check(cfg, seconds):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo, JointState
    from std_msgs.msg import Float64MultiArray

    g, h, cam, arm = cfg["gripper"], cfg["handeye"], cfg["camera"], cfg["arm"]
    prefix = str(cam.get("prefix", "/camera/d435i")).rstrip("/")
    topics = {
        "color": "%s/color/image_raw" % prefix,
        "info": "%s/color/camera_info" % prefix,
        "depth": "%s/aligned_depth_to_color/image_raw" % prefix,
        "js": str(arm.get("joint_states_topic", "/joint_states")),
        "gstate": str(g.get("state_topic", "/gripper/state")),
        "target": str(cfg["perception"].get("target_topic", "/grasp/target_xyz")),
        "movej_result": str(arm.get("movej_result_topic", "/rm_driver/movej_result")),
    }

    rclpy.init()
    node = Node("check_real_hw")
    counts, last = {}, {}

    def bind(key, topic, msg_type):
        def cb(m, k=key):
            counts[k] = counts.get(k, 0) + 1
            last[k] = m
        node.create_subscription(msg_type, topic, cb, 10)

    bind("color", topics["color"], Image)
    bind("info", topics["info"], CameraInfo)
    bind("depth", topics["depth"], Image)
    bind("js", topics["js"], JointState)
    bind("gstate", topics["gstate"], Float64MultiArray)
    bind("target", topics["target"], Float64MultiArray)

    print("\n  采样 %d 秒（只读，不下发任何指令）…" % seconds)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Foxy 返回 [(topic_name, [types]), ...]
    existing = set()
    try:
        existing = set(n for n, _ in node.get_topic_names_and_types())
    except Exception:                               # noqa: BLE001
        pass

    def has(key):
        return counts.get(key, 0) > 0

    # ---- 重复节点检查：上一轮没退干净的节点会和新的抢同一个话题 ----
    # rm_driver 起两份 → movej_cmd 被两个驱动各发一次 → 机械臂动作翻倍/打架，
    # 现象是"突然跑过头"，最难查。grasp_task 起两份 → 两条状态机同时发指令。
    print("\n--- ②b 节点重复 ---")
    try:
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        dup = sorted({n for n in names if names.count(n) > 1 and not n.startswith("_")})
        if dup:
            rec(WARN, "节点", "发现重名节点：%s → 可能是上次没退干净，"
                              "先 pkill -f 再重起（否则指令会被执行两次）"
                % ", ".join(dup))
        else:
            rec(OK, "节点", "没有重名节点（rm_driver / grasp_task 各一份）")
        for key in ("rm_driver", "grasp_task", "gripper_node", "perception_node"):
            if key in names:
                rec(OK, "节点", "%s 在跑" % key)
    except Exception as e:                          # noqa: BLE001
        rec(WARN, "节点", "查不到节点列表：%s" % e)

    print("\n--- ②c 相机 USB 链路（线被拽过就会在这里留痕）---")
    n_usb, msg_usb = usb_health()
    if n_usb is None:
        rec(WARN, "USB", msg_usb)
    elif n_usb == 0:
        rec(OK, "USB", "内核日志里没有 USB 复位/掉线记录")
    else:
        rec(WARN, "USB", "发现 %d 条 USB 复位/掉线记录，最后一条：%s" % (n_usb, msg_usb))
        print("       → 相机线可能被拽过。抓取中途会表现为「帧率掉/深度断流」，")
        print("         看起来像感知坏了，其实是线。先重走线（腕部留一段松弛弧）再跑。")

    print("\n--- ③ 相机 ---")
    rec(OK if has("color") else FAIL, "相机", "彩色图 %s：%d 帧"
        % (topics["color"], counts.get("color", 0)))
    if has("info"):
        K = last["info"].k
        rec(OK, "相机", "camera_info：fx=%.1f fy=%.1f cx=%.1f cy=%.1f（对齐深度用【彩色】内参）"
            % (K[0], K[4], K[2], K[5]))
    else:
        rec(FAIL, "相机", "camera_info 无数据：%s" % topics["info"])

    if has("depth"):
        import numpy as np
        m = last["depth"]
        enc = (m.encoding or "").lower()
        if "16uc1" in enc or "mono16" in enc:
            d = np.frombuffer(m.data, dtype=np.uint16)[:m.height * m.width]
            scale = 1000.0
            unit = "mm"
        elif "32fc1" in enc:
            d = np.frombuffer(m.data, dtype=np.float32)[:m.height * m.width]
            scale = 1.0
            unit = "m"
        else:
            d, scale, unit = np.array([]), 1.0, "?"
        if d.size:
            dv = d.astype(float) / scale
            good = dv[np.isfinite(dv) & (dv > 0.05) & (dv < 3.0)]
            ratio = len(good) / float(d.size)
            med = float(np.median(good)) if len(good) else 0.0
            rec(OK if ratio > 0.30 else FAIL, "相机",
                "对齐深度：%d 帧，有效点 %.1f%%，距离中值 %.3f m（单位 %s）"
                % (counts["depth"], ratio * 100, med, unit))
            if ratio <= 0.30:
                rec(WARN, "相机", "有效点太少：拉窗帘/避免反光/确认目标在 0.28~0.6m/"
                    "launch 加 enable_depth_filters:=true")
            elif med < 0.28:
                rec(WARN, "相机", "目标太近（<0.28m）进 D435i 盲区，抬高相机或拉远")
            elif med > 0.80:
                rec(WARN, "相机", "目标偏远，深度误差随距离增长；尽量落在 0.3~0.6m")
        else:
            rec(WARN, "相机", "深度 encoding=%s 未识别，跳过数值分析" % enc)
    else:
        rec(FAIL, "相机", "对齐深度无数据：%s —— 确认 align_depth.enable:=true" % topics["depth"])

    print("\n--- ④ 机械臂 ---")
    if has("js"):
        names = list(last["js"].name)
        need = list(cfg["robot"]["joints"])
        missing = [n for n in need if n not in names]
        rec(OK if not missing else FAIL, "机械臂",
            "/joint_states：%d 帧，关节 %s%s"
            % (counts["js"], names[:8], ("  缺 %s" % missing) if missing else ""))
        if not missing:
            rec(OK, "机械臂", "FK 可用（感知节点靠它现算 T_base←Link6，禁止缓存）")
    else:
        rec(FAIL, "机械臂", "/joint_states 一条都没有 —— 90% 是 rm_eco65_config.yaml 的 "
            "udp_ip 不是本机 IP；或 rm_driver 没起")
    if topics["movej_result"] in existing or any("movej_result" in n for n in existing):
        rec(OK, "机械臂", "rm_driver 指令通道在线（%s 已声明）" % topics["movej_result"])
    else:
        rec(WARN, "机械臂", "未发现 %s —— 确认 rm_driver 已启动" % topics["movej_result"])

    print("\n--- ⑤ 夹爪 ---")
    if has("gstate"):
        v = list(last["gstate"].data)
        opening = v[0] if v else -1
        rec(OK if opening >= 0 else WARN, "夹爪",
            "/gripper/state：%d 帧，开口=%.1fmm 角度=%.1f° 到位=%s 堵转=%s"
            % (counts["gstate"], opening,
               v[1] if len(v) > 1 else 0, v[2] if len(v) > 2 else 0,
               v[3] if len(v) > 3 else 0))
        if opening < 0:
            rec(WARN, "夹爪", "开口未知（-1）→ 先发 home 建立基准")
    else:
        rec(WARN, "夹爪", "/gripper/state 无数据 —— gripper_node 没起（标定阶段本就不该起）")

    print("\n--- ⑥ 感知 ---")
    if h.get("calibrated"):
        rec(OK if has("target") else FAIL, "感知",
            "/grasp/target_xyz：%d 帧%s"
            % (counts.get("target", 0),
               "" if has("target") else " —— 手眼已标定却无输出，检查颜色阈值/深度/相机前缀"))
    else:
        rec(WARN, "感知", "手眼未标定 → /grasp/target_xyz 本来就该没数据，先做手眼标定")

    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="真机在线硬件链路体检（只读）")
    ap.add_argument("--config", default=None, help="real_params.yaml 路径")
    ap.add_argument("--seconds", type=float, default=8.0, help="采样秒数，默认 8")
    ap.add_argument("--config-only", action="store_true",
                    help="只查配置和串口，不连 ROS（到场前也能跑）")
    a = ap.parse_args()

    print("=" * 66)
    print(" 真机硬件链路体检  %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 66)
    cfg = load_config(a.config)
    g, h, cam, arm = check_config(cfg)
    check_serial(g)

    if not a.config_only:
        try:
            online_check(cfg, a.seconds)
        except Exception as e:                      # noqa: BLE001
            rec(FAIL, "ROS", "在线检查异常：%s（先确认已 source /opt/ros/foxy/setup.bash）" % e)

    n_fail = sum(1 for lv, _, _ in _items if lv == FAIL)
    n_warn = sum(1 for lv, _, _ in _items if lv == WARN)
    print("\n" + "=" * 66)
    print(" 汇总：❌ %d 项致命 / ⚠️  %d 项待办 / ✅ %d 项通过"
          % (n_fail, n_warn, sum(1 for lv, _, _ in _items if lv == OK)))
    if n_fail:
        print(" 结论：NO-GO —— 先修完上面所有 ❌ 再开始标定，不要往下走。")
    else:
        print(" 结论：GO —— 链路就绪，可以开始第 1 段（手眼标定）。")
    print("=" * 66)
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())

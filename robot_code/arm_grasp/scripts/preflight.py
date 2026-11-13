#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""移植自检 —— 在真机接电之前，把"代码/配置/算法"能验的都验完

为什么要有它
------------
把项目从云服务器搬到真机，最怕的是「硬件接好了，结果卡在一行拼写错误」。
本脚本不碰任何硬件，用纯软件把整条链路跑一遍：

  配置 → URDF/FK → IK 规划 → 夹爪协议 → 完整状态机演练 → VLA 接口

跑完会明确告诉你：哪些 ✅ 已经就绪，哪些 ❌ 必须先处理，
以及"接上硬件后第一步该做什么"。

用法（在真机或云服务器都行）：
  python3 preflight.py
  python3 preflight.py --config /path/to/real_params.yaml
"""

from __future__ import annotations

import argparse
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PASS, FAIL, WARN = [], [], []


def ok(msg):
    PASS.append(msg)
    print("  ✅ %s" % msg)


def bad(msg):
    FAIL.append(msg)
    print("  ❌ %s" % msg)


def warn(msg):
    WARN.append(msg)
    print("  ⚠️  %s" % msg)


def section(t):
    print("\n" + "-" * 62 + "\n  " + t + "\n" + "-" * 62)


def main():
    ap = argparse.ArgumentParser(description="ECO65 真机移植自检")
    ap.add_argument("--config", default=None)
    ap.add_argument("--skip-dryrun", action="store_true",
                    help="跳过完整状态机演练（URDF 缺失时用）")
    a = ap.parse_args()

    print("=" * 62)
    print("  ECO65 整机移植自检（不接硬件）")
    print("=" * 62)

    # ---------------- 1 运行环境 ----------------
    section("[1/7] Python 与依赖")
    print("  Python %s" % sys.version.split()[0])
    if sys.version_info[:2] >= (3, 8):
        ok("Python ≥ 3.8（ROS2 Foxy 的 Python 是 3.8）")
    else:
        bad("Python 版本过低，需要 3.8+")
    for mod, required, hint in (
            ("numpy", True, "pip3 install numpy"),
            ("yaml", True, "sudo apt install python3-yaml"),
            ("cv2", False, "sudo apt install python3-opencv（感知节点需要）"),
            ("serial", False, "pip3 install pyserial（真机夹爪需要）"),
            ("rclpy", False, "source /opt/ros/foxy/setup.bash（真机节点需要）")):
        try:
            __import__(mod)
            ok("模块 %-8s 可用" % mod)
        except ImportError:
            (bad if required else warn)("模块 %-8s 缺失 → %s" % (mod, hint))

    # ---------------- 2 配置 ----------------
    section("[2/7] 配置文件")
    try:
        from common import load_config, find_urdf, handeye_matrix, is_dry_run
        cfg = load_config(a.config)
        ok("配置加载成功：%s" % (a.config or "config/real_params.yaml"))
    except Exception as e:                        # noqa: BLE001
        bad("配置加载失败：%s" % e)
        return _summary()
    mode = cfg["profile"].get("mode")
    ok("profile.mode = %s%s" % (mode, "（演练，不动硬件）" if is_dry_run(cfg) else "（真机！）"))
    for k in ("robot", "arm", "camera", "handeye", "perception", "gripper", "task", "vla"):
        if k in cfg:
            ok("配置段 %-12s 存在" % k)
        else:
            bad("配置段 %-12s 缺失" % k)
    plc = cfg.get("place", {}) or {}
    if not plc:
        warn("没有 place 段 → 抓完不会放到指定区域（老配置，只有 8 阶段）")
    elif plc.get("enabled"):
        ok("配置段 place        存在且已启用：放置点 xy=%s surface_z=%.4f"
           % (plc.get("xy"), float(plc.get("surface_z", 0.0))))
        if float(plc.get("release_gap_m", 0.008)) > 0.02:
            warn("place.release_gap_m=%.3fm 偏大，方块落下会歪/弹" %
                 float(plc["release_gap_m"]))
    else:
        ok("配置段 place        存在但 enabled=false：抓完停在抬升终点")
    if not cfg["handeye"].get("calibrated"):
        warn("手眼标定未完成 → 真机抓取会被拒绝（这是安全设计，不是 bug）")
    else:
        # 标定矩阵的「一眼假」检查：这类错误打印矩阵时看不出来，但一上真机全错
        import numpy as np
        M = cfg["handeye"].get("matrix_4x4")
        if not M:
            bad("calibrated=true 但 matrix_4x4 是空的（感知会直接抛错）")
        else:
            X = np.array(M, dtype=float)
            if X.shape != (4, 4):
                bad("matrix_4x4 形状是 %s，必须是 4x4" % (X.shape,))
            else:
                t = X[:3, 3]
                R = X[:3, :3]
                tn = float(np.linalg.norm(t))
                if 0.02 < tn < 0.45:
                    ok("手眼矩阵平移模长 %.3fm（合理：法兰到相机 2~45cm）" % tn)
                else:
                    bad("手眼矩阵平移模长 %.3fm 明显不合理 → 单位错成 mm？A/B 定义反了？" % tn)
                if float(np.abs(R.T @ R - np.eye(3)).max()) < 1e-3:
                    ok("手眼矩阵旋转部分正交（不是缩放/剪切）")
                else:
                    bad("手眼矩阵旋转部分不是正交阵 → 输入四元数没归一化 / 抄错行")
                if abs(float(X[3, 3]) - 1.0) > 1e-6:
                    bad("matrix_4x4 第 4 行第 4 列应为 1.0，当前 %.3f" % float(X[3, 3]))
                # 视觉上最好懂的一条：这是相机在法兰系里的位置
                ok("相机在法兰系的位置 = [%.3f %.3f %.3f] m"
                   % (float(t[0]), float(t[1]), float(t[2])))
    fixed = cfg["task"].get("fixed_target_xyz")
    if fixed:
        warn("task.fixed_target_xyz 已填 %s → 走【示教兜底路径】，不再用视觉。"
             "方块挪位置后必须重新示教" % (fixed,))
    if (cfg.get("perception") or {}).get("xy_bias_m") not in (None, [0.0, 0.0]):
        warn("perception.xy_bias_m 非零 → 正在做常数偏置补偿；换标定后必须清零")
    _src = str((cfg.get("perception") or {}).get("target_source", "color")).lower()
    _b = cfg.get("brain") or {}
    if _src == "brain":
        if _b.get("enabled"):
            ok("感知目标来源 = 大脑（模型 %s @ %s）" % (_b.get("model"), _b.get("url")))
            warn("切 brain 前必须跑一次 brain.py --check 确认服务端通、延迟可接受")
        else:
            warn("target_source=brain 但 brain.enabled=false → 运行时会自动退回颜色阈值")
    else:
        ok("感知目标来源 = 颜色阈值（默认；brain.enabled=%s 留作可选）"
           % _b.get("enabled", False))
    if not cfg["gripper"].get("calibration_done"):
        warn("夹爪开口映射未标定 → 夹持开口只是粗估")
    if not cfg["profile"].get("allow_arm_motion"):
        warn("allow_arm_motion=false → 机械臂运动被第二道锁挡住")

    # ---------------- 3 URDF / FK ----------------
    section("[3/7] 运动学模型（URDF / FK）")
    urdf = None
    try:
        urdf = find_urdf(cfg)
        ok("找到 URDF：%s" % urdf)
    except Exception as e:                        # noqa: BLE001
        bad("URDF：%s" % e)
    kin = None
    if urdf:
        try:
            from common import setup_import_paths
            setup_import_paths(cfg)
            from ik_check import UrdfKinematics
            kin = UrdfKinematics(urdf, cfg["robot"]["base_frame"],
                                 cfg["robot"]["tip_frame"])
            import numpy as np
            T = kin.fk(np.asarray(cfg["robot"]["home_joints"], float))
            ok("FK 自检：home 末端位置 %s" %
               np.round(T[:3, 3], 4).tolist())
        except Exception as e:                    # noqa: BLE001
            bad("FK 自检失败：%s" % e)
            traceback.print_exc(limit=1)

    # ---------------- 3b 安全窗口 ----------------
    if kin is not None:
        section("[3b] 安全窗口（法兰能待在哪）")
        try:
            import numpy as np
            from common import cable_envelope, flange_position, safe_window
            env = cable_envelope(cfg)
            win = safe_window(cfg)
            anchor = flange_position(kin, np.asarray(cfg["robot"]["home_joints"], float))
            anchor_ok = True

            if env:
                lo, hi = env
                print("      绝对盒子：x %+.2f~%+.2f  y %+.2f~%+.2f  z %+.2f~%+.2f m"
                      % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
            else:
                print("      绝对盒子：未启用（cable_envelope_m: null）")

            if win:
                up, out, side = win
                print("      相对窗口：往上 ≤%.0fmm  往前 ≤%.0fmm  侧移 ≤%.0fmm（往下不限）"
                      % (up * 1000, out * 1000, side * 1000))
                print("      锚点 = home 法兰 [%.3f %.3f %.3f]（你实测、线是松的那个姿势）"
                      % (anchor[0], anchor[1], anchor[2]))
            else:
                warn("没配 safety.safe_window_m → 只做绝对盒子检查")

            for name, q, key in (("home", cfg["robot"]["home_joints"], "robot.home_joints"),
                                 ("observe", cfg["task"].get("observe_joints"), "task.observe_joints")):
                if not q:
                    continue
                f = flange_position(kin, np.asarray(q, float))
                dz, dx, dy = f[2] - anchor[2], f[0] - anchor[0], abs(f[1] - anchor[1])
                in_env = True
                if env:
                    lo, hi = env
                    in_env = not (np.any(f < lo) or np.any(f > hi))
                in_win = True
                if win:
                    up, out, side = win
                    in_win = (dz <= up + 1e-6) and (dx <= out + 1e-6) and (dy <= side + 1e-6)
                msg = ("%s 法兰 [%.3f %.3f %.3f]（相对锚点：上%+.0f 前%+.0f 侧%.0f mm）"
                       % (name, f[0], f[1], f[2], dz * 1000, dx * 1000, dy * 1000))
                if in_env and in_win:
                    ok(msg + " → 在窗口内")
                else:
                    bad_or_warn = msg + " → 超出窗口"
                    mode = str(((cfg.get("safety") or {}).get("envelope_mode", "warn"))).lower()
                    if mode == "block":
                        bad(bad_or_warn + "（envelope_mode=block：会被拒绝）")
                    else:
                        warn(bad_or_warn + "（envelope_mode=warn：只告警）")
            print("      💡 改 home/观察位前先跑 pose_view.py 看形状，再用 jog --joint 小步试")
        except Exception as e:                    # noqa: BLE001
            warn("窗口检查异常：%s" % e)

    # ---------------- 4 IK 规划 ----------------
    section("[4/7] IK 路径规划")
    try:
        import planner
        if planner.selftest(cfg):
            ok("IK 规划自检通过（路径连续、未贴限位）")
        else:
            bad("IK 规划自检失败（跳变过大或贴限位）")
    except Exception as e:                       # noqa: BLE001
        bad("IK 规划自检异常：%s" % e)
        traceback.print_exc(limit=1)

    # ---------------- 5 夹爪协议 ----------------
    section("[5/7] MS42DC 夹爪协议（离线核对帧/BCC/映射）")
    try:
        import ms42dc_driver
        if ms42dc_driver.selftest():
            ok("夹爪协议自测全部通过")
        else:
            bad("夹爪协议自测有失败项")
    except Exception as e:                       # noqa: BLE001
        bad("夹爪协议自测异常：%s" % e)
        traceback.print_exc(limit=1)

    # ---------------- 6 VLA 接口 ----------------
    section("[6/7] VLA 桥接")
    try:
        import vla_policy
        if vla_policy.selftest():
            ok("VLA 动作解析自测通过")
    except Exception as e:                       # noqa: BLE001
        bad("VLA 模块异常：%s" % e)
    if cfg["vla"].get("enabled"):
        warn("vla.enabled=true：只有腕部视角，VLA 效果不保证（见 17 号文档第 7 节）")
    else:
        ok("VLA 默认关闭，geometric 策略为默认（最稳）")

    # ---------------- 7 完整状态机演练 ----------------
    section("[7/7] 完整抓取状态机演练（无硬件）")
    if a.skip_dryrun:
        warn("已按 --skip-dryrun 跳过")
    elif kin is None:
        warn("URDF 不可用，跳过（先解决第 3 步）")
    else:
        try:
            import numpy as np
            from grasp_task import (GraspTask, StubNode, StubGripper,
                                    StubPerception)
            from arm_backend import make_backend
            arm_cfg = dict(cfg["arm"])
            arm_cfg["joints"] = list(cfg["robot"]["joints"])
            arm = make_backend(StubNode(), arm_cfg, dry_run=True)
            gripper = StubGripper(cfg)
            perception = StubPerception(cfg)
            # 演练时把观察位姿的起点设为 home，保证有反馈可用
            arm._remember(np.asarray(cfg["robot"]["home_joints"], float))
            task = GraspTask(cfg, kin, arm, gripper, perception,
                             log=lambda s: None, dry_run=True)
            done = task.run()
            if done:
                ok("状态机全部 %d 个阶段跑通（%d 条日志）"
                   % (11 if (cfg.get("place") or {}).get("enabled") else 8,
                      len(task.trace)))
                ok("未调用任何真实硬件接口")
            else:
                reason = task.trace[-2] if len(task.trace) >= 2 else "未知"
                bad("状态机演练中止：%s" % reason)
                for line in task.trace[-6:]:
                    print("      | %s" % line)
        except Exception as e:                   # noqa: BLE001
            bad("状态机演练异常：%s" % e)
            traceback.print_exc(limit=2)

    return _summary()


def _summary():
    print("\n" + "=" * 62)
    print("  自检汇总：✅ %d 项通过   ⚠️  %d 项待办   ❌ %d 项失败"
          % (len(PASS), len(WARN), len(FAIL)))
    print("=" * 62)
    if FAIL:
        print("\n必须先修这些：")
        for m in FAIL:
            print("  ❌ %s" % m)
    if WARN:
        print("\n接硬件前要补的（不是错误，是流程步骤）：")
        for m in WARN:
            print("  ⚠️  %s" % m)
    print("\n下一步（README第 5 步）：")
    print("  0. python3 net_autoconf.py        # 先把 arm_ip/udp_ip 配对好（/joint_states 没数就是它）")
    print("  1. udev 固定夹爪串口 + 拨码 2 号 ON")
    print("  2. 跑 calibrate_gripper.py --verify 再正式标定开口")
    print("  3. 按 25 号清单做手眼标定 → handeye_calib.py --write-config 自动写回配置")
    print("  4. 跑 teach_place_point.py 示教放置点（place.xy / surface_z）")
    print("  5. 把 profile.mode 改 real、allow_arm_motion 改 true（两道锁）")
    print("  6. bash restore_real.sh → ros2 launch arm_grasp real_grasp.launch.py")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())

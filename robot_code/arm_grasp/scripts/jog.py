#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""jog.py —— 键盘示教：不用示教器也能把机械臂挪到位

为什么需要它
------------
真机的 Ubuntu 不适配睿尔曼示教器（用户 2026-11-04 确认），
但整条链路里有 4 个地方原本都要"人把臂摆到位"：
  ① 手眼标定的起始位姿（让相机看得见 ArUco）
  ② 观察位姿 observe_joints
  ③ 放置点示教（指尖碰台面）
  ④ 打点验证时"把指尖微调到压住标记中心"
现在全部改成这个键盘工具，零额外硬件。

按键（笛卡尔模式，默认）
------------------------
  w / s       沿基座 x 前进 / 后退          a / d   沿基座 y 左移 / 右移
  r / f       抬升 / 下降                   1..6    步长 0.5 / 1 / 5 / 10 / 20 / 50 mm
  i / k       绕工具 x 轴俯仰 ±              j / l   绕工具 z 轴偏航 ±
  u / o       绕工具 y 轴滚转 ±             +/-     旋转步长 2° / 5° / 10°
  g           打印当前位姿（关节角 + 指尖 xyz）
  c           打印可直接粘进 real_params.yaml 的片段（观察位 / 放置点都用它）
  h           回 home（会二次确认，速度 8）
  x           轨迹急停（/rm_driver/move_stop_cmd）
  q           退出

  --joint 模式下：1..6 选关节，w/s 微调该关节，i/k 也可以

安全设计
--------
  · 每次只走一个步长；单步关节跳变 > 0.6 rad 直接拒绝并提示减小步长
  · 速度强制 ≤ 10（比抓取任务还慢）
  · 需要两道锁：profile.mode=real + allow_arm_motion=true
  · 起手先显示当前位姿，按任意键才第一次动

用法
----
  python3 robot_code/arm_grasp/scripts/jog.py
  python3 robot_code/arm_grasp/scripts/jog.py --step 1        # 精调 1mm
  python3 robot_code/arm_grasp/scripts/jog.py --joint          # 关节模式
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import tty

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

STEP_MM = [0.5, 1.0, 5.0, 10.0, 20.0, 50.0]
STEP_DEG = [2.0, 5.0, 10.0]

HELP = """
笛卡尔模式（工具姿态保持不变，只平移/绕自身转）
  w/s  x±    a/d  y±    r/f  z±        ← 平移
  i/k  俯仰±  u/o  滚转±  j/l  偏航±   ← 绕工具轴转
  1~6  步长 0.5/1/5/10/20/50 mm       +/-  旋转步长 2/5/10°
  g 打印位姿   c 打印配置片段   h 回home   x 急停   q 退出
"""


def _rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Jogger:
    """把「当前位姿 + 一个增量」变成关节角，交给 arm 后端下发。

    独立成类是为了给 verify_handeye.py 复用（那里要 1mm 级微调）。
    """

    def __init__(self, cfg, kin, arm, log=print):
        self.cfg = cfg
        self.kin = kin
        self.arm = arm
        self.log = log
        self.offset = np.asarray(cfg["robot"]["tcp_offset"], float)
        self.lo = np.asarray(cfg["robot"]["joint_limits_lo"], float)
        self.hi = np.asarray(cfg["robot"]["joint_limits_hi"], float)

    # ---------------------------------------------------------- 位姿
    def pose(self):
        q = self.arm.current_joints()
        if q is None:
            raise RuntimeError("收不到 /joint_states（先跑 net_autoconf.py）")
        T = np.asarray(self.kin.fk(q), float)
        R, p = T[:3, :3], T[:3, 3]
        tip = R @ self.offset + p
        return q, R, tip

    # ---------------------------------------------------------- 移动
    def translate(self, delta, max_jump=0.6):
        """沿基座系平移 delta（米），姿态不变。"""
        q, R, tip = self.pose()
        target_tip = tip + np.asarray(delta, float)
        return self._goto(q, target_tip, R, max_jump)

    def rotate(self, axis, deg, max_jump=0.6):
        """绕【工具自身轴】转 deg 度，指尖位置不动（相机绕指尖转）。"""
        q, R, tip = self.pose()
        R2 = R @ _rot(axis, deg)
        return self._goto(q, tip, R2, max_jump)

    def joint_delta(self, idx, deg, max_jump=0.6):
        q, R, tip = self.pose()
        q2 = q.copy()
        q2[idx] += np.radians(deg)
        if np.any(q2 < self.lo) or np.any(q2 > self.hi):
            return False, "关节 %d 会超限位" % (idx + 1)
        return self.arm.move_joints([q, q2], [0.0, 2.5])

    def _goto(self, q0, tip_target, R_target, max_jump):
        from ik_check import dls_ik
        p_link6 = tip_target - R_target @ self.offset
        q, ok, err = dls_ik(self.kin, q0, p_link6, R_target,
                            max_iter=400, tol=1e-4, lam=1e-3, max_step=0.12)
        if not ok or err > 5e-3:
            return False, "IK 无解（err=%.4f）：这个方向到头了，换个方向" % err
        if np.any(q < self.lo) or np.any(q > self.hi):
            return False, "目标位姿超关节限位"
        jump = float(np.max(np.abs(q - q0)))
        if jump > max_jump:
            return False, ("单步关节跳变 %.2f rad 太大（会突然翻腕），"
                           "把步长调小" % jump)
        # 线缆包络：约束【法兰】。规则：
        #   · 当前在盒内、这一步要出去 → 拒绝（拽线）
        #   · 当前已经在盒外（比如出厂停在高处）→ 允许，只是提醒"正在往里收"
        from common import cable_envelope, flange_position
        env = cable_envelope(self.cfg)
        if env is not None:
            lo, hi = env
            f_new = flange_position(self.kin, q)
            f_old = flange_position(self.kin, q0)
            over_new = np.maximum(lo - f_new, f_new - hi)
            over_old = np.maximum(lo - f_old, f_old - hi)
            out_new = float(np.max(over_new))
            out_old = float(np.max(over_old))
            if out_new > 1e-6 and out_old <= 1e-6:
                k = int(np.argmax(over_new))
                return False, ("超线缆包络：法兰 %s 方向超 %.0fmm"
                               "（允许 x %+.2f~%+.2f y %+.2f~%+.2f z %+.2f~%+.2f）"
                               % ("xyz"[k], over_new[k] * 1000,
                                  lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
            if out_new > 1e-6:
                self.log("   ⚠️ 当前位姿在包络外（超 %.0fmm），这一步允许，"
                         "但请继续往盒内收" % (out_new * 1000))
        return self.arm.move_joints([q0, q], [0.0, 2.5])

    # ---------------------------------------------------------- 输出
    def describe(self):
        q, R, tip = self.pose()
        f = np.asarray(self.kin.fk(q), float)[:3, 3]
        txt = ("关节 [%s]\n  法兰 [%.3f %.3f %.3f]   指尖 [%.3f %.3f %.3f]"
               % (", ".join("%+.3f" % v for v in q),
                  f[0], f[1], f[2], tip[0], tip[1], tip[2]))
        from common import cable_envelope
        env = cable_envelope(self.cfg)
        if env is not None:
            lo, hi = env
            margin = float(np.min(np.minimum(f - lo, hi - f)))
            txt += ("\n  离线缆包络 %s %+.0f mm"
                    % ("最近" if margin >= 0 else "已超", margin * 1000))
        return txt

    def yaml_snippet(self):
        q, R, tip = self.pose()
        arr = "[%s]" % ", ".join("%.4f" % v for v in q)
        lines = ["  # 同一个关节角，两个地方都能用（观察位 / 回零停放位）",
                 "  #   task.observe_joints",
                 "  observe_joints: " + arr,
                 "  #   robot.home_joints",
                 "  home_joints: " + arr,
                 "",
                 "  # 指尖当前位置（放置点用：指尖贴台面时，xy=place.xy / z=surface_z）",
                 "  # xy: [%.4f, %.4f]   surface_z: %.4f" % (tip[0], tip[1], tip[2])]
        return "\n".join(lines)


class _FakeArm:
    """只回答 current_joints()，用于"预览下一段姿态"而不动真机。"""

    def __init__(self, q):
        self._q = np.asarray(q, float)

    def current_joints(self):
        return self._q.copy()

    def move_joints(self, *a, **k):
        return True, "fake"

    def stop(self):
        pass


# ---------------------------------------------------------------- 终端
def _getch(timeout=None):
    """读一个键（非阻塞/带超时）。返回 '' 表示超时。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if timeout is None:
            return sys.stdin.read(1)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(argv=None):
    ap = argparse.ArgumentParser(description="键盘示教（替代示教器）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--step", type=float, default=5.0, help="初始平移步长 mm（默认 5）")
    ap.add_argument("--joint", action="store_true", help="关节模式")
    ap.add_argument("--steps", type=int, default=10,
                    help="--goto 分几段走（每段都要回车，默认 10）")
    ap.add_argument("--goto", default=None,
                    help='慢速走到指定关节角（先 pose_view 看过再动），'
                         '如 --goto "0.2988,-0.1751,1.2633,0.4825,-1.5708,0.2988"')
    ap.add_argument("--speed", type=int, default=5,
                    help="速度 1~100（强制 ≤10）。默认 5，熟练后可到 10")
    a = ap.parse_args(argv)

    from common import load_config, setup_import_paths, find_urdf, is_dry_run
    cfg = load_config(a.config)
    setup_import_paths(cfg)
    from ik_check import UrdfKinematics

    if is_dry_run(cfg) or not cfg["profile"].get("allow_arm_motion", False):
        print("❌ 本工具会动机械臂，需要两道锁：")
        print("   profile.mode: real   +   profile.allow_arm_motion: true")
        print("   改完保存，再跑一次。")
        return 2

    import rclpy
    import threading
    from rclpy.node import Node

    class _N(Node):
        pass

    rclpy.init()
    node = _N("jog")
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    from arm_backend import make_backend
    arm_cfg = dict(cfg["arm"])
    arm_cfg["joints"] = list(cfg["robot"]["joints"])
    arm_cfg["speed"] = max(1, min(int(a.speed), 10))     # 强制低速
    arm = make_backend(node, arm_cfg, dry_run=False)

    kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                         cfg["robot"]["tip_frame"])
    jg = Jogger(cfg, kin, arm)

    print("=" * 62)
    print(" 键盘示教  速度=%d  步长=%.1fmm%s" %
          (arm_cfg["speed"], a.step, "  关节模式" if a.joint else ""))
    print("=" * 62)
    print(" 等关节角…")
    import time
    t0 = time.time()
    while arm.current_joints() is None and time.time() - t0 < 10:
        time.sleep(0.2)
    if arm.current_joints() is None:
        print("❌ 收不到 /joint_states：先跑 net_autoconf.py 把 udp_ip 配对好")
        return 1
    print(" 当前位姿：\n " + jg.describe())

    # ---- --goto：分段慢速走到指定关节角（每段都要你按回车，随时能停）----
    # ★ 2026-11-09 改：原来一口气走 8 段，中途有个姿态把线绷紧了（现场照片）。
    #   关节空间插值的【中间姿态】可能比目标姿态更糟，所以改成：
    #   每段先打印"下一点长什么样 + 法兰在哪 + 离线缆多远"，你按回车才走这一段。
    if a.goto:
        try:
            qt = np.array([float(v) for v in a.goto.replace("，", ",").split(",")], float)
            assert qt.size == 6
        except Exception:                          # noqa: BLE001
            print("❌ --goto 需要 6 个关节角，用逗号分隔")
            return 2
        q0 = arm.current_joints()
        print("\n 目标关节 %s" % np.round(qt, 3).tolist())
        print(" 关节变化(度) %s" % [round(float(np.degrees(d)), 1) for d in (qt - q0)])
        print(" ⚠️ 分 %d 段，速度 %d%%，【每段都要回车】；按 q 停下，按 x 轨迹急停"
              % (a.steps, arm_cfg["speed"]))
        try:
            if input(" 确认请输入 yes 回车：").strip().lower() != "yes":
                print(" 已取消")
                return 0
        except (EOFError, KeyboardInterrupt):
            return 0
        cur = np.asarray(q0, float)
        for i in range(1, a.steps + 1):
            qq = np.asarray(q0, float) + (qt - np.asarray(q0, float)) * i / a.steps
            # 下一段的预览（不动机器）
            prev = Jogger(self.cfg, kin, arm)
            prev.arm = _FakeArm(qq)
            print("\n --- 第 %d/%d 段预览 ---" % (i, a.steps))
            print(" " + prev.describe().replace("\n", "\n "))
            try:
                k = input(" 回车=走这一段 | q=停 | x=急停 > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                k = "q"
            if k == "x":
                arm.stop()
                print(" 已发轨迹急停")
                break
            if k == "q":
                print(" 已停在当前位姿")
                break
            okm, msgm = arm.move_joints([cur, qq], [0.0, 2.5])
            print("   %s" % ("✓ 到位" if okm else "✗ " + msgm))
            if not okm:
                break
            cur = qq
        print("\n 当前位姿：\n " + jg.describe())
        print(" 满意就把这行抄进配置：\n  home_joints: %s" % np.round(cur, 4).tolist())
        return 0

    print(HELP)

    step_mm = float(a.step)
    step_deg = STEP_DEG[0]
    jidx = 0
    try:
        while True:
            k = _getch()
            if k == "":
                continue
            if k in ("q", "Q", "\x03", "\x04"):
                break
            if k in "123456":
                if a.joint:
                    jidx = int(k) - 1
                    print(" 选中关节 %d" % (jidx + 1))
                else:
                    step_mm = STEP_MM[int(k) - 1]
                    print(" 步长 = %.1f mm" % step_mm)
                continue
            if k == "+":
                step_deg = STEP_DEG[(STEP_DEG.index(step_deg) + 1) % len(STEP_DEG)]
                print(" 旋转步长 = %.0f°" % step_deg)
                continue
            if k in ("g", "G"):
                print(" " + jg.describe().replace("\n", "\n "))
                continue
            if k in ("c", "C"):
                print(jg.yaml_snippet())
                continue
            if k in ("x", "X"):
                arm.stop()
                print(" ⛔ 已发轨迹急停（夹爪保持）")
                continue
            if k in ("h", "H"):
                print(" 回 home？会走关节空间，确认输入 y：", end="", flush=True)
                if _getch() in ("y", "Y"):
                    q = arm.current_joints()
                    ok, msg = arm.move_joints(
                        [q, np.asarray(cfg["robot"]["home_joints"], float)],
                        [0.0, 5.0])
                    print(" 回 home: %s" % msg)
                else:
                    print(" 已取消")
                continue

            if a.joint:
                if k in ("w", "s"):
                    d = step_deg if k == "w" else -step_deg
                    ok, msg = jg.joint_delta(jidx, d)
                    print((" 关节%d %+.1f° → %s" % (jidx + 1, d, msg)))
                else:
                    print(" 关节模式只认 w/s（调当前关节）、1~6（选关节）")
                continue

            mm = step_mm / 1000.0
            moved = None
            if k == "w":
                moved = jg.translate([mm, 0, 0])
            elif k == "s":
                moved = jg.translate([-mm, 0, 0])
            elif k == "a":
                moved = jg.translate([0, mm, 0])
            elif k == "d":
                moved = jg.translate([0, -mm, 0])
            elif k == "r":
                moved = jg.translate([0, 0, mm])
            elif k == "f":
                moved = jg.translate([0, 0, -mm])
            elif k == "i":
                moved = jg.rotate("x", step_deg)
            elif k == "k":
                moved = jg.rotate("x", -step_deg)
            elif k == "u":
                moved = jg.rotate("y", step_deg)
            elif k == "o":
                moved = jg.rotate("y", -step_deg)
            elif k == "j":
                moved = jg.rotate("z", step_deg)
            elif k == "l":
                moved = jg.rotate("z", -step_deg)
            if moved is not None:
                ok, msg = moved
                if ok:
                    print(" ✓ " + jg.describe().replace("\n", "\n   "))
                else:
                    print(" ✗ %s" % msg)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n退出。夹爪与机械臂保持当前位置（不会自动回 home）。")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

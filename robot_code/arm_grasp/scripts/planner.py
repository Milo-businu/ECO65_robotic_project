#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取规划（纯 numpy，不依赖 ROS）—— 仿真里验证过的那套 IK，搬到真机用

复用了 sim_grasp/scripts/eff_test.py 的两个关键结论：
  ① 工具 z 轴 = 夹爪纵轴（沿末端旋转轴同轴延伸；长度由 robot.tcp_offset 配置，
     真机实测 17.7cm，仿真 URDF 里是 19.5cm，两者独立）
     → 姿态约束是「工具 z 轴指向下方」，IK 里 down 放旋转矩阵第三列
  ② 关节空间插值会横扫撞飞方块 → 必须用【密集笛卡尔直线路径点】

坐标约定（和手眼标定必须一致，否则整体偏一个固定量）：
  p_base = T_base←Link6(q) · X · p_camera_optical
  X = T_flange←camera_optical（handeye_calib.py 的输出）
  ⚠️ 标定时用的 flange frame 必须等于 FK 的 tip_frame（本项目是 Link6）
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np


DOWN = np.array([0.0, 0.0, -1.0])


def ik_aligned(kin, q_init, p_tip, offset, lo, hi,
               err_max: float = 5e-3, twist_penalty: float = 0.0):
    """工具 z 轴指向下方，绕工具轴搜 4 个朝向。

    ⚠️ 选择准则是「离 q_init 最近」，不是「关节角绝对值最小」。
       4 个朝向对应绕工具轴 0/90/180/270° 四组解，若每步都取全局最小，
       相邻路径点会在不同分支间来回跳（实测 joint6 跳 1.571 rad = 90°）。
       锁住"离上一步最近"的分支，下降段才连续（等价于仿真 ik_with_roll
       的 phi_center/phi_span 连续性约束）。

    返回 (q, R) 或 None。
    """
    q0 = np.asarray(q_init, float)
    best = None
    for xg in (np.array([1.0, 0, 0]), np.array([-1.0, 0, 0]),
               np.array([0, 1.0, 0]), np.array([0, -1.0, 0])):
        yg = np.cross(DOWN, xg)          # 右手系：xg × yg = down
        R = np.column_stack([xg, yg, DOWN])
        q, ok, err = kin.dls_ik(q0, np.asarray(p_tip, float) - R @ offset, R) \
            if hasattr(kin, "dls_ik") else _dls_ik(kin, q0,
                                                   np.asarray(p_tip, float) - R @ offset, R)
        if not ok or err > err_max:
            continue
        if any(x < l or x > h for x, l, h in zip(q, lo, hi)):
            continue
        # ⚠️ 2026-11-09 教训：别用"绝对 |J6| 小"来选分支。
        #   现场实测的安全 home 本身 J6=-1.53（-88°），如果按 |J6| 最小去挑，
        #   会从 -88° 硬转到 +17°，等于多拧 105° —— 反而更容易拽线。
        #   真正该保证的是"J6 不要相对【起始姿态】突跳"，这一点 norm(q-q0)
        #   已经包含了（链式跟随）。所以 twist_penalty 默认 0，只在
        #   "home 工具水平、任务工具朝下"这种特殊场合才手动开。
        score = (float(np.linalg.norm(np.asarray(q, float) - q0))
                 + float(twist_penalty) * abs(float(q[5])))
        if best is None or score < best[0]:
            best = (score, np.asarray(q, float), R)
    return (best[1], best[2]) if best else None


def _dls_ik(kin, q_init, p_des, R_des, max_iter=500, tol=1e-4,
            lam=1e-3, max_step=0.15):
    """从 arm_control/scripts/ik_check.py 导入；这里只做一层转发，避免重复实现。"""
    from ik_check import dls_ik
    return dls_ik(kin, q_init, p_des, R_des, max_iter=max_iter,
                  tol=tol, lam=lam, max_step=max_step)


def max_joint_jump(waypoints: Sequence[Sequence[float]]) -> float:
    """相邻路径点最大关节跳变（弧度）。用来提前发现「突然翻腕」。"""
    if len(waypoints) < 2:
        return 0.0
    return max(float(np.max(np.abs(np.asarray(waypoints[i], float)
                                  - np.asarray(waypoints[i - 1], float))))
               for i in range(1, len(waypoints)))


def tip_position(kin, q, offset) -> np.ndarray:
    """TCP（指尖中点）在基座系的位置。

    p_tip = T_base←Link6(q) · offset
    注意 offset 是【工具坐标系】里的 [0,0,0.177]，所以不能用 T[:3,3] + offset，
    必须先用旋转矩阵转一次：p_tip = R·offset + p_link6。
    夹爪不总是竖直向下（比如观察位姿），这个区别就会体现出来。
    """
    T = np.asarray(kin.fk(np.asarray(q, float)), float)
    return T[:3, :3] @ np.asarray(offset, float) + T[:3, 3]


def _chain(kin, q_start, p_from, p_to, n, offset, lo, hi, tag):
    """把一条三维直线 p_from→p_to 切成 n 段，逐段 IK，返回 n 个路径点（不含起点）。

    这是"笛卡尔直线"的最小实现：两点之间按比例插值位置，姿态固定为工具 z 向下，
    每步都以上一步的解作为 IK 初值（ik_aligned 内部选"离初值最近"的分支），
    所以整条路径关节连续，不会中途翻腕。
    """
    wps = []
    q = np.asarray(q_start, float)
    p_from = np.asarray(p_from, float)
    p_to = np.asarray(p_to, float)
    for i in range(1, n + 1):
        p = p_from * (1.0 - i / float(n)) + p_to * (i / float(n))
        s = ik_aligned(kin, q, p, offset, lo, hi)
        if s is None:
            raise RuntimeError(
                "%s 第 %d/%d 点无 IK 解（目标 [%.3f %.3f %.3f]）。"
                "放置点太远/太低，或 approach_height / retreat_height 设得太高。"
                % (tag, i, n, p[0], p[1], p[2]))
        q = s[0]
        wps.append(q)
    return wps


# ---------------------------------------------------------------- 线缆包络
def envelope_report(kin, waypoints, cfg, skip_first=False):
    """逐个路径点检查【法兰】是否在线缆包络内。

    返回 [(序号, 法兰xyz, 各轴超出量), ...]，空表示全在盒内。
    ⚠️ 判据是法兰，不是指尖（工具竖直时法兰高 0.177m）。
    """
    from common import cable_envelope, flange_position
    env = cable_envelope(cfg)
    if env is None:
        return []
    lo, hi = env
    out = []
    wps = waypoints[1:] if skip_first else waypoints
    base = 1 if skip_first else 0
    for i, q in enumerate(wps):
        f = flange_position(kin, q)
        over = np.maximum(lo - f, f - hi)
        if float(np.max(over)) > 1e-6:
            out.append((i + base, f, over))
    return out


def window_report(kin, waypoints, cfg, skip_first=False):
    """相对【home 法兰锚点】的安全窗口检查（往上/往外/往侧）。

    抓取任务全程在锚点下方，所以"往下"不限制 —— 只挡会把相机线拉直的方向。
    返回 [(序号, 法兰, 说明)]。
    """
    from common import safe_window, flange_position
    w = safe_window(cfg)
    if w is None:
        return []
    up, out, side = w
    anchor = flange_position(kin, cfg["robot"]["home_joints"])
    out_list = []
    wps = waypoints[1:] if skip_first else waypoints
    base = 1 if skip_first else 0
    for i, q in enumerate(wps):
        f = flange_position(kin, q)
        dz, dx, dy = f[2] - anchor[2], f[0] - anchor[0], abs(f[1] - anchor[1])
        why = []
        if dz > up + 1e-6:
            why.append("比 home 高 %.0fmm（限 %.0f）" % (dz * 1000, up * 1000))
        if dx > out + 1e-6:
            why.append("比 home 前伸 %.0fmm（限 %.0f）" % (dx * 1000, out * 1000))
        if dy > side + 1e-6:
            why.append("比 home 侧移 %.0fmm（限 %.0f）" % (dy * 1000, side * 1000))
        if why:
            out_list.append((i + base, f, "；".join(why)))
    return out_list


def assert_envelope(kin, waypoints, cfg, tag, skip_first=False):
    """安全围栏：绝对盒子（若配了）+ 相对 home 的安全窗口。

    safety.envelope_mode = block（拒绝）/ warn（只告警，默认）。
    现场依据：相机 USB 线只能在身前垂圈，往上/往外会把圈拉直。
    """
    mode = str(((cfg or {}).get("safety") or {}).get("envelope_mode", "warn")).lower()
    msgs = []

    bad = envelope_report(kin, waypoints, cfg, skip_first=skip_first)
    if bad:
        i, f, over = bad[0]
        k = int(np.argmax(over))
        from common import cable_envelope
        lo, hi = cable_envelope(cfg)
        msgs.append("%s：第 %d 个路径点的法兰 [%.3f %.3f %.3f] 超出线缆包络"
                    "（%s 方向超 %.0fmm，允许 %+.2f~%+.2f）共 %d 点越界。"
                    % (tag, i, f[0], f[1], f[2], "xyz"[k], over[k] * 1000,
                       lo[k], hi[k], len(bad)))

    win = window_report(kin, waypoints, cfg, skip_first=skip_first)
    if win:
        i, f, why = win[0]
        msgs.append("%s：第 %d 个路径点的法兰 [%.3f %.3f %.3f] 超出安全窗口"
                    "（%s）共 %d 点越界。"
                    % (tag, i, f[0], f[1], f[2], why, len(win)))

    if not msgs:
        return
    text = "\n".join(msgs)
    if mode != "block":
        print("⚠️  " + text + "\n   （envelope_mode=warn：继续执行，但请确认相机线没被拉直）")
        return
    raise RuntimeError(
        text + "\n  这不是算法问题：线不够长/线不能改走线。"
              "\n  处理：改目标或姿态让它落在窗口内；"
              "\n  或把 safety.envelope_mode 改成 warn（只告警）后自己盯线。")

def plan_grasp(kin, q_now, target_top_center, cfg, obj_half=None) -> dict:
    """生成完整抓取路径。

    target_top_center : 感知给出的【方块顶面中心】[x,y,z]（base_link 系）
    返回 dict：
      approach_waypoints / approach_times   观察位 → 高点 → 直线下降到抓取点
      grasp_q                             抓取关节角
      lift_waypoints / lift_times          直线抬升
      center                              方块中心（顶面 z 减去半边长）
      diagnostics                         限位余量 / 关节跳变等
    """
    robot = cfg["robot"]
    task = cfg["task"]
    offset = np.asarray(robot["tcp_offset"], float)
    lo = np.asarray(robot["joint_limits_lo"], float)
    hi = np.asarray(robot["joint_limits_hi"], float)
    half = float(obj_half) if obj_half else float(cfg["perception"]["box_half_size_m"])
    center = np.asarray(target_top_center, float) - np.array([0.0, 0.0, half])

    diag = {"center": center.tolist(), "jump": [], "limit_margin": []}

    # 观察位姿直接由配置给【关节角】，不做 IK：
    # 真机上是"用 jog.py 摆好、读 /joint_states 填进来"的姿势，本来就没有对应的
    # "工具竖直向下"笛卡尔约束，硬做 IK 只会无解（preflight 实测踩到）。
    observe_q = np.asarray(task.get("observe_joints",
                                    robot["home_joints"]), float)

    # ---- 高点（方块正上方）----
    top_z = center[2] + float(task["approach_height_m"])
    s_top = ik_aligned(kin, q_now, np.array([center[0], center[1], top_z]),
                       offset, lo, hi)
    if s_top is None:
        raise RuntimeError("高点无 IK 解：TCP 抬太高会无解（tcp_offset=%.3fm），"
                           "把 task.approach_height_m 调小（建议 0.14~0.16）"
                           % offset[2])

    # 观察位姿单独由状态机执行（要先看清目标再规划），这里只生成
    # 「当前 → 高点 → 直线下降」的路径，避免重复下发同一个姿势。
    wps = [np.asarray(q_now, float), s_top[0]]
    ts = [0.0, 4.0]
    cur = s_top[0]

    # ---- 笛卡尔直线下降（密集路径点，避免关节空间横扫）----
    n = int(task.get("descend_steps", 6))
    for i in range(1, n + 1):
        z = top_z + (center[2] - top_z) * i / n
        s = ik_aligned(kin, cur, np.array([center[0], center[1], z]),
                       offset, lo, hi)
        if s is None:
            raise RuntimeError("下降第 %d 步无 IK 解" % i)
        cur = s[0]
        wps.append(cur)
        ts.append(4.0 + 2.0 * i)
    grasp_q = cur

    # ---- 直线抬升 ----
    m = int(task.get("lift_steps", 5))
    lift_h = float(task.get("lift_height_m", 0.10))
    lwps = [grasp_q]
    lts = [0.0]
    c2 = grasp_q
    for i in range(1, m + 1):
        z = center[2] + lift_h * i / m
        s = ik_aligned(kin, c2, np.array([center[0], center[1], z]),
                       offset, lo, hi)
        if s is None:
            break
        c2 = s[0]
        lwps.append(c2)
        lts.append(2.0 * i)

    # 第一跳（当前姿态 → 高点）是"自然位姿翻到工具竖直向下"的必然大动作，
    # 属于正常，单独报；判连续性只看高点之后的【下降段】。
    diag["initial_jump"] = float(max_joint_jump(wps[:2]))
    diag["jump"] = float(max_joint_jump(wps[1:]))
    diag["limit_margin"] = [float(min(np.minimum(np.asarray(w) - lo,
                                                 hi - np.asarray(w))))
                            for w in wps]
    diag["min_limit_margin"] = min(diag["limit_margin"]) if diag["limit_margin"] else 0.0
    diag["descend_points"] = n
    diag["lift_points"] = len(lwps) - 1

    # 线缆包络：抓取全过程（下降 + 抬升）的法兰都要在盒内
    assert_envelope(kin, wps, cfg, "[抓取下降段]", skip_first=True)
    assert_envelope(kin, lwps, cfg, "[抓取抬升段]")

    return {
        "approach_waypoints": wps,
        "approach_times": ts,
        "observe_q": observe_q,
        "grasp_q": grasp_q,
        "lift_waypoints": lwps,
        "lift_times": lts,
        "center": center,
        "diagnostics": diag,
    }


def plan_place(kin, q_start, cfg, obj_half=None) -> dict:
    """生成「夹着方块 → 放到固定区域」的路径（不靠视觉，坐标来自配置）。

    为什么放着不直接 movej 过去
    --------------------------
    夹着东西走关节空间 = 方块沿弧线飞过去，中途可能扫地、撞台面、撞别的方块。
    所以照搬抓取那套思路，拆成三段【笛卡尔直线】：

        当前（抬升终点）
          │ ① 原地竖直抬到转运高度 travel_z
          │
          ├──── ② 水平平移到放置点正上方（同一高度）
          │
          ▼ ③ 竖直下降到释放高度 release_z
        张开 → ④ 竖直抬到撤离高度 retreat_z →（可选）回 home

    ①②③ 全部由 _chain() 逐点 IK 生成，姿态固定「工具 z 竖直向下」，
    分支锁定在"离上一步最近"，所以整条路径关节连续。

    高度约定（这是最容易搞错的地方）
    ------------------------------
      surface_z    放置台面高度（方块底面最终落在这一层）
      center_z     = surface_z + 半边长     ← 方块中心 / TCP 目标
      release_z    = center_z + gap        ← 实际下降到这里就停，给台面留余量
    因为真机夹持点时 TCP 就压在方块中心，所以 TCP 到 center_z 时方块底面正好贴台面。

    返回
    ----
      transfer_waypoints / transfer_times   抬升终点 → 放置点上方 → 释放点
      release_q / release_xyz               释放瞬间的关节角 / TCP 位置
      retreat_waypoints / retreat_times     释放点 → 撤离高度
      place_center                          方块最终中心（给日志用）
      diagnostics                           jump / limit_margin / 各段点数
    """
    pcfg = cfg["place"]
    robot = cfg["robot"]
    if "xy" not in pcfg:
        raise RuntimeError(
            "place.xy 没配。先跑一次示教：\n"
            "  python3 robot_code/arm_grasp/scripts/teach_place_point.py\n"
            "把指尖移到放置区中心、刚好碰到台面，回车后它会把 xy / surface_z 打印出来。")
    xy = np.asarray(pcfg["xy"], float).reshape(-1)
    if xy.size < 2:
        raise RuntimeError("place.xy 需要 2 个数（x, y），当前 %s" % pcfg["xy"])
    xy = xy[:2]
    offset = np.asarray(robot["tcp_offset"], float)
    lo = np.asarray(robot["joint_limits_lo"], float)
    hi = np.asarray(robot["joint_limits_hi"], float)
    half = float(obj_half) if obj_half else float(cfg["perception"]["box_half_size_m"])

    surface_z = float(pcfg.get("surface_z", 0.0))
    gap = float(pcfg.get("release_gap_m", 0.008))
    center_z = surface_z + half
    release_z = center_z + gap
    travel_z = surface_z + float(pcfg.get("approach_height_m", 0.18))
    retreat_z = surface_z + float(pcfg.get("retreat_height_m", 0.20))

    n_up = int(pcfg.get("up_steps", 3))
    n_across = int(pcfg.get("across_steps", 6))
    n_down = int(pcfg.get("descend_steps", 6))
    n_retreat = int(pcfg.get("retreat_steps", 4))

    q0 = np.asarray(q_start, float)
    p0 = tip_position(kin, q0, offset)

    # 转运高度必须既高过当前指尖（否则"先抬后移"变成往下走），
    # 又高过释放点（否则没有下降段可走）。两条都满足不了就自动顶高并说明原因。
    notes = []
    if travel_z < p0[2]:
        notes.append("转运高度 %.3f 低于当前指尖 %.3f，就地顶高" % (travel_z, p0[2]))
        travel_z = float(p0[2])
    if travel_z < release_z + 0.04:
        notes.append("转运高度低于释放点+4cm，抬到 %.3f" % (release_z + 0.04))
        travel_z = release_z + 0.04
    if retreat_z < travel_z:
        retreat_z = travel_z

    transfer = []
    transfer += _chain(kin, q0, p0, np.array([p0[0], p0[1], travel_z]),
                       n_up, offset, lo, hi, "[place①竖直抬升]")
    q1 = transfer[-1] if transfer else q0
    transfer += _chain(kin, q1, np.array([p0[0], p0[1], travel_z]),
                       np.array([xy[0], xy[1], travel_z]),
                       n_across, offset, lo, hi, "[place②水平转运]")
    q2 = transfer[-1] if transfer else q0
    transfer += _chain(kin, q2, np.array([xy[0], xy[1], travel_z]),
                       np.array([xy[0], xy[1], release_z]),
                       n_down, offset, lo, hi, "[place③竖直下降]")
    release_q = transfer[-1] if transfer else q0

    retreat = _chain(kin, release_q, np.array([xy[0], xy[1], release_z]),
                     np.array([xy[0], xy[1], retreat_z]),
                     n_retreat, offset, lo, hi, "[place④撤离抬升]")

    diag = {
        "jump": float(max_joint_jump(transfer)),
        "retreat_jump": float(max_joint_jump([release_q] + retreat)),
        "limit_margin": [float(min(np.minimum(np.asarray(w) - lo, hi - np.asarray(w))))
                         for w in (transfer + retreat)],
        "up_points": n_up, "across_points": n_across,
        "down_points": n_down, "retreat_points": len(retreat),
        "travel_z": travel_z, "release_z": release_z, "retreat_z": retreat_z,
        "distance_m": float(np.hypot(xy[0] - p0[0], xy[1] - p0[1])),
        "notes": notes,
    }
    diag["min_limit_margin"] = min(diag["limit_margin"]) if diag["limit_margin"] else 0.0

    assert_envelope(kin, transfer, cfg, "[转运到放置区]")
    assert_envelope(kin, retreat, cfg, "[释放后撤离]")

    return {
        "transfer_waypoints": transfer,
        "transfer_times": [2.0 * i for i in range(1, len(transfer) + 1)],
        "release_q": release_q,
        "release_xyz": [float(xy[0]), float(xy[1]), release_z],
        "retreat_waypoints": retreat,
        "retreat_times": [1.5 * i for i in range(1, len(retreat) + 1)],
        "place_center": [float(xy[0]), float(xy[1]), center_z],
        "diagnostics": diag,
    }


def selftest(cfg) -> bool:
    """离线验证：用真实 URDF 跑一遍 FK/IK/路径规划，不需要任何硬件。"""
    import os
    import sys

    urdf = cfg["robot"].get("urdf") or os.environ.get("ECO65_URDF", "")
    if not urdf:
        for p in ("/tmp/eco65_6fb.urdf",
                  os.path.expanduser("~/robot_vla_ws/src/sim_grasp/config/eco65_6fb.urdf"),
                  os.path.expanduser("~/robot_vla_ws/robot_code/sim_grasp/config/eco65_6fb.urdf")):
            if os.path.isfile(p):
                urdf = p
                break
    if not urdf or not os.path.isfile(urdf):
        print("  ⚠️ 找不到 URDF，跳过 IK 自检（真机上先跑 `urdf` 别名生成）")
        return True

    for d in (os.path.expanduser("~/robot_vla_ws/src/arm_control/scripts"),
              os.path.expanduser("~/robot_vla_ws/robot_code/arm_control/scripts"),
              os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                           "arm_control", "scripts")):
        d = os.path.abspath(d)
        if os.path.isfile(os.path.join(d, "ik_check.py")):
            sys.path.insert(0, d)
            break
    else:
        print("  ⚠️ 找不到 ik_check.py，跳过 IK 自检")
        return True

    from ik_check import UrdfKinematics
    kin = UrdfKinematics(urdf, cfg["robot"]["base_frame"], cfg["robot"]["tip_frame"])
    q0 = np.asarray(cfg["robot"]["home_joints"], float)
    T = kin.fk(q0)
    print("  ✅ URDF 解析成功，home 末端位置 %s" % np.round(T[:3, 3], 4).tolist())

    # 用配置里的桌面作业区中心当"感知出来的顶面中心"（仿真里验证过可达）
    half = float(cfg["perception"]["box_half_size_m"])
    center = np.asarray(cfg["task"]["work_aim_m"], float)
    target = center + np.array([0.0, 0.0, half])
    plan = plan_grasp(kin, q0, target, cfg)
    d = plan["diagnostics"]
    print("  ✅ 规划成功：路径点 %d，下降 %d 步，抬升 %d 步" %
          (len(plan["approach_waypoints"]), d["descend_points"], d["lift_points"]))
    print("     首次姿态切换 %.3f rad（正常），下降段最大跳变 %.3f rad，"
          "最小限位余量 %.3f rad" %
          (d["initial_jump"], d["jump"], d["min_limit_margin"]))
    ok = d["jump"] < 1.0 and d["min_limit_margin"] > 0.0
    print("  %s 路径连续性/限位检查" % ("✅" if ok else "❌"))

    # ---- 放置段自检（抓着方块从抬升终点走到放置区）----
    if not cfg.get("place", {}).get("enabled"):
        print("  ·  放置段未启用（place.enabled=false），跳过")
        return ok
    try:
        pp = plan_place(kin, plan["lift_waypoints"][-1], cfg)
    except Exception as e:                        # noqa: BLE001
        print("  ❌ 放置段规划失败：%s" % e)
        return False
    dp = pp["diagnostics"]
    print("  ✅ 放置段规划成功：转运 %d 点（抬 %d / 移 %d / 降 %d），撤离 %d 点"
          % (len(pp["transfer_waypoints"]), dp["up_points"], dp["across_points"],
             dp["down_points"], dp["retreat_points"]))
    print("     放置点 %s  水平转运距离 %.3fm  转运高度 %.3fm"
          % (np.round(pp["place_center"], 3).tolist(), dp["distance_m"],
             dp["travel_z"]))
    print("     最大跳变 %.3f rad，最小限位余量 %.3f rad"
          % (dp["jump"], dp["min_limit_margin"]))
    for n in dp["notes"]:
        print("     ⚠️  %s" % n)
    ok2 = dp["jump"] < 1.0 and dp["min_limit_margin"] > 0.0
    print("  %s 放置段连续性/限位检查" % ("✅" if ok2 else "❌"))
    return ok and ok2

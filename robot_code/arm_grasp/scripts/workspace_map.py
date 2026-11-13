#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""workspace_map.py —— 量出「真正能抓的作业区」，别再用猜的 1m×1m

为什么要它
----------
用户估的作业区是 1m×1m，但 ECO65 的工作半径只有 0.627m【实测参数】，
以底座为原点的 1m×1m 方块根本不可能全部可达。而后面所有参数——
作业区中心、放置点、标定板摆哪、观察位该看多大范围——都取决于
"真正能到的地方有多大、在哪"。

这个脚本不动机器人，只用 URDF + IK 把可达区算出来，打成一张 ASCII 图，
并给出四个建议值：
  · task.work_aim_m      （作业区中心，演练与兜底用）
  · place.xy 的建议范围   （放哪儿最稳）
  · 标定板该摆的区域      （必须落在可达区内部）
  · 观察位大概要多宽视场

判据用的是【工具 z 轴竖直向下】——和抓取时完全一致的约束，所以算出来的
就是"能抓的地方"，不是"理论可达球"。

用法
----
  python3 robot_code/arm_grasp/scripts/workspace_map.py
  python3 robot_code/arm_grasp/scripts/workspace_map.py --surface-z 0.0 --step 0.03
  python3 robot_code/arm_grasp/scripts/workspace_map.py --surface-z 0.10   # 桌面比底座高
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main(argv=None):
    ap = argparse.ArgumentParser(description="可达作业区测绘（工具竖直向下约束）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--surface-z", type=float, default=None,
                    help="台面高度（米）。默认取 place.surface_z")
    ap.add_argument("--step", type=float, default=0.04, help="网格步长（米，默认 0.04）")
    ap.add_argument("--x-max", type=float, default=0.66, help="x 扫描上限（米）")
    ap.add_argument("--y-half", type=float, default=0.34, help="y 扫描半宽（米）")
    a = ap.parse_args(argv)

    from common import load_config, setup_import_paths, find_urdf
    from planner import DOWN

    cfg = load_config(a.config)
    setup_import_paths(cfg)
    from ik_check import UrdfKinematics

    kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                         cfg["robot"]["tip_frame"])
    offset = np.asarray(cfg["robot"]["tcp_offset"], float)
    lo = np.asarray(cfg["robot"]["joint_limits_lo"], float)
    hi = np.asarray(cfg["robot"]["joint_limits_hi"], float)
    q_home = np.asarray(cfg["robot"]["home_joints"], float)
    half = float(cfg["perception"]["box_half_size_m"])

    surf = a.surface_z
    if surf is None:
        surf = float((cfg.get("place") or {}).get("surface_z", 0.0))

    zs = {"抓取": surf + half, "抬升": surf + half + 0.10, "转运": surf + 0.18}
    print("=" * 70)
    print(" 可达作业区测绘   台面 z=%.3f   约束=工具 z 轴竖直向下" % surf)
    print(" 三个高度：%s" % "  ".join("%s z=%.3f" % (k, v) for k, v in zs.items()))
    print("=" * 70)

    # 线缆包络：约束【法兰】，工具竖直向下时法兰 = 指尖 + (0,0,0.177)
    from common import cable_envelope
    env = cable_envelope(cfg)
    if env:
        elo, ehi = env        # ⚠️ 别用 lo/hi，那两个是关节限位
        xs = [x for x in np.arange(0.12, a.x_max + 1e-9, a.step)
              if elo[0] <= x <= ehi[0]]
        ys = [y for y in np.arange(-a.y_half, a.y_half + 1e-9, a.step)
              if elo[1] <= y <= ehi[1]]
        print("\n 已按线缆包络裁剪：x ≤ %.2f、|y| ≤ %.2f、法兰 z ≤ %.2f（指尖 ≤ %.2f）"
              % (ehi[0], ehi[1], ehi[2], ehi[2] - offset[2]))
        zmax = ehi[2] - offset[2]
        for k in list(zs):
            if zs[k] > zmax + 1e-9:
                print(" ⚠️ %s 高度 %.3f 超包络（指尖最多 %.3f），已按 %.3f 计"
                      % (k, zs[k], zmax, zmax))
                zs[k] = min(zs[k], zmax)
    else:
        xs = list(np.arange(0.12, a.x_max + 1e-9, a.step))
        ys = list(np.arange(-a.y_half, a.y_half + 1e-9, a.step))

    # 这里的 IK 只判"可达/不可达"，不需要 1e-4 精度：
    # 用 120 次迭代 + 2 个朝向（而不是 planner 里的 500 次 + 4 个朝向），
    # 单点从 ~310ms 降到 ~25ms，整张图才能在一分钟内画完。
    def quick_ik(q_init, p_tip):
        from ik_check import dls_ik
        for xg in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
            yg = np.cross(DOWN, xg)
            R = np.column_stack([xg, yg, DOWN])
            q, ok, err = dls_ik(kin, np.asarray(q_init, float),
                                np.asarray(p_tip, float) - R @ offset, R,
                                max_iter=120, tol=2e-3, lam=1e-3, max_step=0.2)
            if not ok or err > 8e-3:
                continue
            if np.any(q < lo) or np.any(q > hi):
                continue
            return q
        return None

    grid_low, grid_all = {}, {}
    n_low = n_all = 0
    last_q = q_home
    print("\n 扫格中（每个点判三次高度，约 %.0f 秒）…" % (len(xs) * len(ys) * 0.03))
    for y in ys:
        for x in xs:
            q1 = quick_ik(last_q, np.array([x, y, zs["抓取"]]))
            grid_low[(x, y)] = q1 is not None
            if q1 is not None:
                n_low += 1
                last_q = q1          # 用上一个解热启动，既快又保持同一分支
                ok_all = all(quick_ik(q1, np.array([x, y, z])) is not None
                             for z in (zs["抬升"], zs["转运"]))
                grid_all[(x, y)] = ok_all
                n_all += 1 if ok_all else 0
            else:
                grid_all[(x, y)] = False
        print(".", end="", flush=True)
    print()

    # ---------------- ASCII 图 ----------------
    print("\n 图例：# 三层都可达（最稳）   + 只能到抓取高度   · 不可达")
    print("      纵轴 y（米），横轴 x（米）；原点 = 机械臂底座")
    ncol = len(xs)
    print("         " + "".join(("%-5.2f" % x) if i % 5 == 0 else ""
                               for i, x in enumerate(xs)))
    print("         " + "".join("|" if i % 5 == 0 else " " for i in range(ncol)))
    for y in reversed(ys):
        row = ""
        for x in xs:
            row += "#" if grid_all[(x, y)] else ("+" if grid_low[(x, y)] else "·")
        print("  y=%+.2f %s" % (y, row))
    print("         " + "".join("|" if i % 5 == 0 else " " for i in range(ncol)))
    print("         x 从 %.2f 到 %.2f，每格 %.0fmm"
          % (xs[0], xs[-1], a.step * 1000))

    # ---------------- 结论 ----------------
    pts = [k for k, v in grid_all.items() if v]
    pts_low = [k for k, v in grid_low.items() if v]
    if not pts:
        print("\n ❌ 三层都可达的点一个都没有：检查 --surface-z 是否填错（台面高度）")
        return 1
    ax = np.array([p[0] for p in pts])
    ay = np.array([p[1] for p in pts])
    print("\n" + "-" * 70)
    print(" 三层都可达的区域（这才是「稳定作业区」）")
    print("   x: %.3f ~ %.3f   （宽 %.0fmm）" % (ax.min(), ax.max(),
                                              (ax.max() - ax.min()) * 1000))
    print("   y: %.3f ~ %.3f   （宽 %.0fmm）" % (ay.min(), ay.max(),
                                              (ay.max() - ay.min()) * 1000))
    print("   面积 ≈ %.3f m²   （网格点 %d 个）" % (
        (ax.max() - ax.min()) * (ay.max() - ay.min()), len(pts)))
    if pts_low:
        lx = np.array([p[0] for p in pts_low])
        ly = np.array([p[1] for p in pts_low])
        print(" 仅抓取高度可达（能抓但抬不高等高转运）：x %.3f~%.3f, y %.3f~%.3f"
              % (lx.min(), lx.max(), ly.min(), ly.max()))

    cx, cy = float(ax.mean()), float(ay.mean())
    print("\n 建议值（直接抄进 real_params.yaml）")
    print("   task.work_aim_m: [%.3f, %.3f, %.3f]" % (cx, cy, surf + half))
    print("   # 上面这个只是【几何中心】。真机第一次跑之前，"
          "用 jog.py 把指尖挪到那儿确认没有障碍")
    print("\n   标定板摆放区（必须在三层可达区内部，且相机看得见）：")
    print("     x %.3f ~ %.3f ,  y %.3f ~ %.3f"
          % (ax.min() + 0.03, ax.max() - 0.03,
             ay.min() + 0.03, ay.max() - 0.03))
    print("\n   放置点建议（挑一个离抓取区有距离、又不贴边的位置）：")
    for name, (px, py) in (("左侧", (cx, ay.max() - 0.06)),
                           ("右侧", (cx, ay.min() + 0.06)),
                           ("远端", (ax.max() - 0.06, cy))):
        okp = quick_ik(q_home, np.array([px, py, zs["转运"]])) is not None
        print("     %s  place.xy = [%.3f, %.3f]  %s"
              % (name, px, py, "✅ 转运高度可达" if okp else "❌ 太高，别选"))

    print("\n 和你的预估对一下：")
    print("   你估的是 1m×1m = 1.0 m²；实测稳定区 %.3f m²。"
          % ((ax.max() - ax.min()) * (ay.max() - ay.min())))
    print("   差得多是正常的：ECO65 半径 0.627m，正方形四角到不了。")
    print("-" * 70)
    print("\n 下一步：")
    print("   1) 用这个区域决定标定板摆哪、方块摆哪、放置区在哪")
    print("   2) jog.py 把臂摆到能看见整个区域的位姿 → 按 c 打印 observe_joints")
    print("   3) 该区域就是 34 号文档里说的「作业区」，别再按 1m×1m 规划")
    return 0


if __name__ == "__main__":
    sys.exit(main())

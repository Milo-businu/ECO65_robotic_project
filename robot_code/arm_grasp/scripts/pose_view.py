#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pose_view.py —— 不动机器人，先看一个姿态长什么样（终端里画侧视图）

为什么需要它
------------
2026-11-09 我给了一个「法兰 0.45m、工具朝下」的 home，算出来很漂亮，
但**手腕折了 90°、工具又自转 73°** —— 现场一看线要被拧断。

教训：**姿态不能只看末端位置**。肘、腕的折叠角度决定线缆怎么走，
必须"先看形状，再决定要不要动"。

这个脚本只做正运动学（不碰硬件），把 6 个连杆画成终端里的侧视图（x-z 平面），
同时给出：法兰/TCP 坐标、工具轴方向、每个关节角、离线缆包络的余量。

用法
----
  python3 robot_code/arm_grasp/scripts/pose_view.py                 # 看配置里的 home
  python3 robot_code/arm_grasp/scripts/pose_view.py --name observe
  python3 robot_code/arm_grasp/scripts/pose_view.py --joints "0,-0.25,1.8,0,0,0"
  python3 robot_code/arm_grasp/scripts/pose_view.py --compare A B   # 并排对比（见 --help）

判读要点
--------
  · 关节 4/5/6 接近 0 = 手腕是直的（线最松）；|J5| 接近 1.57 = 手腕折 90°（危险）
  · 肘（J3）越大，前臂越贴着上臂折叠，线也越容易被绷紧
  · 「工具轴」写着 (0,0,-1) 表示工具朝下；(0,1,0) 表示水平朝侧面
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------- URDF
def load_chain(urdf_path):
    """从 URDF 读 joint1..joint6 的 (xyz, rpy, axis)。"""
    root = ET.parse(urdf_path).getroot()
    by_name = {}
    for j in root.findall("joint"):
        name = j.get("name")
        if not name or not name.startswith("joint"):
            continue
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy") if o is not None else "0 0 0").split()]
        a = j.find("axis")
        axis = [float(v) for v in (a.get("xyz") if a is not None else "0 0 1").split()]
        by_name[name] = (xyz, rpy, axis)
    chain = [by_name["joint%d" % i] for i in range(1, 7)]
    return chain


def rpy_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def axang(axis, q):
    a = np.array(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def fk_chain(chain, q):
    T = np.eye(4)
    pts = [T[:3, 3].copy()]
    for i, (xyz, rpyv, axis) in enumerate(chain):
        M = np.eye(4)
        M[:3, :3] = rpy_mat(*rpyv)
        M[:3, 3] = xyz
        T = T @ M
        R = np.eye(4)
        R[:3, :3] = axang(axis, q[i])
        T = T @ R
        pts.append(T[:3, 3].copy())
    return pts, T


# ---------------------------------------------------------------- 画图
def side_view(pts, tip, env, w=52, h=19,
              xlim=(-0.28, 0.55), zlim=(-0.06, 0.86)):
    """终端里的 x-z 侧视图。#=包络边界, 1..6=关节, F=法兰, T=指尖, B=基座。"""
    grid = [[" "] * w for _ in range(h)]

    def to_cell(x, z):
        c = int(round((x - xlim[0]) / (xlim[1] - xlim[0]) * (w - 1)))
        r = int(round((zlim[1] - z) / (zlim[1] - zlim[0]) * (h - 1)))
        return r, c

    # 台面 z=0
    r0, _ = to_cell(0, 0)
    for c in range(w):
        grid[r0][c] = "-"
    # 包络（只画 z 上限与 y/x 范围不在此视图）
    if env:
        lo, hi = env
        rz, _ = to_cell(0, hi[2])
        for c in range(w):
            if grid[rz][c] == " ":
                grid[rz][c] = "."
        for x in (lo[0], hi[0]):
            c = to_cell(x, 0)[1]
            for r in range(rz, r0 + 1):
                if grid[r][c] == " ":
                    grid[r][c] = "."

    # 连杆
    seq = [("B", pts[0], None)] + [("%d" % (i + 1), pts[i + 1], None) for i in range(6)]
    for label, p, _ in seq:
        r, c = to_cell(p[0], p[2])
        if 0 <= r < h and 0 <= c < w:
            grid[r][c] = label
    # 法兰 F 与指尖 T
    r, c = to_cell(pts[6][0], pts[6][2])
    if 0 <= r < h and 0 <= c < w:
        grid[r][c] = "F"
    r, c = to_cell(tip[0], tip[2])
    if 0 <= r < h and 0 <= c < w:
        grid[r][c] = "T"

    out = []
    for i, row in enumerate(grid):
        z = zlim[1] - (zlim[1] - zlim[0]) * i / (h - 1)
        out.append("  %+.2f |%s" % (z, "".join(row)))
    out.append("       +" + "-" * w)
    out.append("        x %+.2f (前) … %+.2f" % (xlim[0], xlim[1]))
    return "\n".join(out)


def describe(name, q, chain, offset, env, cfg_safety=None):
    pts, T = fk_chain(chain, np.asarray(q, float))
    f = T[:3, 3]
    tip = T[:3, :3] @ np.asarray(offset, float) + f
    axis = T[:3, 2]
    lines = []
    lines.append("=" * 66)
    lines.append(" %s" % name)
    lines.append("=" * 66)
    lines.append(" 关节(rad) " + "  ".join("J%d=%+.3f" % (i + 1, v) for i, v in enumerate(q)))
    lines.append(" 关节(deg) " + "  ".join("J%d=%+6.1f" % (i + 1, math.degrees(v))
                                            for i, v in enumerate(q)))
    lines.append(" 法兰 [%.3f %.3f %.3f]   指尖 [%.3f %.3f %.3f]"
                 % (f[0], f[1], f[2], tip[0], tip[1], tip[2]))
    # 工具轴用大白话说
    ax = axis
    if ax[2] < -0.8:
        word = "朝下（垂直向下）"
    elif ax[2] > 0.8:
        word = "朝上"
    elif abs(ax[1]) > 0.8:
        word = "水平朝侧向（+y）"
    elif abs(ax[0]) > 0.8:
        word = "水平朝前/后（x）"
    else:
        word = "斜的"
    lines.append(" 工具轴 [%.2f %.2f %.2f] → %s" % (ax[0], ax[1], ax[2], word))
    # 线缆风险提示（只按几何说话，不知道你线怎么走）
    warn = []
    if abs(q[5]) > 0.6:
        warn.append("★ J6=%+.0f° 工具自转了很多 → 夹爪那边线会绕（同一个末端位置，"
                    "选别的分支 J6 可能只有 10~20°）" % math.degrees(q[5]))
    if abs(q[2]) > 1.5:
        warn.append("J3=%+.0f° 肘折得很紧 → 前臂贴着上臂，线可能被夹" %
                    math.degrees(q[2]))
    if abs(q[4]) > 1.2:
        warn.append("J5=%+.0f°：工具朝下时这是【必然】的（不是问题），"
                    "别被这个数吓到" % math.degrees(q[4]))
    if not warn:
        warn.append("J3/J6 都不大 → 这个形状对线最友好")
    for w in warn:
        lines.append(" · " + w)
    if env:
        lo, hi = env
        over = np.maximum(lo - f, f - hi)
        m = float(np.max(over))
        if m > 1e-6:
            k = int(np.argmax(over))
            lines.append(" 包络：法兰超出 %s 方向 %.0fmm（允许 x %+.2f~%+.2f y %+.2f~%+.2f z %+.2f~%+.2f）"
                         % ("xyz"[k], over[k] * 1000, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
        else:
            lines.append(" 包络：法兰在盒内，最近余量 %.0fmm"
                         % (float(np.min(np.minimum(f - lo, hi - f))) * 1000))
    lines.append("")
    lines.append(side_view(pts, tip, env))
    return "\n".join(lines), pts, T, tip


def main(argv=None):
    ap = argparse.ArgumentParser(description="预览一个关节姿态（不动机器人）")
    ap.add_argument("--joints", default=None, help='6 个关节角，逗号分隔，如 "0,0.2,0.3,0,0,0"')
    ap.add_argument("--name", default="home", choices=["home", "observe"],
                    help="不给 --joints 时看配置里的哪个")
    ap.add_argument("--config", default=None)
    a = ap.parse_args(argv)

    from common import load_config, find_urdf, cable_envelope

    cfg = load_config(a.config)
    urdf = find_urdf(cfg)
    chain = load_chain(urdf)
    offset = np.asarray(cfg["robot"]["tcp_offset"], float)
    env = cable_envelope(cfg)
    mode = str(((cfg.get("safety") or {}).get("envelope_mode", "warn"))).lower()

    if a.joints:
        q = [float(v) for v in a.joints.replace("，", ",").split(",")]
        if len(q) != 6:
            print("❌ --joints 需要 6 个数")
            return 2
        name = "自定义姿态"
    else:
        q = cfg["task"]["observe_joints"] if a.name == "observe" else cfg["robot"]["home_joints"]
        name = "配置里的 %s" % a.name

    text, *_ = describe(name, q, chain, offset, env)
    print(text)
    print("\n  包络模式 = %s（%s）" % (mode, "超了只告警" if mode != "block" else "超了拒绝执行"))
    print("  确认形状 OK 再动臂：把上面那行关节角填进 robot.home_joints / task.observe_joints")
    return 0


if __name__ == "__main__":
    sys.exit(main())

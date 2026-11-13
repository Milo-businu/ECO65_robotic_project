#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""handeye_quality.py —— 手眼标定数据的「GO / NO-GO」判据

为什么要有它：AX=XB 最容易废在【姿态激励不足】，而不是算法。
常见翻车是采了 20 组，但机械臂只在一个平面里平移 / 只绕一个轴转，
解出来的旋转分量在某个方向完全不可信 —— 残差看起来还挺小，
一上真机抓取点就飘。采完立刻跑这个脚本，别等求解完再后悔。

用法：
  python3 handeye_quality.py --input ~/handeye_samples.json
  python3 handeye_quality.py --input ~/handeye_samples.json --probe 1.8 2.4 2.1 3.0

判据（来源见 33 号预案「外部经验」一节）：
  样本数        >= 12（推荐 15~20）
  三轴旋转覆盖  至少两轴相对首帧变化 >= 15°，最好三轴
  相对旋转角    最大 >= 30°，且至少 8 组 >= 15°
  相邻姿态      相对旋转 >= 5°，平移 >= 20mm（否则这一组白采）
  打点验证      max 误差 < 3mm（--probe 传入实测偏差 mm）

⚠️ 残差小 ≠ 标定对。必须再做物理打点验证（多个位置，不能只测图像中心一点）。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import quat_wxyz_to_R   # noqa: E402

_items = []


def rec(level, msg):
    _items.append(level)
    print("  %s %s" % (level, msg))


def rotvec_deg(R):
    """旋转矩阵 → 旋转向量（轴 × 角，单位度）"""
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    th = float(np.arccos(c))
    if th < 1e-6:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]]) / (2.0 * np.sin(th))
    return np.degrees(th) * v


def analyse(samples):
    """返回 (Rs, xyzs, 相邻相对旋转角列表, 相邻平移 mm 列表)"""
    Rs, xyzs = [], []
    for s in samples:
        fb = s["flange_in_base"]
        q = fb.get("quat_wxyz") or [1, 0, 0, 0]
        Rs.append(quat_wxyz_to_R(np.asarray(q, float)))
        xyzs.append(np.asarray(fb.get("xyz", [0, 0, 0]), float))
    step_rot, step_xyz = [], []
    for i in range(1, len(Rs)):
        rv = rotvec_deg(Rs[i - 1].T @ Rs[i])
        step_rot.append(float(np.linalg.norm(rv)))
        step_xyz.append(float(np.linalg.norm(xyzs[i] - xyzs[i - 1])) * 1000.0)
    return Rs, xyzs, step_rot, step_xyz


def main():
    ap = argparse.ArgumentParser(description="手眼标定数据质量判据")
    ap.add_argument("--input", default="~/handeye_samples.json")
    ap.add_argument("--probe", nargs="*", type=float, default=[],
                    help="打点验证的实测偏差（mm），可传多个")
    ap.add_argument("--min-samples", type=int, default=12)
    ap.add_argument("--focused", action="store_true",
                    help="作业区聚焦模式：现场作业区约 1m×1m、方块放机械臂周围近处，"
                         "不追求大范围/最高处姿态。放宽样本数与旋转幅度要求，"
                         "但【强制】要求打点验证覆盖作业区四角")
    a = ap.parse_args()
    min_n = 8 if a.focused else a.min_samples
    axis_th = 12.0 if a.focused else 15.0
    max_ang = 20.0 if a.focused else 30.0
    big_n = 5 if a.focused else 8

    path = os.path.expanduser(a.input)
    print("=" * 66)
    print(" 手眼标定数据质量检查  %s" % path)
    if a.focused:
        print(" 模式: 作业区聚焦（1m×1m 近处作业区，不要求最高处姿态）")
    print("=" * 66)
    if not os.path.isfile(path):
        print("  ❌ 找不到文件")
        return 1
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    samples = data.get("samples", [])
    print("\n  数据来源: %s" % data.get("_readme", "（无说明）"))
    print("  flange_source: %s / frames: %s"
          % (data.get("flange_source", "?"), data.get("frames", {})))

    print("\n--- 样本量 ---")
    n = len(samples)
    rec("✅" if n >= min_n else "❌",
        "样本 %d 组（本模式要求 >= %d%s）"
        % (n, min_n, "，聚焦作业区 8~12 组即可" if a.focused else "，推荐 15~20"))
    if n < 3:
        print("\n  ❌ 样本太少，无法分析")
        return 1

    Rs, xyzs, step_rot, step_xyz = analyse(samples)

    print("\n--- 姿态旋转激励（相对首帧，度）---")
    rel = [rotvec_deg(Rs[0].T @ R) for R in Rs]
    arr = np.asarray(rel)
    axes = ["X", "Y", "Z"]
    spans = []
    for k, ax in enumerate(axes):
        span = float(arr[:, k].max() - arr[:, k].min())
        spans.append(span)
        rec("✅" if span >= axis_th else "⚠️ ",
            "绕 %s 轴分量范围 %5.1f° %s"
            % (ax, span, "" if span >= axis_th else "← 激励不足，补采绕该轴旋转的位姿"))
    good_axes = sum(1 for s in spans if s >= axis_th)
    rec("✅" if good_axes >= 2 else "❌",
        "有 %d 个轴达到 >=%.0f°（至少 2 个，最好 3 个）" % (good_axes, axis_th))

    angles = [float(np.linalg.norm(r)) for r in rel]
    big = sum(1 for x in angles if x >= axis_th)
    rec("✅" if max(angles) >= max_ang and big >= big_n else "⚠️ ",
        "相对首帧旋转角 min %.1f° / max %.1f° / 平均 %.1f°；>=%.0f° 的有 %d 组"
        % (min(angles), max(angles), float(np.mean(angles)), axis_th, big))

    print("\n--- 相邻姿态（有没有白采 / 重复）---")
    if step_rot:
        rec("✅" if min(step_rot) >= 5.0 else "⚠️ ",
            "相邻相对旋转 min %.1f° / max %.1f°"
            % (min(step_rot), max(step_rot)))
        bad_rot = [i + 1 for i, v in enumerate(step_rot) if v < 5.0]
        if bad_rot:
            rec("⚠️ ", "第 %s 组几乎没转（<5°）。若这些是【平移组】则是正常的"
                "（扫位设计：平移组不改姿态，专门激励平移）；人工采集则属于重复采样"
                % ", ".join(str(i + 1) for i in bad_rot))
        rec("✅" if min(step_xyz) >= 20.0 else "⚠️ ",
            "相邻平移 min %.0fmm / max %.0fmm（建议 >=20mm，且标定板覆盖图像中心与四角）"
            % (min(step_xyz), max(step_xyz)))

    print("\n--- 物理打点验证（残差小 ≠ 标定对）---")
    need_pts = 5 if a.focused else 3
    if a.probe:
        mx = max(a.probe)
        rec("✅" if mx < 3.0 else "❌",
            "打点误差 %s mm，最大 %.1f mm（要求 <3mm）"
            % ("/".join("%.1f" % v for v in a.probe), mx))
        if len(a.probe) < need_pts:
            rec("❌" if a.focused else "⚠️ ",
                "只测了 %d 个点（本模式要求 >=%d）—— 必须覆盖作业区中心 + 四角，"
                "不能只测图像中心一点%s"
                % (len(a.probe), need_pts,
                   "；聚焦模式姿态范围小，打点是唯一可信验证" if a.focused else ""))
    else:
        # ★ 注意：打点验证要【先有 X】才能做（要用它把标记位置算出来再去碰），
        #   所以在"求解前"跑本脚本时一定还没有 --probe。
        #   这里只提醒，不打 ❌ —— 否则第一次检查永远 NO-GO，属于逻辑死循环。
        rec("⚠️ ", "还没有打点数据（正常：打点必须在求解之后做）。"
                   "流程是：质量检查 → 求解 → verify_handeye 5 点 → "
                   "再回来 `--probe <5 个数值>` 复检，那时才是真正的 GO/NO-GO")

    print("\n--- 现场条件（脚本查不了，人工确认）---")
    rec("⚠️ ", "A4 标定纸已贴到硬板并用胶带/多孔台面夹具固定，全程未动")
    rec("⚠️ ", "ArUco 外框边长用尺子实量后填 --marker-size（A4 打印有 0.5~2% 缩放误差）"
        if data.get("board") == "aruco" else
        "标定板哑光不反光、方格边长用尺子/卡尺实量")
    rec("⚠️ ", "每组【停稳后】才按回车（图像与关节角时间同步 <10ms）")
    rec("⚠️ ", "相机支架/线缆无松动（微位移会破坏刚性假设）")
    rec("⚠️ ", "标定姿态覆盖的区域 = 明天抓取的作业区（DH 误差会让边缘区域偏）")
    if a.focused:
        rec("⚠️ ", "聚焦模式：不必采最高处/极限姿态，但作业区四角都要有姿态覆盖")

    n_fail = _items.count("❌")
    n_warn = _items.count("⚠️ ")
    print("\n" + "=" * 66)
    print(" 汇总：❌ %d / ⚠️  %d / ✅ %d" % (n_fail, n_warn, _items.count("✅")))
    if n_fail:
        print(" 结论：NO-GO —— 补采后再求解，别拿这组数据去抓真机")
    elif n_warn:
        print(" 结论：条件性 GO —— 可以求解，但必须完成 ⚠️ 项（尤其打点验证）")
    else:
        print(" 结论：GO —— 可以求解并把 4x4 矩阵填进 real_params.yaml")
    print("=" * 66)
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())

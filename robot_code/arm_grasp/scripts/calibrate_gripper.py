#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS42DC 开口标定 —— 建立「电机角度° ↔ 夹爪开口 mm」的映射

为什么必须做
------------
夹爪只有【位置模式】，没有力控。夹持靠「让目标开口略小于物体宽度，
手指顶到物体后堵转自动停」。如果角度→开口的换算不准，
要么夹空（目标开口算大了），要么一路顶死（算小了）。

★「每次 300° 到底怎么精准控制？」—— 答案在这里
------------------------------------------------
1) **不用你手动控制**。按回车，脚本就发一条精确的数字指令：
     角度字段单位是 0.1°，300° → 3000 → 两字节 0x0B 0xB8
   步进电机是「数脉冲走路」，驱动器按脉冲数走，指令本身非常准。
   32 细分下分辨率约 0.056°/微步，比指令分辨率还细 —— 精度不是瓶颈。

2) **但别信指令值，要读回实际值**。真正会出错的是【丢步】或【堵转提前停】
   （顶到东西、速度太快、供电不足）。一旦丢步，"我以为走了 300°"和
   "实际走了 300°"就不一样，拟合出来的 deg_per_mm 会系统性偏掉。

3) 所以本脚本现在每次闭合后都发 CMD_QUERY **回读累计角度**，
   以 home 后的读数为基准做差分，用【实际角度】去拟合；
   同时把「指令累计 vs 实际累计」的差打印出来给你看，偏差大就直接判不可信。

★ 拟合模型：带【机械空程】的分段线性（2026-11-03 真机实测修正）
--------------------------------------------------------------
   旧模型 angle = (w_max - w)·k 强制过原点，把空程塞进斜率里，
   实测 8 组数据残差 10mm 以上。真机丝杠/齿轮有间隙：电机先空转
   约 105° 手指才动，之后才严格线性，末端还会撞机械限位。

       w(angle) = w_max - max(0, angle - deadzone_deg)/k

   拟合用【中位数】求 k（每段 Δangle/Δ开口）与 deadzone（各点偏移），
   天然剔除起始过渡段和末端限位段；残差判定也只看稳定段。
   结果输出四个值：deg_per_mm / deadzone_deg / max_width_mm / calibration_done。
   ⚠️ 从 home 出发的第一次闭合要走 deadzone + Δ·k 度（见空夹验证）；
      之后连续闭合的增量里 deadzone 会抵消。

⚠️ 协议发的是【相对角度增量】；反馈回的是【累计角度】，不是绝对开口。
⚠️ 拨码开关 2 号必须 ON、供电 10~25V、不支持热插拔。

用法
----
  # 第一步：先验证「发 300° 是否真的走 300°」+ 反馈倍率对不对（不用尺子）
  python3 calibrate_gripper.py --verify --port /dev/motor_serial

  # 第二步：正式标定（需要尺子）
  python3 calibrate_gripper.py --port /dev/motor_serial

  # 不接硬件，演示拟合算法
  python3 calibrate_gripper.py --demo

★ 关于「grip:40」（2026-11-03 用户踩坑）
------------------------------------------
  `grip:40` **不是终端命令**，直接敲会 command not found。
  它是发给夹爪 ROS 节点的【话题消息内容】，含义是
  「按物体宽度 40mm 夹持 → 目标开口 = 40 − interference_mm」。
  三种等价做法：
    ① 本脚本标定成功后会直接问「现在做空夹验证吗？」，回车即可，
       用刚拟合出的参数走串口，不用起 ROS（推荐）
    ② ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'grip:40'"
       （需先起 gripper_node.py；注意 pub 支持 --once，echo 不支持）
    ③ 起 gripper_web.py，在 http://127.0.0.1:8777 页面上输入 40 点 Grip

⚠️ 拟合最大残差 > 3mm 时脚本会判定数据不可信并拒绝输出配置值，
   请按提示重标，别把不准的 deg_per_mm 填进 real_params.yaml。
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ms42dc_driver import Ms42dc, OpeningMap, build_cmd   # noqa: E402


def fit(w_max: float, samples, min_width: float = 10.0):
    """稳健拟合【带空程的分段线性】模型。

        w(angle) = w_max - max(0, angle - deadzone)/k

    为什么不用"强制过原点最小二乘"：真机丝杠有机械空程（电机先空转约
    105° 才带动手指），末端还有机械限位。旧模型把空程塞进斜率，
    实测 8 组数据残差 10mm 以上且 deg_per_mm 被系统性拉偏。

    这里改用【中位数】，天然剔除过渡段和限位段：
      k         = median(每段 Δangle / Δ开口)
      deadzone  = median(angle_i - (w_max - w_i)·k)

    返回 (k, deadzone, w_max, 稳定段最大残差, model, rows)
    rows = [(角度, 实测, 预测, 残差, 是否稳定段), ...]
    """
    seg_k = []
    for i in range(1, len(samples)):
        da = samples[i][0] - samples[i - 1][0]
        dw = samples[i - 1][1] - samples[i][1]   # 开口减少量（应为正）
        if da > 1e-6 and dw > 1e-6:
            seg_k.append(da / dw)
    if len(seg_k) < 2:
        raise ValueError("有效样本不足 3 组（无法求段斜率）")
    k = float(np.median(seg_k))

    offs = [a - (w_max - w) * k for a, w in samples[1:]]
    deadzone = max(0.0, float(np.median(offs)))

    model = OpeningMap(max_width_mm=w_max, deg_per_mm=k,
                       deadzone_deg=deadzone, min_width_mm=min_width,
                       calibrated=True)
    rows, errs = [], []
    for i, (a, w) in enumerate(samples):
        pred = model.angle_to_width(a)
        d = w - pred
        # 首组是空程渐变过渡、末端组是机械限位，都不参与"数据是否可信"判定
        stable = (i >= 2) and (w > min_width + 2.0) and (a > 0)
        rows.append((a, w, pred, d, stable))
        if stable:
            errs.append(abs(d))
    err = max(errs) if errs else 0.0
    return k, deadzone, w_max, err, model, rows


def read_angle(dev, scale):
    """发 CMD_QUERY 读回【累计角度】；读不到返回 None。"""
    fb = dev.query()
    if fb is None:
        return None
    return float(fb["angle_deg"]) * float(scale)


# ---------------------------------------------------------------- 验证模式
def verify(dev, args):
    print("\n[验证模式] 不量开口，只反复发 %.0f° 并回读，看电机有没有真的走到位"
          % args.step_deg)
    print("  判读：每次增量应 ≈ %.0f°；若稳定是 %.0f° 或 %.0f°，说明反馈倍率解析错了\n"
          % (args.step_deg, args.step_deg / 10.0, args.step_deg * 10.0))
    prev = None
    for i in range(1, args.points + 1):
        input("  按回车发第 %d/%d 次 %.0f°…" % (i, args.points, args.step_deg))
        frame = dev.move_angle(args.step_deg, 1)     # 1 = 闭合
        time.sleep(args.wait)
        if i == 1:
            print("    下发帧: %s" % frame.hex(" "))
            print("             └ 角度字段 %d = %.1f° × 10（0x%02X%02X）"
                  % (int(args.step_deg * 10), args.step_deg,
                     frame[5], frame[6]))
        a = read_angle(dev, args.feedback_scale)
        if a is None:
            print("    ❌ 没收到反馈帧。检查：拨码 2 号 ON / 供电 10~25V / 串口名 / "
                  "是否支持请求式反馈（须先发 CMD_QUERY）")
            continue
        if prev is None:
            print("    第 %d 次 回读累计 %9.1f°（作为基准）" % (i, a))
        else:
            delta = a - prev
            ratio = delta / args.step_deg if args.step_deg else 0.0
            flag = "✅" if abs(delta - args.step_deg) <= args.max_angle_error else "⚠️"
            print("    第 %d 次 回读累计 %9.1f°  本次增量 %8.1f°  "
                  "(指令 %.0f°)  比值 %.3f  %s"
                  % (i, a, delta, args.step_deg, ratio, flag))
            if flag == "⚠️":
                print("       偏差 %.1f° > 阈值 %.1f°" %
                      (delta - args.step_deg, args.max_angle_error))
                if 0.05 < ratio < 0.5:
                    print("       → 像是反馈角度少读了一位，试 --feedback-scale 10")
                elif 2 < ratio < 20:
                    print("       → 像是反馈角度多读了一位，试 --feedback-scale 0.1")
                else:
                    print("       → 多半是丢步：降低 --speed（如 10）、确认供电 10~25V、"
                          "手指别卡住")
        prev = a
    print("\n  验证完。若增量稳定 ≈ %.0f°，就可以正式标定了：" % args.step_deg)
    print("    python3 calibrate_gripper.py --port %s" % args.port)
    dev.close()
    return 0


# ---------------------------------------------------------------- 标定模式
def _interference_mm(args):
    """从 real_params.yaml 读过盈量；读不到就用默认 3mm。"""
    try:
        from common import load_config
        return float(load_config(getattr(args, "config", None))["gripper"]["interference_mm"])
    except Exception:                              # noqa: BLE001
        return 3.0


def do_empty_grip(args, k, deadzone, w_max, model):
    """空夹验证：用【刚拟合出的】参数走一次「grip:<物体宽度>」，量开口对不对。

    这一步不需要起 ROS、不需要 gripper_node、不需要网页 —— 直接独占串口。
    注意从 home 出发要包含【空程角】，所以必须用 model.width_to_angle（绝对累计角度），
    不能用简单的 (w_max - target) * k。
    """
    obj_mm = float(args.grip_width)
    inter = _interference_mm(args)
    target = obj_mm - inter
    delta_deg = model.width_to_angle(target)
    print("\n" + "-" * 66)
    print(" 空夹验证（k=%.4f °/mm，空程角=%.1f°，不重开 ROS）" % (k, deadzone))
    print("   grip:%.0f 的含义：目标开口 = 物体宽 %.0f − 过盈 %.1f = %.1f mm"
          % (obj_mm, obj_mm, inter, target))
    print("   从最大开口 %.1fmm 闭合到 %.1fmm，要走 %.1f°"
          "（含空程角 %.1f°）"
          % (w_max, target, delta_deg, deadzone))
    print("-" * 66)
    dev = Ms42dc(port=args.port, baud=args.baud, addr=args.addr,
                 speed_rads=args.speed, dry_run=args.dry_run)
    try:
        input("  ① 按回车让夹爪先【张开到底】…")
        dev.move_angle(args.home_angle, 0)
        time.sleep(args.wait)
        input("  ② 按回车闭合到 %.1fmm…" % target)
        dev.move_angle(delta_deg, 1)      # 1 = 闭合
        time.sleep(args.wait)
        measured = float(input("  ③ 用尺子量实际开口(mm)，输入：").strip())
        diff = measured - target
        print("\n   目标 %.1f mm / 实测 %.1f mm / 误差 %+.1f mm  %s"
              % (target, measured, diff,
                 "✅ 通过（<2mm）" if abs(diff) <= 2.0 else "⚠️ 偏差过大"))
        if abs(diff) > 2.0:
            print("   → 别急着填配置。多半是 deg_per_mm 还需微调，或标定时读数有问题，"
                  "建议重标一轮。")
        return abs(diff) <= 2.0
    finally:
        dev.close()


def run_pairs(args):
    """用已经测好的 (开口, 角) 数据直接拟合 —— 不用再接串口重测一遍。"""
    print("=" * 66)
    print(" 由已测数据直接拟合（--pairs 模式，不接硬件）")
    print("=" * 66)
    pts = []
    for item in args.pairs.replace("，", ",").split(","):
        item = item.strip()
        if not item:
            continue
        try:
            w_s, a_s = item.split("@")
            pts.append((float(a_s), float(w_s)))
        except Exception:                          # noqa: BLE001
            print(" ❌ 格式错误：%r（应为 开口mm@角度）" % item)
            return 2
    if len(pts) < 3:
        print(" ❌ 至少 3 组")
        return 2
    # 第一组当作 home（w_max）
    a0, w_max = pts[0]
    samples = [(max(0.0, a - a0), w) for a, w in pts]
    print(" w_max（home 开口） = %.1f mm" % w_max)
    print(" 采样（Δ角, 开口）:")
    for a, w in samples:
        print("   %8.1f° → %6.1f mm" % (a, w))
    try:
        k, dz, mx, err, model, rows = fit(w_max, samples,
                                          min_width=args.min_width)
    except ValueError as e:
        print(" ❌ 拟合失败：%s" % e)
        return 1
    print("\n" + "=" * 66)
    print(" 拟合结果（带空程的分段线性模型）")
    print("   deg_per_mm       = %.4f  °/mm" % k)
    print("   deadzone_deg     = %.1f  °" % dz)
    print("   max_width_mm     = %.1f  mm" % mx)
    print("   min_width_mm     = %.1f  mm" % args.min_width)
    print("   拟合残差         = %.2f mm" % err)
    if rows:
        print("\n 逐点残差：")
        for r in rows:
            print("   " + str(r))
    print("\n 填进 real_params.yaml 的 gripper 段，并把 calibration_done 改 true")
    print("=" * 66)
    return 0


def run(args):
    if args.demo:
        print("\n[DEMO] 不接硬件，用一组模拟实测数据演示拟合")
        w_max = 118.0
        samples = [(0.0, 118.0), (300.0, 99.5), (600.0, 81.0),
                   (900.0, 62.5), (1200.0, 44.0), (1500.0, 25.5)]
        for a, w in samples:
            print("   累计 %7.1f°  →  实测开口 %6.1f mm" % (a, w))
        k, dz, mx, err, model, rows = fit(w_max, samples)
        print("\n   拟合 deg_per_mm = %.4f °/mm，空程角 = %.1f°，"
              "最大开口 = %.1f mm，稳定段残差 = %.2f mm" % (k, dz, mx, err))
        print("   交叉校验 40mm 开口 → %.1f°" % model.width_to_angle(40.0))
        print("\n   把下面四行填进 config/real_params.yaml 的 gripper 段：")
        print("     calibration_done: true")
        print("     max_width_mm: %.1f" % mx)
        print("     deg_per_mm: %.4f" % k)
        print("     deadzone_deg: %.1f" % dz)
        return 0

    dev = Ms42dc(port=args.port, baud=args.baud, addr=args.addr,
                 speed_rads=args.speed, dry_run=args.dry_run)
    print("\n" + "=" * 66)
    print(" MS42DC 开口标定")
    print("=" * 66)
    print(" 串口: %s   模式: %s   每次: %.0f°   反馈倍率: %g"
          % (args.port, "DRY-RUN" if args.dry_run else "REAL",
             args.step_deg, args.feedback_scale))
    print(" ⚠️ 确认：供电 10~25V、拨码 2 号 ON、1 号 OFF、USB-C 已接")
    print(" ⚠️ 标定过程中手指会动，别把手放在指间！")
    print(" ℹ️  %.0f° 指令对应字节 %s"
          % (args.step_deg,
             build_cmd(args.addr, 1, args.step_deg, args.speed).hex(" ")))
    print()

    if args.verify:
        return verify(dev, args)

    # ---- 1) home：张开到底 ----
    input("① 确认夹爪前方无遮挡，按回车让夹爪【张开到底】…")
    dev.move_angle(args.home_angle, 0)   # 0 = 张开
    time.sleep(args.wait)
    w_max = float(input("② 用尺子量两指尖【内侧】距离(mm)，输入：").strip())
    print("   最大开口 w_max = %.1f mm" % w_max)
    if w_max > 122.0 or w_max < 8.0:
        print("   ⚠️  %.1fmm 超出夹爪标称 10~120mm 行程 —— 确认量的是两指尖【内侧】，"
              "尺子贴平、读数别量到外侧" % w_max)

    # home 后读一次反馈，作为「累计角度」的零基准
    a0 = read_angle(dev, args.feedback_scale)
    if a0 is None:
        print("   ⚠️ 读不到反馈 → 本脚本只能退回【指令值累加】，结果不可信！")
        print("      强烈建议先跑：python3 calibrate_gripper.py --verify --port %s"
              % args.port)
    else:
        print("   反馈基准角 a0 = %.1f°（后面每次都用它做差分）" % a0)

    # ---- 2) 逐次闭合并实量 ----
    samples = [(0.0, w_max)]
    cmd_total = 0.0
    prev_a = 0.0
    bad = False
    for i in range(1, args.points + 1):
        input("③-%d 按回车闭合 %.0f°…" % (i, args.step_deg))
        dev.move_angle(args.step_deg, 1)    # 1 = 闭合
        time.sleep(args.wait)
        cmd_total += args.step_deg

        a = read_angle(dev, args.feedback_scale)
        if a is not None and a0 is not None:
            a_rel = a - a0
            # ★ 判据看【本步增量】，不看累计：
            #   电机刻度恒定偏 0.3% 时，每一步只差 1°，但累计误差会线性涨到 5~6°。
            #   那不是丢步，是标尺系数，不该判"不可信"（2026-11-09 现场 8 组数据都是这样）。
            step_act = a_rel - prev_a
            step_err = step_act - args.step_deg
            rel = abs(step_err) / max(1e-9, args.step_deg)
            ok_step = (abs(step_err) <= args.max_angle_error) or (rel <= 0.02)
            flag = "✅" if ok_step else "⚠️"
            if not ok_step:
                bad = True
            print("    [回读] 本步 指令 %6.1f° / 实走 %6.1f°（差 %+5.1f°）| "
                  "累计 指令 %7.1f° / 实际 %7.1f° %s"
                  % (args.step_deg, step_act, step_err, cmd_total, a_rel, flag))
            prev_a = a_rel
            angle_used = a_rel
        else:
            angle_used = cmd_total
            print("    [开环] 无反馈，暂用指令值 %7.1f°（不可信）" % cmd_total)

        w = float(input("    量当前开口(mm)，输入：").strip())
        samples.append((angle_used, w))
        print("    记录：角度 %7.1f° → %6.1f mm\n" % (angle_used, w))

    dev.close()

    # ---- 3) 拟合 ----
    print("=" * 66)
    if a0 is None:
        print(" ❌ 全程没读到反馈，本次是【开环标定】，结果不可用。")
        print("    先跑 --verify 排查拨码 2 号 / 供电 / 反馈倍率，再重标。")
        return 1
    if bad:
        print(" ❌ 有某一步的增量偏差超过 %.1f°（且 >2%%），本次标定数据不可信！"
              % args.max_angle_error)
        print("    常见原因：丢步（--speed 调小到 10）、供电不足、手指被挡、"
              "反馈倍率不对（--feedback-scale）。")
        print("    先跑 --verify 确认每次增量，再重标。")
        return 1
    try:
        k, dz, mx, err, model, rows = fit(w_max, samples,
                                          min_width=args.min_width)
    except ValueError as e:
        print(" ❌ 拟合失败：%s" % e)
        return 1
    print(" 拟合结果（带空程的分段线性模型，角度用回读实际值）")
    print("   deg_per_mm       = %.4f  °/mm   ← 线性区斜率" % k)
    print("   deadzone_deg     = %.1f    °   ← 机械空程（电机空转、手指不动）" % dz)
    print("   max_width_mm     = %.1f   mm" % mx)
    print("   稳定段最大残差   = %.2f   mm %s" %
          (err, "（<2mm 可用）" if err < 2.0 else "（偏大）"))
    print("   校验：40mm 开口 → %.1f°" % model.width_to_angle(40.0))

    # ---- 逐点残差表：当场指出哪一组读错/打滑 ----
    print("\n 逐点残差（预测 = w_max − max(0, 角度−空程)/k）")
    print("   %-11s %-9s %-9s %-9s %s" %
          ("累计角度", "实测mm", "预测mm", "残差mm", "备注"))
    for a, w, pred, d, stable in rows:
        note = ""
        if abs(d) > 3.0:
            note = "  ← 重点复查"
        elif not stable:
            note = "  (起始过渡 / 末端限位，不参与判定)"
        print("   %9.1f° %8.1f %8.1f %+8.1f%s" % (a, w, pred, d, note))

    if err > 3.0:
        print("\n ❌ 稳定段最大残差 %.2f mm > 3mm —— 本次数据【不可信】，不要填进配置！" % err)
        print("    （模型已自动扣除机械空程 %.1f° 和末端限位，"
              "剩下的残差多半是尺子读错/打滑）" % dz)
        print("    重标建议：每次闭合后加 --wait 3.5、同一位置量两次、"
              "全程单向闭合、加 --points 7。")
        print("=" * 66)
        return 1

    print("\n ✅ 把下面四行填进 config/real_params.yaml 的 gripper 段：")
    print("   calibration_done: true")
    print("   max_width_mm: %.1f" % mx)
    print("   deg_per_mm: %.4f" % k)
    print("   deadzone_deg: %.1f" % dz)

    # ---- 4) 空夹验证：用刚拟合出的参数，直接走串口，不必起 ROS ----
    done_inline = False
    if not args.skip_empty_grip:
        ans = input("\n④ 现在做一次空夹验证吗？（回车=做，n=跳过）：").strip()
        if ans.lower() != 'n':
            do_empty_grip(args, k, dz, mx, model)
            done_inline = True
    if not done_inline:
        print("\n ⚠️  手动空夹验证（下面是 ROS 话题命令，不是终端裸命令）：")
        print("    ros2 topic pub --once /gripper/cmd std_msgs/msg/String \"data: 'grip:40'\"")
        print("    或起 gripper_web.py，在 http://127.0.0.1:8777 页面上输入 40 点 Grip")
        print("    ⚠️ ros2 topic pub 支持 --once；ros2 topic echo 不支持，别搞混")
    print("=" * 66)
    return 0


def main():
    ap = argparse.ArgumentParser(description="MS42DC 夹爪开口标定")
    ap.add_argument("--port", default="/dev/motor_serial")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=1)
    ap.add_argument("--speed", type=float, default=20.0,
                    help="转速 rad/s（丢步就调小，如 10）")
    ap.add_argument("--home-angle", type=float, default=1900.0,
                    help="张开到底用的角度（靠限位/堵转停）")
    ap.add_argument("--step-deg", type=float, default=300.0,
                    help="每次闭合的角度增量（默认 300°）")
    ap.add_argument("--points", type=int, default=5, help="采样组数")
    ap.add_argument("--wait", type=float, default=2.5,
                    help="每次动作后等待秒数（等电机停稳再量）")
    ap.add_argument("--verify", action="store_true",
                    help="只验证「发 300° 是否真走 300°」+ 反馈倍率，不用尺子")
    ap.add_argument("--feedback-scale", type=float, default=1.0,
                    help="反馈角度倍率校正。--verify 显示增量是 30° 就填 10，"
                         "是 3000° 就填 0.1（手册表 3-15 有 10 倍疑点）")
    ap.add_argument("--max-angle-error", type=float, default=5.0,
                    help="指令累计与回读累计允许的最大偏差（度），超过判不可信")
    ap.add_argument("--grip-width", type=float, default=40.0,
                    help="空夹验证时模拟的物体宽度 mm（默认 40）")
    ap.add_argument("--min-width", type=float, default=10.0,
                    help="夹爪最小开口 mm（默认 10）；接近它的采样点视为机械限位，不参与残差判定")
    ap.add_argument("--pairs", default=None,
                    help='★ 直接喂已测数据做拟合，不接硬件。格式 '
                         '"开口mm@实际累计角,开口mm@实际累计角,..."，'
                         '例 "130@0,112@298.9,93@597.9,73@897,54@1196,34@1495.2,14@1794.2"'
                         '（第一组必须是 home 时的 w_max，角度写 0 或实际回读值）')
    ap.add_argument("--config", default=None,
                    help="real_params.yaml 路径，用于读 interference_mm（留空自动找）")
    ap.add_argument("--skip-empty-grip", action="store_true",
                    help="跳过脚本内的空夹验证交互（改由 ROS 话题/网页手动验证）")
    ap.add_argument("--dry-run", action="store_true", help="不打开串口（配合 --demo/--verify 演练）")
    ap.add_argument("--demo", action="store_true", help="不接硬件，演示拟合算法")
    a = ap.parse_args()
    if a.pairs:                       # ★ 先在 main 派发，避免 run() 里先开串口
        return run_pairs(a)
    return run(a)


if __name__ == "__main__":
    sys.exit(main())

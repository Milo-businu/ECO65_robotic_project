#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取任务总控 —— 把机械臂 / 夹爪 / 相机 / VLA 串成一条完整状态机

设计要点（这是"移植到真机就能跑"的关键）
------------------------------------------
  ① GraspTask 类本身【不依赖 rclpy】，只依赖三个抽象接口：
        arm        : move_joints / current_joints / stop
        gripper    : command / width / state
        perception : wait_target
     所以同一套状态机既能用 ROS adapter 跑真机，也能用桩对象离线演练。
  ② 所有"因机器而异"的参数都在 real_params.yaml，代码里零硬编码。
  ③ 默认 dry-run，真机要动必须同时满足 profile.mode=real
     且 profile.allow_arm_motion=true —— 两道锁，防止误动硬件。
  ④ 任何异常都走 abort()：发急停 + 记录原因，绝不悄悄继续。

状态机（和仿真 eff_test.py 一一对应）
  HOME → 张开夹爪 → 观察位姿 → 感知定位 → IK 规划(直线下降)
       → 预夹 → 下降 → 夹持(堵转) → 直线抬升
       →【放置】转运(抬→平移→降) → 张开释放 → 撤离 → (回 home)
  place.enabled=false 时是前 8 个阶段；true 时是 11 个阶段。
  放置点【不靠视觉】，是配置里示教出来的固定坐标（见 teach_place_point.py）。

运行
----
  # 演练（配置里 mode: dry-run 时是默认行为）
  python3 grasp_task.py --config config/real_params.yaml

  # 强制演练：仓库里的配置已被真机改成 mode: real 时，云服务器/新机器用这个
  python3 grasp_task.py --config config/real_params.yaml --dry-run

  # 真机（两道锁都打开后）
  python3 grasp_task.py --config config/real_params.yaml --real

  # 或 ros2 run arm_grasp grasp_task
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import (load_config, setup_import_paths, find_urdf,   # noqa: E402
                    is_dry_run, banner, fmt)
from planner import plan_grasp, plan_place, assert_envelope   # noqa: E402


# =====================================================================
#  状态机主体（纯 Python，不含 ROS）
# =====================================================================
class GraspTask:
    def __init__(self, cfg, kin, arm, gripper, perception, log=print,
                 vla=None, dry_run=True):
        self.cfg = cfg
        self.kin = kin
        self.arm = arm
        self.gripper = gripper
        self.perception = perception
        self.log = log
        self.vla = vla
        self.dry_run = dry_run
        self.aborted = False
        self.trace = []

    def _say(self, msg):
        self.trace.append(msg)
        self.log(msg)

    def _abort(self, reason):
        self.aborted = True
        self._say("!! 中止：%s" % reason)
        try:
            self.arm.stop()
            self._say("   已下发轨迹急停（夹爪保持不变，避免物体掉落）")
        except Exception as e:                       # noqa: BLE001
            self._say("   急停下发失败：%s" % e)

    def run(self) -> bool:
        cfg = self.cfg
        prof, robot = cfg["profile"], cfg["robot"]
        gcfg, pcfg, tcfg = cfg["gripper"], cfg["perception"], cfg["task"]
        box_mm = 2.0 * float(pcfg["box_half_size_m"]) * 1000.0   # 40mm（默认，被实测覆盖）
        half_m = float(pcfg["box_half_size_m"])                  # 物体半高（同上）
        place_cfg = cfg.get("place", {}) or {}
        place_on = bool(place_cfg.get("enabled", False))
        total = 11 if place_on else 8

        # 兜底：目标位置由示教给定（不依赖相机/手眼）。
        # 适用场景：手眼当天没收敛，但要保证「抓取+放置」整条任务能跑通。
        # 一旦这里非空，感知/手眼那两道门都不再是硬前提 —— 但仍然要过两道安全锁。
        fixed = tcfg.get("fixed_target_xyz")
        use_fixed = bool(fixed)
        if use_fixed:
            fixed = np.asarray(fixed, float).reshape(-1)[:3]
            if fixed.size != 3:
                self._say("❌ task.fixed_target_xyz 需要 3 个数（x, y, z）")
                return False

        def ph(i, txt):
            """打印阶段抬头：place 关时 /8，开时 /11，避免手写分母对不上。"""
            self._say("\n[%d/%d] %s" % (i, total, txt))

        self._say(banner("抓取任务  mode=%s  policy=%s  place=%s  目标源=%s" %
                         ("DRY-RUN（不动硬件）" if self.dry_run else "REAL（会动硬件！）",
                          tcfg.get("policy", "geometric"),
                          "开" if place_on else "关",
                          "示教固定点" if use_fixed else "视觉感知")))
        if not self.dry_run and not prof.get("allow_arm_motion", False):
            self._say("❌ profile.allow_arm_motion=false：两道锁只开了一道，"
                      "拒绝下发机械臂运动指令。")
            return False
        if not self.dry_run and not gcfg.get("calibration_done", False):
            self._say("⚠️ 警告：夹爪开口映射未标定，夹持开口只是粗估！"
                      "强烈建议先跑 calibrate_gripper.py")
        if not self.dry_run and not use_fixed and not cfg["handeye"].get("calibrated", False):
            self._say("❌ 手眼标定未完成，拒绝执行（会拿错误坐标去撞东西）。\n"
                      "   若今天手眼没收敛、只想先跑通任务：把 task.fixed_target_xyz "
                      "填成示教出来的方块顶面中心（走兜底路径，不依赖视觉）。")
            return False

        q_now = self.arm.current_joints()
        if q_now is None:
            # dry-run 用配置的 home 起步；真机必须有真实反馈
            if self.dry_run:
                q_now = np.asarray(robot["home_joints"], float)
                self._say("（演练）无关节反馈，用 home_joints 起步")
            else:
                self._say("❌ 收不到关节角：检查 rm_driver 的 udp_ip / joint_source")
                return False
        self._say("当前关节角 %s" % fmt(q_now))
        try:
            from planner import envelope_report
            bad = envelope_report(self.kin, [q_now], cfg)
            if bad:
                _i, _f, _o = bad[0]
                _k = int(np.argmax(_o))
                self._say("⚠️ 当前位姿的法兰 [%.3f %.3f %.3f] 已在包络外"
                          "（%s 超 %.0fmm）—— 会先慢慢回 home，别急停打断"
                          % (_f[0], _f[1], _f[2], "xyz"[_k], _o[_k] * 1000))
        except Exception:                          # noqa: BLE001
            pass

        try:
            # ---------- 1) 回 home + 张开夹爪 ----------
            ph(1, "回 home 位姿 + 夹爪张开")
            q_home = np.asarray(robot["home_joints"], float)
            assert_envelope(self.kin, [q_home], cfg, "[回 home]")
            ok, msg = self.arm.move_joints([q_now, q_home], [0.0, 4.0])
            self._say("   回 home: %s" % msg)
            if not ok:
                raise RuntimeError("回 home 失败")
            self.gripper.command("home")
            self.gripper.command("open")
            time.sleep(float(tcfg.get("settle_s", 1.0)))

            # ---------- 2) 观察位姿 ----------
            # 直接下发配置里的关节角（真机用 jog.py 摆好后、按 c 打印出来的）。
            # 不做 IK：观察姿势没有"工具竖直"约束，硬解会无解。
            ph(2, "移到观察位姿（配置里的关节角，不做 IK）")
            q_obs = np.asarray(tcfg.get("observe_joints",
                                        robot["home_joints"]), float)
            assert_envelope(self.kin, [q_obs], cfg, "[去观察位]")
            ok, msg = self.arm.move_joints([q_home, q_obs], [0.0, 5.0])
            self._say("   到观察位姿: %s  关节角 %s" % (msg, fmt(q_obs)))
            if not ok:
                raise RuntimeError("到观察位姿失败")
            time.sleep(float(tcfg.get("settle_s", 1.0)))

            # ---------- 3) 目标位置（视觉 or 示教固定点） ----------
            if use_fixed:
                ph(3, "使用示教固定目标（不走视觉，兜底路径）")
                target = fixed.copy()
                self._say("   目标顶面中心 [%.4f %.4f %.4f]" % tuple(target))
                self._say("   ⚠️ 这个坐标是人示教出来的：换了方块位置必须重新示教，"
                          "否则会撞空/撞偏。")
            else:
                ph(3, "感知定位（等待目标顶面中心）")
                target = self.perception.wait_target(timeout_s=25.0)
                self._say("   目标顶面中心 [%.4f %.4f %.4f]" % tuple(target))

            # ---------- 3.5) 可选 VLA 微调 ----------
            if tcfg.get("policy", "geometric") == "vla" and self.vla is not None:
                self._say("   [VLA] 请求 π0 动作块…")
                acts = self.vla.predict(self.perception.wrist_b64(),
                                        q_home, box_mm)
                if acts is None:
                    self._say("   [VLA] 服务不可用 → 回退 geometric 策略")
                else:
                    from vla_policy import VLAPolicy
                    d = VLAPolicy.action_to_delta(acts)
                    target = target + d[:3]
                    self._say("   [VLA] 目标微调后 %s（⚠️ 无场景视角，效果不保证）"
                              % fmt(target))

            # ---------- 4) 规划 ----------
            ph(4, "IK 规划（笛卡尔直线下降，避免关节空间横扫）")
            q_act = self.arm.current_joints()
            if q_act is None:
                q_act = q_obs
                self._say("   （无实时反馈，用观察位姿作为规划起点）")
            # ★ 2026-11-10：优先用感知给的实测尺寸（宽/高），拿不到才退回配置的 40mm
            _sz = getattr(self.perception, "obj_size", None)
            if _sz is not None and len(_sz) >= 2 and float(_sz[0]) > 0.005:
                box_mm = float(_sz[0]) * 1000.0
                # 高度不可信时（只看到顶面、侧脸没入镜）退回配置值，否则会抓空
                _h, _w = float(_sz[1]), float(_sz[0])
                if _h > 0.015:
                    # 永不比"半个窄宽"更深：万一顶面 z 被抬高，这里兜住；
                    # 对又高又窄的物体（瓶子）也只是抓上半段，属于合理抓法
                    half_m = 0.5 * min(_h, 1.05 * _w)
                else:
                    half_m = float(pcfg["box_half_size_m"])
                self._say("   实测尺寸：宽 %.1fmm  高 %.1fmm"
                          % (box_mm, half_m * 2000.0))
                _maxw = float(gcfg["max_width_mm"])
                _minw = float(gcfg["min_width_mm"])
                if box_mm > _maxw * 0.95:
                    raise RuntimeError(
                        "目标宽 %.0fmm 超过夹爪最大开口 %.0fmm → 拒绝抓取（别硬夹）"
                        % (box_mm, _maxw))
                if box_mm < _minw + 4.0:
                    raise RuntimeError(
                        "目标宽 %.0fmm 小于夹爪可夹下限 %.0fmm → 拒绝抓取"
                        % (box_mm, _minw))
            else:
                self._say("   未拿到实测尺寸 → 沿用配置值 %.0fmm（宽）" % box_mm)
            plan = plan_grasp(self.kin, q_act, target, cfg, obj_half=half_m)
            d = plan["diagnostics"]
            self._say("   方块中心 %s  路径点 %d  下降 %d 步  抬升 %d 步"
                      % (fmt(plan["center"]), len(plan["approach_waypoints"]),
                         d["descend_points"], d["lift_points"]))
            self._say("   最大关节跳变 %.3f rad，最小限位余量 %.3f rad"
                      % (d["jump"], d["min_limit_margin"]))
            if d["min_limit_margin"] <= 0:
                raise RuntimeError("路径点贴/超关节限位，拒绝执行")
            if d["jump"] > 1.0:
                raise RuntimeError("相邻路径点跳变 %.2f rad 过大（会突然翻腕）" % d["jump"])

            # ---------- 5) 预夹 ----------
            pre_mm = box_mm + float(gcfg["pregrip_margin_mm"])
            ph(5, "预夹：开口 %.1fmm（方块 %.1fmm，留余量）"
                      % (pre_mm, box_mm))
            self.gripper.width(pre_mm)

            # ---------- 6) 下降 ----------
            ph(6, "直线下降到抓取点")
            ok, msg = self.arm.move_joints(plan["approach_waypoints"],
                                           plan["approach_times"])
            self._say("   下降: %s" % msg)
            if not ok:
                raise RuntimeError("下降失败")
            time.sleep(float(tcfg.get("settle_s", 1.0)))

            # 下降后再看一眼：目标若被碰动就中止（真值 vs 当前感知）
            # 固定目标模式下没有感知，跳过（本来也没有可信的二次观测量）
            if use_fixed:
                self._say("   （固定目标模式：跳过下降后碰动检查）")
            elif tcfg.get("post_descend_check", False):
                after_tgt = None
                try:
                    after_tgt = self.perception.wait_target(timeout_s=4.0)
                except Exception as e: # noqa: BLE001
                    # 2026-11-10 现场：下降后相机已进入 D435i <0.28m 盲区，取不到数是预期行为，
                    # 不能当致命错误（之前把 wait_target 的 RuntimeError 又抛出去 → 误中止）
                    self._say(" （无二次观测，跳过碰动检查：%s）" % e)
                if after_tgt is not None:
                    shift = float(np.hypot(after_tgt[0] - target[0], after_tgt[1] - target[1]))
                    self._say("   下降后目标水平位移 %.1fmm" % (shift * 1000))
                    max_shift = float(tcfg.get("abort_on_target_shift_m", 0.01))
                    insane = float(tcfg.get("implausible_shift_m", 0.10))
                    if shift > insane:
                        self._say("   \u26a0\ufe0f 位移 %.0fmm 超出物理可能（相机近距深度会烂）"
                                  % (shift * 1000))
                        self._say("   → 判为【无效观测】，跳过碰动检查，继续执行")
                    elif shift > max_shift:
                        raise RuntimeError("下降途中目标被碰动（%.1fmm > 阈值）" % (shift * 1000))
            else:
                self._say("   （post_descend_check 关闭：下降后相机在 <0.28m 盲区，")
                self._say("     该高度上的二次观测实测不可信（2026-11-10：13/25/214mm 全是垃圾）")
                self._say("     → 跳过碰动检查；成功与否以抬升结果为准")

            # ---------- 7) 夹持（靠堵转 = 真机的力控） ----------
            grip_target = box_mm - float(gcfg["interference_mm"])
            ph(7, "夹持：目标开口 %.1fmm（过盈 %.1fmm → 顶到堵转）"
                  % (grip_target, float(gcfg["interference_mm"])))
            self.gripper.command("grip:%.1f" % box_mm, expect_width=grip_target)
            st = self.gripper.state()
            self._say("   夹爪状态 [开口 %.1f 角度 %.0f 到位 %d 堵转 %d]"
                      % (float(st[0]), float(st[1]), int(float(st[2])), int(float(st[3]))))
            # 到位 vs 堵转：这是「夹空 / 夹到」唯一可用的电学判据（无六维力）
            #   · 指间有方块 → 电机到不了目标开口 → 堵转=1、到位=0
            #   · 指间空的   → 电机走到目标开口 → 到位=1、堵转=0
            # ⚠️ 这两个标志的真实语义必须在现场先用「空夹 / 夹方块」各试一次确认
            #    （0:30–1:00 那一段做），确认前只当提示，不据此中止。
            if len(st) >= 4 and float(st[0]) > 0:
                if float(st[2]) > 0.5 and float(st[3]) < 0.5:
                    self._say(" · 夹爪报『到位 1 / 堵转 0』（电机走到了目标开口）")
                    self._say(" 本机是软橡胶指：过盈被指面压缩吸收，夹住时同样报『到位』")
                    self._say(" → 该判据在本机【无效】，真判据是抬升时方块是否跟着起来")
                elif float(st[3]) > 0.5:
                    self._say(" · 堵转 → 指间被顶住（本机软指基本不会触发）")

            # ---------- 8) 直线抬升 ----------
            ph(8, "保持夹持，笛卡尔直线抬升 %.2fm"
                  % float(tcfg["lift_height_m"]))
            ok, msg = self.arm.move_joints(plan["lift_waypoints"],
                                           plan["lift_times"])
            self._say("   抬升: %s" % msg)
            if not ok:
                raise RuntimeError("抬升失败")

            if not place_on:
                self._say("\n" + "=" * 62)
                self._say(" ✅ 抓取流程执行完毕（place.enabled=false，不做放置）" +
                          ("\n    （DRY-RUN：未动硬件，仅验证链路）" if self.dry_run else ""))
                self._say("=" * 62)
                return True

            # ---------- 9) 转运到放置区（抓取与放置的分界） ----------
            ph(9, "转运到放置区（竖直抬 → 水平移 → 竖直降，全程笛卡尔）")
            q_lift_end = self.arm.current_joints()
            if q_lift_end is None:
                q_lift_end = plan["lift_waypoints"][-1]
                self._say("   （无实时反馈，用抬升终点作为转运起点）")
            plan_p = plan_place(self.kin, q_lift_end, cfg, obj_half=half_m)
            dp = plan_p["diagnostics"]
            self._say("   放置点 %s（台面 %.3fm + 方块半高 %.3fm）"
                      % (fmt(plan_p["place_center"]), float(place_cfg.get("surface_z", 0.0)),
                         0.5 * box_mm / 1000.0))
            self._say("   转运 %d 点：抬 %d / 移 %.3fm (%d 点) / 降 %d；撤离 %d 点"
                      % (len(plan_p["transfer_waypoints"]), dp["up_points"],
                         dp["distance_m"], dp["across_points"], dp["down_points"],
                         dp["retreat_points"]))
            self._say("   最大关节跳变 %.3f rad，最小限位余量 %.3f rad"
                      % (dp["jump"], dp["min_limit_margin"]))
            for n in dp["notes"]:
                self._say("   ⚠️  %s" % n)
            if dp["min_limit_margin"] <= 0:
                raise RuntimeError("转运路径贴/超关节限位，拒绝执行")
            if dp["jump"] > 1.0:
                raise RuntimeError("转运相邻点跳变 %.2f rad 过大（会突然翻腕）" % dp["jump"])
            ok, msg = self.arm.move_joints(plan_p["transfer_waypoints"],
                                           plan_p["transfer_times"])
            self._say("   转运: %s" % msg)
            if not ok:
                raise RuntimeError("转运失败")
            time.sleep(float(tcfg.get("settle_s", 1.0)))

            # ---------- 10) 张开释放 ----------
            ph(10, "张开夹爪释放（目标开口 %.1fmm）"
                   % float(gcfg["open_width_mm"]))
            w_before = float(self.gripper.state()[0])
            self.gripper.command("open")
            time.sleep(float(place_cfg.get("release_settle_s", 0.8)))
            w_after = float(self.gripper.state()[0])
            self._say("   开口 %.1f → %.1fmm" % (w_before, w_after))
            if place_cfg.get("grip_check", True) and w_before > 0:
                # 张开后开口没变大 = 方块可能卡在指间/粘在指尖
                # 这时抬起会把方块带走，是最容易"看不见的失败"
                if w_after - w_before < 5.0:
                    self._say("   ⚠️ 开口几乎没变（%.1fmm）：方块可能卡在指间，"
                              "撤离时会被带起来。先手动确认再继续。"
                              % (w_after - w_before))
                else:
                    self._say("   ✅ 开口已张开，方块应已落在放置区")

            # ---------- 11) 撤离 + 可选回 home ----------
            ph(11, "撤离：竖直抬到台面上 %.2fm%s"
                   % (float(place_cfg.get("retreat_height_m", 0.20)),
                      "，然后回 home" if place_cfg.get("return_home", True) else ""))
            ok, msg = self.arm.move_joints(plan_p["retreat_waypoints"],
                                           plan_p["retreat_times"])
            self._say("   撤离: %s" % msg)
            if not ok:
                raise RuntimeError("撤离抬升失败")
            if place_cfg.get("return_home", True):
                q_home2 = np.asarray(robot["home_joints"], float)
                ok, msg = self.arm.move_joints(
                    [plan_p["retreat_waypoints"][-1], q_home2], [0.0, 5.0])
                self._say("   回 home: %s" % msg)
                if not ok:
                    raise RuntimeError("回 home 失败")
                self._say("   ⚠️  回 home 是关节空间走位（不是直线），"
                          "台面有高障碍物时把 place.return_home 改成 false")

            self._say("\n" + "=" * 62)
            self._say(" ✅ 抓取 + 放置流程执行完毕" +
                      ("（DRY-RUN：未动硬件，仅验证链路）" if self.dry_run else ""))
            self._say("=" * 62)
            return True

        except Exception as e:                      # noqa: BLE001
            self._abort("%s: %s" % (type(e).__name__, e))
            return False


# =====================================================================
#  离线桩对象（dry-run 用，不碰 rclpy / 不等任何话题）
# =====================================================================
class _StubLogger:
    def info(self, s): pass
    def warn(self, s): pass
    def error(self, s): pass


class StubNode:
    """给 DryRunBackend 用的最小 Node 替身（它只调 get_logger）。"""
    def __init__(self):
        self._log = _StubLogger()

    def get_logger(self):
        return self._log


class StubGripper:
    """模拟夹爪：记住开口、立刻"到位"，打印每条命令。"""

    def __init__(self, cfg, log=print):
        g = cfg["gripper"]
        self.g = g
        self.log = log
        self.width_mm = float(g["max_width_mm"])
        self.reached = 1.0
        self.stall = 0.0
        self.last_angle = 0.0

    def command(self, cmd, expect_width=None, timeout=None):
        cmd = str(cmd).strip().lower()
        if cmd == "home":
            self.width_mm = float(self.g["max_width_mm"])
        elif cmd == "open":
            self.width_mm = float(self.g["open_width_mm"])
        elif cmd == "close":
            self.width_mm = float(self.g["min_width_mm"])
        elif cmd.startswith("grip:"):
            obj = float(cmd.split(":", 1)[1])
            self.width_mm = obj - float(self.g["interference_mm"])
            self.stall = 1.0
        self.last_angle += 100.0
        self.log("   [夹爪-演练] %-14s → 开口 %.1fmm" % (cmd, self.width_mm))

    def width(self, mm):
        self.width_mm = float(mm)
        self.log("   [夹爪-演练] 目标开口 %.1fmm" % mm)

    def state(self):
        return [self.width_mm, self.last_angle, self.reached, self.stall]


class StubPerception:
    """模拟感知：固定给一个目标顶面中心，用于验证规划链路。"""

    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.log = log
        self.calls = 0

    def wait_target(self, timeout_s=25.0):
        self.calls += 1
        half = float(self.cfg["perception"]["box_half_size_m"])
        center = np.asarray(self.cfg["task"]["work_aim_m"], float)
        # 顶面中心 = 方块中心 + 半边长
        top = center + np.array([0.0, 0.0, half])
        self.log("   [感知-演练] 目标顶面中心 %s" % fmt(top))
        return top

    def wrist_b64(self):
        return None


# =====================================================================
#  ROS adapters（只有这一层碰 rclpy）
# =====================================================================
class ROSGripper:
    def __init__(self, node, cfg):
        from std_msgs.msg import String, Float64, Float64MultiArray
        g = cfg["gripper"]
        self.node = node
        self.cfg = cfg
        self.cmd_pub = node.create_publisher(String, g["cmd_topic"], 10)
        self.width_pub = node.create_publisher(Float64, g["width_topic"], 10)
        self.state_data = [-1.0, 0.0, 1.0, 0.0]   # [开口mm, 角度°, 到位, 堵转]
        self.state_time = 0.0
        node.create_subscription(Float64MultiArray, g["state_topic"],
                                 self._on_state, 10)
        self._String, self._Float64 = String, Float64

    def _on_state(self, m):
        self.state_data = [float(x) for x in m.data]
        self.state_time = time.time()

    def _wait_width(self, expect_mm, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (time.time() - self.state_time < 2.0
                    and abs(self.state_data[0] - expect_mm) < 3.0
                    and self.state_data[2] > 0.5):
                return True
            time.sleep(0.1)
        return False

    def command(self, cmd, expect_width=None, timeout=None):
        from std_msgs.msg import String
        m = String()
        m.data = str(cmd)
        self.cmd_pub.publish(m)
        t = timeout or float(self.cfg["gripper"]["move_timeout_s"]) + 3.0
        if expect_width is not None:
            self._wait_width(expect_width, t)
        else:
            time.sleep(min(2.0, t))

    def width(self, mm):
        from std_msgs.msg import Float64
        m = Float64()
        m.data = float(mm)
        self.width_pub.publish(m)
        self._wait_width(float(mm), float(self.cfg["gripper"]["move_timeout_s"]) + 3.0)

    def state(self):
        return list(self.state_data)


class ROSPerception:
    def __init__(self, node, cfg):
        from std_msgs.msg import Float64MultiArray
        from sensor_msgs.msg import Image
        p, c = cfg["perception"], cfg["camera"]
        self.cfg = cfg
        self.node = node
        self.target = None
        self.target_time = 0.0
        self.obj_size = None            # [宽, 高]（m），感知给的
        self._img = None
        node.create_subscription(Float64MultiArray, p["target_topic"],
                                 self._on_target, 10)
        node.create_subscription(Image, "%s/%s" % (c["prefix"].rstrip("/"),
                                                    c["color_suffix"]),
                                 self._on_img, 10)

    def _on_target(self, m):
        if len(m.data) >= 3:
            self.target = np.array([float(x) for x in m.data[:3]])
            if len(m.data) >= 5:
                self.obj_size = np.array([float(x) for x in m.data[3:5]])
            self.target_time = time.time()

    def _on_img(self, m):
        self._img = m

    def wait_target(self, timeout_s=25.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.target is not None and time.time() - self.target_time < 3.0:
                return self.target.copy()
            time.sleep(0.1)
        raise RuntimeError("等待 %s 超时（检查相机话题前缀 / 颜色 / 手眼标定）"
                           % self.cfg["perception"]["target_topic"])

    def wrist_b64(self):
        """把最新彩色图压成 base64 JPEG 给 VLA。cv2 缺失时返回 None。"""
        if self._img is None:
            return None
        try:
            import cv2
            import base64
            h, w = self._img.height, self._img.width
            buf = np.frombuffer(self._img.data, dtype=np.uint8)
            if len(buf) < h * w * 3:
                return None
            img = buf[:h * w * 3].reshape(h, w, 3)
            if "bgr" not in (self._img.encoding or "").lower():
                img = img[:, :, ::-1]
            ok, enc = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            return base64.b64encode(enc.tobytes()).decode("ascii") if ok else None
        except Exception:                            # noqa: BLE001
            return None


# =====================================================================
#  main
# =====================================================================
def main(argv=None):
    ap = argparse.ArgumentParser(description="ECO65 整机抓取任务总控")
    ap.add_argument("--config", default=None, help="real_params.yaml 路径")
    ap.add_argument("--real", action="store_true",
                    help="显式要求真机模式（还需配置里 allow_arm_motion=true）")
    ap.add_argument("--slow", action="store_true",
                    help="强制速度 5%%（第一次真机实跑用；比配置里的 10%% 更慢）")
    ap.add_argument("--dry-run", action="store_true",
                    help="强制演练模式（不接硬件），不受配置 profile.mode 影响。"
                         "仓库里的 real_params.yaml 若已被真机改成 mode: real，"
                         "云服务器/新机器跑演练就加这个开关")
    a, _ = ap.parse_known_args(argv)
    cfg = load_config(a.config)

    from arm_backend import make_backend

    setup_import_paths(cfg)
    from ik_check import UrdfKinematics
    kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                         cfg["robot"]["tip_frame"])

    # --dry-run 优先级最高：即使配置已被真机改成 mode: real，云服务器/新机器
    # 也能安全演练，不会误连真实硬件（--real 不会覆盖它）。
    if a.dry_run:
        dry_run = True
    else:
        dry_run = is_dry_run(cfg) and not a.real
        if a.real and is_dry_run(cfg):
            print("⚠️ --real 已给，但配置 profile.mode 仍是 dry-run；"
                  "请把 real_params.yaml 里改成 mode: real 才会真的动硬件。")

    # robot.joints 要给后端用（关节状态按名字取值），两处配置合并
    arm_cfg = dict(cfg["arm"])
    arm_cfg["joints"] = list(cfg["robot"]["joints"])
    if a.slow:
        arm_cfg["speed"] = 5          # 第一次实跑：慢到能跟得上
        print("⚠️ --slow：速度强制 5%%（配置里是 %s%%）" % cfg["arm"].get("speed"))

    vla = None
    if cfg["task"].get("policy") == "vla" and cfg["vla"].get("enabled"):
        from vla_policy import VLAPolicy
        vla = VLAPolicy(cfg)

    if dry_run:
        # 全程无 ROS：桩对象 + 演练后端，云机/Mac 上都能把状态机跑一遍
        n = StubNode()
        arm = make_backend(n, arm_cfg, dry_run=True)
        gripper = StubGripper(cfg)
        perception = StubPerception(cfg)
        log = lambda s: print(s, flush=True)        # noqa: E731
        task = GraspTask(cfg, kin, arm, gripper, perception,
                         log=log, vla=vla, dry_run=True)
        return 0 if task.run() else 1

    # ---- 真机：走 ROS ----
    import rclpy
    import threading
    from rclpy.node import Node

    class _N(Node):
        pass

    rclpy.init()
    n = _N("grasp_task")
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    arm = make_backend(n, arm_cfg, dry_run=False)
    gripper = ROSGripper(n, cfg)
    perception = ROSPerception(n, cfg)

    def log(s):
        print(s, flush=True)
        n.get_logger().info(s)

    task = GraspTask(cfg, kin, arm, gripper, perception,
                     log=log, vla=vla, dry_run=False)
    ok = task.run()
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真机感知节点 —— D435i 彩色 + 对齐深度 → 目标物在 base_link 系的 3D 位置

链路（和仿真 localize() 思路一致，只把「仿真相机姿态」换成「手眼标定 X」）：

  彩色图颜色分割 → 对齐深度反投影 → 相机光学系 3D 点
      ↓
  p_base = T_base←Link6(q当前) · X · p_cam      X = T_flange←camera_optical
      ↓
  取顶面高度(99 分位) + 顶面点 xy 包围盒中心 → 目标顶面中心

⚠️ 四个必须记住的坑
  1. 话题前缀由 real_params.yaml 的 camera.prefix 决定，
     d435i_eco65.launch.py 会【自动反推】camera_namespace，两者永远一致。
     拿不准就 `ros2 topic list | grep d435i` 看真实名字。
  2. 对齐深度单位是【毫米】(16UC1)，必须 /1000 才是米。
  3. D435i 最小可测距离约 0.28m，腕装相机太近会全黑/空洞。
  4. 【禁止缓存 T_base←flange】：机械臂一动它就变，每次都用当前 joint_states 现算 FK。

★ 现场调试（2026-11-04 补）
  - 颜色阈值不再写死，从 perception.color_thresholds 读；用 tune_perception.py
    可视化对准（现场光照/自动曝光会让它漂移）。
  - 每个「不发布」的分支都有节流诊断日志，/grasp/target_xyz 没数时看日志就知道卡在哪一环。

⚠️ 手眼未标定时（handeye.calibrated=false）本节点在 real 模式下拒绝输出，
   这是故意的：拿安装估计值去抓真机必撞。

运行：
  python3 perception_node.py --config /path/to/real_params.yaml
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (load_config, setup_import_paths, find_urdf,   # noqa: E402
                    handeye_matrix, is_dry_run, banner)


DEFAULT_THRESHOLDS = {
    "green": {"min": 100, "diff_r": 40, "diff_b": 40},
    "red":   {"min": 100, "diff_g": 40, "diff_b": 40},
    "blue":  {"min": 100, "diff_r": 30, "diff_g": 20},
}


def color_mask(img_rgb, which: str, thresholds=None):
    """返回 bool mask。img_rgb 是 (h,w,3) uint8，通道顺序已统一成 RGB。

    阈值从 real_params.yaml 的 perception.color_thresholds 读（现场光照会让它漂移）。
    没配就用 DEFAULT_THRESHOLDS，保持向后兼容。
    """
    r = img_rgb[:, :, 0].astype(int)
    g = img_rgb[:, :, 1].astype(int)
    b = img_rgb[:, :, 2].astype(int)
    cfg_t = thresholds or {}

    def build(name):
        t = dict(DEFAULT_THRESHOLDS[name])
        t.update(cfg_t.get(name) or {})
        return t

    tg, tr, tb = build("green"), build("red"), build("blue")
    masks = {
        "green": (g > tg["min"]) & (g > r + tg["diff_r"]) & (g > b + tg["diff_b"]),
        "red":   (r > tr["min"]) & (r > g + tr["diff_g"]) & (r > b + tr["diff_b"]),
        "blue":  (b > tb["min"]) & (b > r + tb["diff_r"]) & (b > g + tb["diff_g"]),
    }
    if which == "auto":
        best, best_n = None, 0
        for m in masks.values():
            n = int(m.sum())
            if n > best_n:
                best, best_n = m, n
        return best if best is not None else masks["green"]
    if which not in masks:
        raise ValueError("未知 target_color: %s（支持 green/red/blue/auto）" % which)
    return masks[which]


class PerceptionNode:
    def __init__(self, cfg, node):
        from sensor_msgs.msg import Image, CameraInfo, JointState
        from std_msgs.msg import Float64MultiArray
        from geometry_msgs.msg import PoseStamped

        self.cfg = cfg
        self.node = node
        self.dry_run = is_dry_run(cfg)
        c, p = cfg["camera"], cfg["perception"]
        self.prefix = c["prefix"].rstrip("/")
        self.img = None
        self.depth = None
        self.K = None
        self.q = None
        self.img_t = 0.0
        self.depth_t = 0.0
        self.q_t = 0.0
        self._lock = threading.Lock()
        self.samples = []
        self.last_publish = 0.0
        self._warn_t = {}          # 简易日志节流（Foxy 的 logger 没有 throttle_duration_sec）

        # 目标来源：color（颜色阈值，默认，最稳）| brain（端到端识别大模型，可选）
        self.target_source = str(p.get("target_source", "color")).lower()
        self.brain = None
        self._box = None
        self._box_t = 0.0
        self._box_info = ""
        self._box_label = ""

        node.create_subscription(Image, "%s/%s" % (self.prefix, c["color_suffix"]),
                                 self.on_img, 10)
        node.create_subscription(Image, "%s/%s" % (self.prefix, c["depth_suffix"]),
                                 self.on_depth, 10)
        node.create_subscription(CameraInfo, "%s/%s" % (self.prefix, c["info_suffix"]),
                                 self.on_info, 10)
        node.create_subscription(JointState, cfg["arm"]["joint_states_topic"],
                                 self.on_js, 10)

        self.pub = node.create_publisher(Float64MultiArray, p["target_topic"], 10)
        self.pose_pub = node.create_publisher(PoseStamped,
                                              p["target_topic"] + "_pose", 10)
        self._F64 = Float64MultiArray
        self._Pose = PoseStamped

        setup_import_paths(cfg)
        from ik_check import UrdfKinematics
        self.kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                                  cfg["robot"]["tip_frame"])
        self.X = handeye_matrix(cfg, allow_fallback=self.dry_run)
        self.joint_names = list(cfg["robot"]["joints"])

        node.get_logger().info(
            "感知节点就绪 话题前缀=%s 目标颜色=%s 目标来源=%s 手眼=%s" %
            (self.prefix, p["target_color"],
             self.target_source,
             "已标定" if cfg["handeye"].get("calibrated") else
             ("未标定(DRY-RUN 占位)" if self.dry_run else "未标定→拒绝输出")))
        _b = p.get("xy_bias_m") or [0.0, 0.0]
        if len(_b) >= 2 and (abs(float(_b[0])) > 1e-9 or abs(float(_b[1])) > 1e-9):
            node.get_logger().warn(
                "已启用常数偏置补偿 xy_bias_m = [%.4f, %.4f] m"
                "（只在确认是固定偏移时才用；换标定后必须清零）"
                % (float(_b[0]), float(_b[1])))

        # ---------- 可选：端到端识别大脑（brain） ----------
        if self.target_source == "brain":
            try:
                from brain import BrainClient
                self.brain = BrainClient(cfg, log=node.get_logger().info)
                if not self.brain.enabled:
                    node.get_logger().warn(
                        "perception.target_source=brain 但 brain.enabled=false → "
                        "自动退回颜色阈值模式")
                    self.target_source = "color"
                else:
                    node.get_logger().warn(
                        "目标来源 = 大脑（%s / %s）。颜色阈值仍可用，"
                        "把 target_source 改回 color 即可切回"
                        % (self.brain.model, self.brain.url))
                    threading.Thread(target=self._brain_loop, daemon=True).start()
            except Exception as e:                 # noqa: BLE001
                node.get_logger().warn("brain 初始化失败（%s）→ 退回颜色阈值" % e)
                self.target_source = "color"
        # ★ 2026-11-10 运行期切换目标颜色：说一句话就能换目标，不用重启感知
        from std_msgs.msg import String as _Str
        node.create_subscription(_Str, "/perception/target_color",
                                 self._on_set_color, 10)
        node.create_timer(0.2, self._tick)

    def _on_set_color(self, m):
        """运行期改 target_color。只接受 green/red/blue/auto。"""
        v = str(m.data).strip().lower()
        if v not in ("green", "red", "blue", "auto"):
            self.node.get_logger().warn(
                "忽略未知颜色 %r（只接受 green/red/blue/auto）" % v)
            return
        self.cfg["perception"]["target_color"] = v
        self.node.get_logger().warn("运行期切换 target_color → %s" % v)

    # ------------------------------------------------------------ 大脑线程
    def _brain_loop(self):
        """单独线程跑 HTTP 推理：绝不能在定时器里阻塞 spin（会丢图丢关节角）。"""
        interval = float((self.cfg.get("brain") or {}).get("interval_s", 0.5))
        import time as _t
        while True:
            _t.sleep(max(0.1, interval))
            with self._lock:
                img = self.img
            if img is None:
                continue
            try:
                box, label, info = self.brain.detect_box(img)
            except Exception as e:                 # noqa: BLE001
                box, label, info = None, "", "%s: %s" % (type(e).__name__, e)
            with self._lock:
                self._box = box
                self._box_t = _t.time()
                self._box_info = info
                self._box_label = label

    def _warn_throttled(self, key, msg, period=5.0):
        now = time.time()
        if now - self._warn_t.get(key, 0.0) < period:
            return
        self._warn_t[key] = now
        self.node.get_logger().warn(msg)

    # ------------------------------------------------------------ 回调
    def on_img(self, m):
        h, w = m.height, m.width
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if len(buf) < h * w * 3:
            return
        img = buf[:h * w * 3].reshape(h, w, 3)
        enc = (m.encoding or "").lower()
        if "bgr" in enc:
            img = img[:, :, ::-1].copy()      # 统一成 RGB
        with self._lock:
            self.img = img
            self.img_t = time.time()

    def on_depth(self, m):
        h, w = m.height, m.width
        enc = (m.encoding or "").lower()
        if "16uc1" in enc or "mono16" in enc:
            d = np.frombuffer(m.data, dtype=np.uint16)[:h * w].reshape(h, w)
            d = d.astype(np.float32) / 1000.0        # ⚠️ 毫米 → 米
        elif "32fc1" in enc:
            d = np.frombuffer(m.data, dtype=np.float32)[:h * w].reshape(h, w)
        else:
            return
        with self._lock:
            self.depth = d
            self.depth_t = time.time()

    def on_info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in self.joint_names):
            with self._lock:
                self.q = np.array([d[n] for n in self.joint_names], dtype=float)
                self.q_t = time.time()

    # ------------------------------------------------------------ 计算
    def _tick(self):
        p = self.cfg["perception"]
        now = time.time()
        with self._lock:
            img = self.img
            dep = self.depth
            K = self.K
            q = self.q.copy() if self.q is not None else None
            img_age = now - self.img_t
            dep_age = now - self.depth_t
            q_age = now - self.q_t
            box, box_t, box_info, box_label = (self._box, self._box_t,
                                               self._box_info, self._box_label)
        # 每一道「不发布」的分支都给节流诊断，别让现场对着空话题瞎猜
        if img is None or dep is None or K is None:
            self._warn_throttled("no_data",
                "等待图像数据：彩色图=%s 对齐深度=%s 内参=%s（检查 camera.prefix / align_depth.enable）"
                % (img is not None, dep is not None, K is not None), 5.0)
            return
        if q is None:
            # 没有关节角就不能算 FK —— 绝对不用旧位姿凑合
            self._warn_throttled("no_js",
                "收不到 %s：感知无法算 p_base。检查 rm_driver 的 udp_ip 是否填了本机 IP"
                % self.cfg["arm"]["joint_states_topic"], 5.0)
            return
        # ★ 新鲜度检查：话题"停更"时不能拿最后一帧继续算。
        #   /joint_states 尤其致命：臂动了但 pose 不更新 → 坐标全错，而且看起来一切正常。
        if q_age > 2.0:
            self._warn_throttled("stale_js",
                "/joint_states 已经 %.1fs 没更新 → 停止发布（rm_driver 挂了或 udp_ip 变了）"
                % q_age, 3.0)
            return
        if img_age > 1.5 or dep_age > 1.5:
            self._warn_throttled("stale_img",
                "图像/深度已经 %.1fs 没更新 → 停止发布（相机掉线？USB 带宽不足？）"
                % max(img_age, dep_age), 3.0)
            return
        if not self.dry_run and not self.cfg["handeye"].get("calibrated", False):
            self._warn_throttled("no_handeye",
                "手眼标定未完成，拒绝发布目标位姿（防止用错误坐标撞东西）", 10.0)
            return

        h, w = dep.shape
        if img.shape[:2] != (h, w):
            self._warn_throttled("shape",
                "彩色图 %s 与对齐深度 %s 尺寸不一致 → 确认 align_depth_to_color"
                % (img.shape[:2], (h, w)), 5.0)
            return
        # ---------- 目标来源：颜色阈值 / 端到端大脑 ----------
        src = self.target_source
        if src == "brain":
            bcfg = self.cfg.get("brain") or {}
            box_age = now - box_t
            if box is None or box_age > float(bcfg.get("max_age_s", 3.0)):
                msg = ("大脑还没给出目标（%s，最后一帧 %.1fs 前）"
                       % (box_info or "等待中", box_age if box_t else -1))
                if bcfg.get("fallback_to_color", True):
                    self._warn_throttled("brain_miss", msg + " → 本轮退回颜色阈值", 4.0)
                    src = "color"
                else:
                    self._warn_throttled("brain_miss", msg, 4.0)
                    return
            else:
                mask = self.brain.box_mask(img.shape, box)
                self._warn_throttled("brain_ok",
                    "大脑：%s 框 [%.0f %.0f %.0f %.0f]（%.1fs 前）"
                    % (box_label or "?", box[0], box[1], box[2], box[3], box_age), 8.0)
        if src == "color":
            mask = color_mask(img, p["target_color"], p.get("color_thresholds"))
        n_mask = int(mask.sum())
        if n_mask < int(p["min_pixels"]):
            self._warn_throttled("small_mask",
                "%s只命中 %d 像素（要求 >=%d，target_color=%s）→ "
                "用 tune_perception.py 对准阈值 / 检查光照与自动曝光 / 或看大脑返回"
                % ("大脑框" if src == "brain" else "颜色分割", n_mask,
                   int(p["min_pixels"]), p["target_color"]), 3.0)
            return

        vs, us = np.where(mask)
        z = dep[vs, us]
        good = np.isfinite(z) & (z > float(p["depth_min_m"])) & (z < float(p["depth_max_m"]))
        if good.sum() < 5:
            self._warn_throttled("no_depth",
                "mask 命中 %d 像素但有效深度只有 %d 个 → 检查 align_depth / "
                "目标是否在 0.3~0.6m / 反光 / 深度 0 值"
                % (n_mask, int(good.sum())), 3.0)
            return
        vs, us, z = vs[good], us[good], z[good]
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        # 相机光学系：x 右、y 下、z 前
        p_cam = np.column_stack([(us - cx) * z / fx,
                                 (vs - cy) * z / fy,
                                 z])
        T6 = self.kin.fk(q)                 # T_base←Link6（时变，每次现算）
        Tb = T6[:3, :3] @ self.X[:3, :3]
        tb = T6[:3, 3] + T6[:3, :3] @ self.X[:3, 3]
        p_base = p_cam @ Tb.T + tb

        # 4cm 立方体：用顶面(99 分位)求高度，只取顶面点算 xy 中心，避免侧面带偏
        # ★ 2026-11-10 抗旋转：顶面 z 取"最密的一层"，不用 99 分位。
        #   方块转 45° 时一个棱朝前，深度在棱上串色 → 少量点被抬到顶面之上，
        #   99 分位会抓住这些稀疏离群点（实测偏高 9mm → 抓空）。
        #   顶面是密集平面，串色是稀疏离群 → 用 2mm 直方图取最密的一桶即可分开。
        _z = p_base[:, 2]
        _z_hi = float(np.percentile(_z, 80))
        _cand = _z[_z >= _z_hi]
        if _cand.size >= 20:
            _bins = np.arange(float(_cand.min()), float(_cand.max()) + 0.002, 0.002)
            if _bins.size >= 2:
                _cnt, _edges = np.histogram(_cand, bins=_bins)
                _k = int(np.argmax(_cnt))
                z_top = float(0.5 * (_edges[_k] + _edges[_k + 1]))
            else:
                z_top = float(np.median(_cand))
        else:
            z_top = float(np.percentile(_z, 99))
        top = p_base[p_base[:, 2] > z_top - 0.004]
        if len(top) < 5:
            top = p_base
        x = 0.5 * (np.percentile(top[:, 0], 3) + np.percentile(top[:, 0], 97))
        y = 0.5 * (np.percentile(top[:, 1], 3) + np.percentile(top[:, 1], 97))
        # ★ 2026-11-10 尺寸感知：宽/高从这里出，夹爪开口与下降深度不再写死 40mm
        _wx = float(np.percentile(top[:, 0], 97) - np.percentile(top[:, 0], 3))
        _wy = float(np.percentile(top[:, 1], 97) - np.percentile(top[:, 1], 3))
        w_obj = min(_wx, _wy)                      # 夹爪应沿较窄方向闭合
        z_bot = float(np.percentile(p_base[:, 2], 2))
        h_obj = max(z_top - z_bot, 0.005)          # 只看到顶面时高度会退化，给 5mm 下限
        est = np.array([x, y, z_top, w_obj, h_obj])
        # 常数偏置补偿（默认 0）。只在「5 次抓取都把方块往同一个方向推」时使用：
        # 那不是阈值问题，是 X 的平移分量带了一个固定误差。
        # ⚠️ 偏置必须由【标定板/固定参考】量出来，绝不能拿方块真值去凑（工程伦理红线）。
        bias = p.get("xy_bias_m") or [0.0, 0.0]
        if len(bias) >= 2 and (abs(float(bias[0])) > 1e-9 or abs(float(bias[1])) > 1e-9):
            est[0] += float(bias[0])
            est[1] += float(bias[1])
        # 目标被挪走 / 换成另一个物体时，别拿旧样本一起平均
        # （否则会有一两秒"均值在两个位置中间"，抖动检查把它当噪声，表现为突然不出数）
        if len(self.samples) >= 3:
            m = np.mean(self.samples, axis=0)
            if float(np.hypot(est[0] - m[0], est[1] - m[1])) > 0.08:
                self.samples = []
        self.samples.append(est)
        n = int(p.get("samples", 12))
        self.samples = self.samples[-n:]
        if len(self.samples) < 3:
            return
        arr = np.asarray(self.samples)
        mean, std = arr.mean(axis=0), arr.std(axis=0)
        if float(np.max(std[:2])) > float(p.get("jitter_max_m", 0.02)):
            self._warn_throttled("jitter",
                "目标抖动 %.1fmm 过大，本轮不发布" % (np.max(std[:2]) * 1000), 3.0)
            return
        m = self._F64()
        m.data = [float(v) for v in mean]
        self.pub.publish(m)
        pose = self._Pose()
        pose.header.frame_id = self.cfg["robot"]["base_frame"]
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = \
            float(mean[0]), float(mean[1]), float(mean[2])
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        if time.time() - self.last_publish > 3.0:
            self.node.get_logger().info(
                "目标顶面中心 base_link = [%.4f %.4f %.4f] | %s命中%d px / "
                "有效深度%d 点 / 距离中值 %.3f m | 样本%d 抖动%.1fmm"
                % (mean[0], mean[1], mean[2],
                   "大脑框" if src == "brain" else "颜色", n_mask, len(z),
                   float(np.median(z)), len(self.samples),
                   np.max(std[:2]) * 1000))
            self.last_publish = time.time()


def main(argv=None):
    ap = argparse.ArgumentParser(description="D435i 真机目标感知节点")
    ap.add_argument("--config", default=None)
    a, _ = ap.parse_known_args(argv)
    cfg = load_config(a.config)
    import rclpy
    from rclpy.node import Node

    class _N(Node):
        pass

    rclpy.init()
    n = _N("perception_node")
    PerceptionNode(cfg, n)
    print(banner("感知节点已启动  前缀=%s  模式=%s" %
                 (cfg["camera"]["prefix"],
                  "DRY-RUN" if is_dry_run(cfg) else "REAL")))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

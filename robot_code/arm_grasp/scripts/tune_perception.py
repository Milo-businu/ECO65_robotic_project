#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tune_perception.py —— 现场 5 分钟把「认出方块」对准（只订阅，不下发任何指令）

为什么需要它：本项目识别方块靠的是 HSV/RGB 颜色阈值，不是训练出来的模型。
现场顶光、色温、D435i 自动曝光会让写死的阈值漂移 —— 到了现场第一件事
应该是【把方块放桌上，看着分割结果把阈值对准】，而不是直接跑抓取。

它同时把感知链路的 5 道门逐个点亮，/grasp/target_xyz 没数时能立刻定位卡在哪：
  [1] 彩色图   [2] 对齐深度   [3] camera_info   [4] /joint_states   [5] 手眼 calibrated

用法（真机有桌面，推荐）：
  python3 tune_perception.py --color green
  # 拖 3 个滑块让方块被完整高亮、且背景尽量干净；按 s 打印/保存建议阈值；按 q 退出

用法（无桌面 / 远程）：
  python3 tune_perception.py --color green --headless --seconds 20

用法（只看链路状态，不调阈值）：
  python3 tune_perception.py --check-only --seconds 8
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (load_config, setup_import_paths, find_urdf,   # noqa: E402
                    handeye_matrix)
from perception_node import color_mask, DEFAULT_THRESHOLDS   # noqa: E402

KEYS = {
    "green": ("min", "diff_r", "diff_b"),
    "red":   ("min", "diff_g", "diff_b"),
    "blue":  ("min", "diff_r", "diff_g"),
}


def main():
    ap = argparse.ArgumentParser(description="现场颜色阈值对准 + 感知链路自检")
    ap.add_argument("--config", default=None, help="real_params.yaml 路径")
    ap.add_argument("--color", choices=["green", "red", "blue", "auto"],
                    default=None, help="默认从配置的 target_color 读")
    ap.add_argument("--headless", action="store_true", help="不开窗口，只打印统计")
    ap.add_argument("--check-only", action="store_true", help="只检查 5 道门，不调阈值")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="自动退出秒数；0 = 一直跑（按 q 退出）")
    ap.add_argument("--write-config", action="store_true",
                    help="按 s 保存时【直接写进 real_params.yaml】的 color_thresholds"
                         "（自动备份）。不加就只打印，需要手抄")
    a = ap.parse_args()

    cfg = load_config(a.config)
    c, p = cfg["camera"], cfg["perception"]
    color = a.color or p.get("target_color", "green")
    if color == "auto":
        color = "green"       # 调阈值时钉死一个颜色，auto 会来回跳
    prefix = c["prefix"].rstrip("/")
    base_t = dict(DEFAULT_THRESHOLDS[color])
    base_t.update((p.get("color_thresholds") or {}).get(color) or {})

    try:
        import cv2
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image, CameraInfo, JointState
    except ImportError as e:
        print("❌ 缺依赖：%s（source /opt/ros/foxy/setup.bash，装 python3-opencv）" % e)
        return 1

    rclpy.init()
    node = Node("tune_perception")
    st = {"img": None, "dep": None, "K": None, "q": None, "n_img": 0,
          "n_dep": 0, "n_info": 0, "n_js": 0}

    def on_img(m):
        h, w = m.height, m.width
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if len(buf) < h * w * 3:
            return
        img = buf[:h * w * 3].reshape(h, w, 3)
        if "bgr" in (m.encoding or "").lower():
            img = img[:, :, ::-1].copy()
        st["img"] = img
        st["n_img"] += 1

    def on_depth(m):
        h, w = m.height, m.width
        enc = (m.encoding or "").lower()
        if "16uc1" in enc or "mono16" in enc:
            d = np.frombuffer(m.data, dtype=np.uint16)[:h * w].reshape(h, w)
            st["dep"] = d.astype(np.float32) / 1000.0
        elif "32fc1" in enc:
            st["dep"] = np.frombuffer(m.data, dtype=np.float32)[:h * w].reshape(h, w)
        else:
            return
        st["n_dep"] += 1

    def on_info(m):
        st["K"] = np.array(m.k, dtype=np.float64).reshape(3, 3)
        st["n_info"] += 1

    def on_js(m):
        d = dict(zip(m.name, m.position))
        need = list(cfg["robot"]["joints"])
        if all(n in d for n in need):
            st["q"] = np.array([float(d[n]) for n in need])
            st["n_js"] += 1

    node.create_subscription(Image, "%s/%s" % (prefix, c["color_suffix"]), on_img, 10)
    node.create_subscription(Image, "%s/%s" % (prefix, c["depth_suffix"]), on_depth, 10)
    node.create_subscription(CameraInfo, "%s/%s" % (prefix, c["info_suffix"]), on_info, 10)
    node.create_subscription(JointState, cfg["arm"]["joint_states_topic"], on_js, 10)

    # 手眼矩阵：仅已标定时才用于显示 p_base，绝不拿 fallback 占位值冒充
    kin, X = None, None
    try:
        setup_import_paths(cfg)
        from ik_check import UrdfKinematics
        kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                             cfg["robot"]["tip_frame"])
        X = handeye_matrix(cfg, allow_fallback=False)
    except Exception:                              # noqa: BLE001
        X = None

    headless = a.headless or not os.environ.get("DISPLAY")
    keys = KEYS[color]
    win = None
    if not headless and not a.check_only:
        try:
            cv2.namedWindow("tune", cv2.WINDOW_NORMAL)
            cv2.createTrackbar("min", "tune", int(base_t[keys[0]]), 255, lambda v: None)
            cv2.createTrackbar("lead1", "tune", int(base_t[keys[1]]), 150, lambda v: None)
            cv2.createTrackbar("lead2", "tune", int(base_t[keys[2]]), 150, lambda v: None)
            win = "tune"
        except Exception as e:                     # noqa: BLE001
            print("⚠️ 开不了窗口（%s），退回 headless" % e)
            headless = True

    print("=" * 66)
    print(" 现场感知对准  颜色=%s  前缀=%s  %s"
          % (color, prefix, "headless" if headless else "窗口模式"))
    print(" 把方块放在作业区，让它在彩色图里完整可见；目标距离 0.3~0.6m")
    print("=" * 66)

    t0 = time.time()
    last_print = 0.0
    final_t = dict(base_t)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            t = dict(base_t)
            if win and not a.check_only:
                t[keys[0]] = cv2.getTrackbarPos("min", win)
                t[keys[1]] = cv2.getTrackbarPos("lead1", win)
                t[keys[2]] = cv2.getTrackbarPos("lead2", win)
            final_t = t

            img, dep, K, q = st["img"], st["dep"], st["K"], st["q"]
            n_mask, n_good, z_med, p_base_txt = 0, 0, float("nan"), "（待手眼标定）"
            overlay = None
            if img is not None and not a.check_only:
                mask = color_mask(img, color, {color: t})
                n_mask = int(mask.sum())
                if dep is not None and img.shape[:2] == dep.shape:
                    z = dep[mask]
                    good = z[np.isfinite(z) & (z > float(p["depth_min_m"]))
                             & (z < float(p["depth_max_m"]))]
                    n_good = int(len(good))
                    if len(good):
                        z_med = float(np.median(good))
                if not headless:
                    bgr = img[:, :, ::-1].copy()
                    bgr[mask] = (0.6 * bgr[mask] + 0.4 * np.array([0, 255, 0])).astype(np.uint8)
                    overlay = bgr
            if img is not None and K is not None and q is not None and X is not None \
                    and dep is not None and n_good > 0:
                vs, us = np.where(color_mask(img, color, {color: t}))
                zz = dep[vs, us]
                ok = np.isfinite(zz) & (zz > p["depth_min_m"]) & (zz < p["depth_max_m"])
                if ok.sum() > 5:
                    u, v, z = us[ok], vs[ok], zz[ok]
                    p_cam = np.column_stack([(u - K[0, 2]) * z / K[0, 0],
                                             (v - K[1, 2]) * z / K[1, 1], z])
                    T6 = kin.fk(q)
                    R = T6[:3, :3] @ X[:3, :3]
                    tt = T6[:3, 3] + T6[:3, :3] @ X[:3, 3]
                    pb = p_cam @ R.T + tt
                    ztop = float(np.percentile(pb[:, 2], 99))
                    top = pb[pb[:, 2] > ztop - 0.004]
                    if len(top) < 5:
                        top = pb
                    p_base_txt = "[%.3f %.3f %.3f]" % (
                        0.5 * (np.percentile(top[:, 0], 3) + np.percentile(top[:, 0], 97)),
                        0.5 * (np.percentile(top[:, 1], 3) + np.percentile(top[:, 1], 97)),
                        ztop)

            z_txt = "%.3fm" % z_med if np.isfinite(z_med) else "--"
            if overlay is not None:
                label = "mask=%d  depth=%d  z=%s" % (n_mask, n_good, z_txt)
                cv2.putText(overlay, label, (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
                cv2.putText(overlay, label, (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            if time.time() - last_print > 1.0:
                last_print = time.time()
                gates = "[1]图%s [2]深%s [3]内参%s [4]关节%s [5]手眼%s" % (
                    "OK" if st["n_img"] else "--", "OK" if st["n_dep"] else "--",
                    "OK" if K is not None else "--", "OK" if q is not None else "--",
                    "OK" if X is not None else "未标定")
                print("  %s | 阈值 min=%d lead=%d/%d | mask=%d(需>=%d) 有效深度=%d z=%s | p_base=%s"
                      % (gates, t[keys[0]], t[keys[1]], t[keys[2]],
                         n_mask, int(p["min_pixels"]), n_good, z_txt, p_base_txt))
                if st["n_img"] and not st["n_dep"]:
                    print("     → 对齐深度没数据：确认 align_depth.enable:=true")
                if q is None:
                    print("     → 关节角没数据：检查 rm_eco65_config.yaml 的 udp_ip")
                if X is None:
                    print("     → 感知节点会拒绝发布；先完成手眼标定并填 matrix_4x4 / calibrated")

            if overlay is not None:
                cv2.imshow(win, overlay)
                k = cv2.waitKey(30) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("s"):
                    _save(color, keys, final_t)
                    if a.write_config:
                        _patch_config(color, keys, final_t, a.config)
            if a.seconds > 0 and time.time() - t0 > a.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if win:
            try:
                cv2.destroyAllWindows()
            except Exception:                      # noqa: BLE001
                pass
        node.destroy_node()
        rclpy.shutdown()

    print("\n" + "=" * 66)
    print(" 建议填进 real_params.yaml 的 perception.color_thresholds：")
    _emit(color, keys, final_t)
    print("=" * 66)
    return 0


def _emit(color, keys, t):
    print("   %s: {min: %d, %s: %d, %s: %d}"
          % (color, t[keys[0]], keys[1], t[keys[1]], keys[2], t[keys[2]]))


def _patch_config(color, keys, t, cfg_path=None):
    """把阈值写进 real_params.yaml 的 perception.color_thresholds.<color>。

    只改那一行内联 flow 映射，其它注释与顺序原样保留（yaml.dump 会全删注释）。
    """
    import re as _re
    import shutil as _sh
    import time as _t
    from common import DEFAULT_CONFIG
    path = os.path.expanduser(cfg_path or DEFAULT_CONFIG)
    if not os.path.isfile(path):
        print("   ❌ 找不到 %s，阈值没写进去" % path)
        return False
    lines = open(path, encoding="utf-8").read().splitlines()
    out, in_ct, hit = [], False, False
    for ln in lines:
        if _re.match(r"^\s*color_thresholds:\s*$", ln):
            in_ct = True
            out.append(ln)
            continue
        if in_ct and _re.match(r"^[A-Za-z_]", ln):      # 出了这个段
            in_ct = False
        m = _re.match(r"^(\s*)%s:\s*\{.*\}\s*(#.*)?$" % color, ln) if in_ct else None
        if m and not hit:
            out.append("%s%s: {min: %d, %s: %d, %s: %d}%s"
                       % (m.group(1), color, int(t[keys[0]]), keys[1],
                          int(t[keys[1]]), keys[2], int(t[keys[2]]),
                          ("  " + m.group(2)) if m.group(2) else ""))
            hit = True
            continue
        out.append(ln)
    if not hit:
        print("   ❌ 没在 color_thresholds 段里找到 %s 那一行" % color)
        return False
    _sh.copy2(path, path + ".bak.%s" % _t.strftime("%Y%m%d-%H%M%S"))
    open(path, "w", encoding="utf-8").write("\n".join(out) + "\n")
    print("   [已写入] %s 的 perception.color_thresholds.%s" % (path, color))
    print("   ⚠️ 感知节点要重启才生效（配置只在启动时读一次）")
    return True


def _save(color, keys, t):
    import yaml
    path = os.path.expanduser("~/perception_thresholds.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump({"color_thresholds": {
            color: {keys[0]: int(t[keys[0]]), keys[1]: int(t[keys[1]]),
                    keys[2]: int(t[keys[2]])}}}, f, allow_unicode=True)
    print("   [已保存] %s" % path)
    _emit(color, keys, t)


if __name__ == "__main__":
    sys.exit(main())

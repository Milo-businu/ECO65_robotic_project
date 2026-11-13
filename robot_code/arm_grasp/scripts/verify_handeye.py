#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_handeye.py —— 手眼标定的「物理打点验证 + 常数偏置求解」

为什么必须做，且为什么不能靠尺子
--------------------------------
  残差小只说明「数据自洽」，不说明「坐标对」。AX=XB 完全可以在数据本身有
  系统偏差（板尺寸量错、纸翘、tcp_offset 不准）时给出残差 0.3mm 的解，
  一上真机整体偏 8mm。

  拿尺子量「XYZ 是不是真的在 0.35/-0.20」很难做到 1mm 级：没有基准边。
  本脚本换了个基准——**机械臂自己的编码器**：

    ① 用 X 算出「标记中心在基座系的位置 P」，把指尖开到 P 正上方
    ② 人用键盘（w/s/a/d，1mm 步长）把指尖微调到【正好压在标记中心上】
    ③ 读这一刻的关节角 → FK → 指尖真实位置 P'
    ④ 误差 c = P' − P

  P' 完全不经过相机与手眼矩阵，所以这不是循环论证：它测的正是
  「相机算出来的坐标」与「机械臂实际走到的位置」之间的差。
  标定板是标定基准物，不是被测对象 —— 不存在「拿物体真值喂程序」的问题。

  在作业区不同位置重复 3~5 次：
    · c 基本一样 → 常数偏置，直接补偿掉（写 perception.xy_bias_m）
    · c 忽大忽小、方向乱 → 旋转/尺度有问题，必须重标，补偿救不了

前置
----
  · 相机在跑（彩色图 + camera_info）
  · rm_driver 在跑（/joint_states）
  · **不要**同时开 grasp_task / 夹爪标定（会抢串口或抢控制权）
  · 两道锁要打开（本脚本会动机械臂）：profile.mode=real + allow_arm_motion=true
  · A4 ArUco 板贴在桌面、固定不动

用法
----
  python3 robot_code/arm_grasp/scripts/verify_handeye.py
  python3 robot_code/arm_grasp/scripts/verify_handeye.py --points 5 --gap 0.02
  python3 robot_code/arm_grasp/scripts/verify_handeye.py --dry-run     # 只算不动作
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------- ArUco
def make_detector(marker_size, dict_name="auto", marker_id=None):
    """返回 detect(bgr, K, D) -> (T_cam←marker 4x4 | None, 说明)"""
    import cv2
    import cv2.aruco as A

    names = ["DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250",
             "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250",
             "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250",
             "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250",
             "DICT_ARUCO_ORIGINAL"]
    dicts = []
    for n in names:
        if dict_name != "auto" and dict_name.upper() != n:
            continue
        v = getattr(A, n, None)
        if v is None:
            continue
        try:
            d = (A.Dictionary_get(v) if hasattr(A, "Dictionary_get")
                 else A.getPredefinedDictionary(v))
            dicts.append((n, d))
        except Exception:                          # noqa: BLE001
            pass
    locked = {"d": None}

    def detect(bgr, K, D, want_id=None):
        if not dicts:
            return None, "没有可用的 ArUco 字典", []
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        cands = [locked["d"]] if locked["d"] else dicts
        for name, dic in cands:
            try:
                corners, ids, _ = A.detectMarkers(gray, dic)
            except Exception:                      # noqa: BLE001
                continue
            if ids is None or len(ids) == 0:
                continue
            id_list = sorted(int(v) for v in ids.ravel())
            want = want_id if want_id is not None else marker_id
            pick = 0
            if want is not None:
                hits = [i for i, v in enumerate(ids.ravel()) if int(v) == int(want)]
                if not hits:
                    continue
                pick = hits[0]
            c = corners[pick]
            try:
                res = A.estimatePoseSingleMarkers([c], marker_size, K, D)
                rvec, tvec = res[0][0], res[1][0]
            except Exception as e:                 # noqa: BLE001
                return None, "estimatePose 失败：%s" % e
            Rm, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = Rm
            T[:3, 3] = np.asarray(tvec).ravel()
            locked["d"] = (name, dic)
            return T, "dict=%s id=%d" % (name, int(ids.ravel()[pick])), id_list
        return None, "没检测到 ArUco（检查：完整入镜 / 纸贴平 / 光照 / --marker-size 实量）", []

    return detect


# ---------------------------------------------------------------- 主流程
def main(argv=None):
    ap = argparse.ArgumentParser(description="手眼标定物理打点验证 + 常数偏置求解")
    ap.add_argument("--config", default=None)
    ap.add_argument("--points", type=int, default=5,
                    help="验证点数（聚焦模式要求 >=5，覆盖作业区四角+中心）")
    ap.add_argument("--gap", type=float, default=0.02,
                    help="指尖停在标记上方多少米（默认 0.02，便于眼睛判断对中）")
    ap.add_argument("--marker-size", type=float, default=None,
                    help="ArUco 外框实量边长（默认从 samples json 读，读不到用 0.05）")
    ap.add_argument("--each-marker", action="store_true",
                    help="★ 标定板固定不能挪时用：把纸上的多个标记依次当验证点，"
                         "板子不用动（配合 --points 4 正好用完一张 A4 的四个标记）")
    ap.add_argument("--out", default="~/handeye_probe.json", help="结果留档")
    ap.add_argument("--dry-run", action="store_true", help="只算 IK 不动作（不看相机也行）")
    a = ap.parse_args(argv)

    from common import load_config, setup_import_paths, find_urdf, handeye_matrix, is_dry_run
    from planner import ik_aligned, tip_position

    cfg = load_config(a.config)
    setup_import_paths(cfg)
    from ik_check import UrdfKinematics

    kin = UrdfKinematics(find_urdf(cfg), cfg["robot"]["base_frame"],
                         cfg["robot"]["tip_frame"])
    offset = np.asarray(cfg["robot"]["tcp_offset"], float)
    lo = np.asarray(cfg["robot"]["joint_limits_lo"], float)
    hi = np.asarray(cfg["robot"]["joint_limits_hi"], float)

    if not cfg["handeye"].get("calibrated") and not a.dry_run:
        print("❌ handeye.calibrated=false：还没有 X，验什么？先把标定写进配置")
        return 2
    X = handeye_matrix(cfg, allow_fallback=is_dry_run(cfg) or a.dry_run)

    # marker 尺寸：优先命令行 → samples json 的 _readme 里没结构化存 → 默认 0.05
    marker_size = float(a.marker_size if a.marker_size
                        else (cfg.get("handeye") or {}).get("marker_size_m") or 0.05)

    print("=" * 62)
    print(" 手眼标定 物理打点验证（用机械臂编码器当基准，不用尺子）")
    print("=" * 62)
    print(" X 平移 = %s" % np.round(X[:3, 3], 4).tolist())
    print(" marker 边长 = %.4f m%s" % (marker_size, "" if a.marker_size else "（默认值，最好用 --marker-size 实量）"))

    if a.dry_run:
        print("\n（--dry-run：只验证 IK 与坐标链，不动机械臂、不读相机）")
        q0 = np.asarray(cfg["robot"]["home_joints"], float)
        print(" home 时指尖位置 %s" % np.round(tip_position(kin, q0, offset), 3).tolist())
        print(" 这一步只能证明代码没崩；真正的偏差必须上机测。")
        return 0

    # ---------- 安全锁 ----------
    prof = cfg["profile"]
    if is_dry_run(cfg) or not prof.get("allow_arm_motion", False):
        print("\n❌ 本脚本会动机械臂，需要两道锁同时打开：")
        print("   profile.mode: real   +   profile.allow_arm_motion: true")
        return 2

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo, JointState

    rclpy.init()
    node = Node("verify_handeye")
    state = {"img": None, "K": None, "D": None, "q": None}
    names = list(cfg["robot"]["joints"])
    c = cfg["camera"]
    prefix = c["prefix"].rstrip("/")
    import cv2

    def on_img(m):
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if len(buf) < m.height * m.width * 3:
            return
        img = buf[:m.height * m.width * 3].reshape(m.height, m.width, 3)
        state["img"] = img.copy() if "bgr" in (m.encoding or "").lower() \
            else img[:, :, ::-1].copy()

    def on_info(m):
        if state["K"] is None:
            state["K"] = np.array(m.k, dtype=np.float64).reshape(3, 3)
            state["D"] = np.array(m.d, dtype=np.float64) if m.d else None

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in names):
            state["q"] = np.array([d[n] for n in names], dtype=float)

    node.create_subscription(Image, "%s/%s" % (prefix, c["color_suffix"]), on_img, 10)
    node.create_subscription(CameraInfo, "%s/%s" % (prefix, c["info_suffix"]), on_info, 10)
    node.create_subscription(JointState, cfg["arm"]["joint_states_topic"], on_js, 10)

    # ★ 关键：必须开后台 spin 线程。
    #   move_joints() 是【阻塞等 movej_result】的，如果主线程只在输入处 spin_once，
    #   等结果的那段时间没有任何回调被处理 → 永远超时（现场表现为"机械臂明明能动"）。
    import threading
    _stop_spin = threading.Event()

    def _spin_forever():
        while not _stop_spin.is_set():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception:                      # noqa: BLE001
                break
    threading.Thread(target=_spin_forever, daemon=True).start()

    from arm_backend import make_backend
    arm_cfg = dict(cfg["arm"])
    arm_cfg["joints"] = names
    arm = make_backend(node, arm_cfg, dry_run=False)

    detect = make_detector(marker_size)

    def spin(sec=0.3):
        # 后台线程已经在 spin，这里只是等数据到齐
        time.sleep(sec)

    # 等数据
    t0 = time.time()
    while time.time() - t0 < 10 and (state["img"] is None or state["q"] is None):
        spin(0.2)
    if state["q"] is None:
        print("❌ 没有 /joint_states —— 先跑 net_autoconf.py 把 udp_ip 配对好")
        return 1
    if state["img"] is None:
        print("❌ 没有彩色图 —— 相机没起或前缀不对")
        return 1
    print(" ✅ 相机与关节角都有数据")

    results = []
    for k in range(a.points):
        print("\n" + "-" * 62)
        print(" 第 %d/%d 个点" % (k + 1, a.points))
        print("-" * 62)
        if k == 0:
            print(" ① 确认相机能看见 A4 标记（看不见就先 Ctrl+C，用 jog.py 摆好臂再来），回车")
        elif a.each_marker:
            print(" ① 板子不用动；这一点会去碰纸上的另一个标记（指尖会自己移过去），回车")
        else:
            print(" ① 把 A4 板整体挪到作业区另一个位置（机械臂不用动，挪 10~15cm）；"
                  "板子固定死的话改用 --each-marker，回车")
        try:
            input("    回车继续 > ")
        except (EOFError, KeyboardInterrupt):
            break
        spins = 0
        while spins < 30:
            spin(0.2)
            spins += 1
        if state["img"] is None or state["K"] is None:
            print("   ❌ 图像/内参还没到，跳过这一点")
            continue
        Tm, note, ids_all = detect(state["img"], state["K"], state["D"])
        if Tm is None:
            print("   ❌ %s" % note)
            continue
        if a.each_marker and len(ids_all) > 1:
            # ★ 标定板固定死时用它：同一张纸上的 4 个标记当 4 个不同的验证点，
            #   板子完全不用动，且这 4 点在纸上分散开（约 20×15cm），
            #   正好等价于"挪板子"的空间覆盖。
            want = ids_all[k % len(ids_all)]
            Tm2, note2, _ = detect(state["img"], state["K"], state["D"], want_id=want)
            if Tm2 is None:
                print("   ❌ 指定的 marker id=%d 检测失败" % want)
                continue
            Tm, note = Tm2, note2
            print("   （--each-marker：本点用 id=%d，本帧可见 %s）" % (want, ids_all))
        # p_base = FK(q)·X·t_marker（只用平移，因为 ArUco 原点就是标记中心）
        q1 = state["q"].copy()
        T6 = kin.fk(q1)
        p_marker = (T6[:3, :3] @ (X[:3, :3] @ Tm[:3, 3] + X[:3, 3]) + T6[:3, 3])
        print("   %s → 标记中心在基座系 [%.4f %.4f %.4f]"
              % (note, p_marker[0], p_marker[1], p_marker[2]))

        p_tcp = np.array([p_marker[0], p_marker[1], p_marker[2] + a.gap])
        s = ik_aligned(kin, q1, p_tcp, offset, lo, hi)
        if s is None:
            print("   ❌ IK 无解：标记位置超出可达区，或 gap 太高。换个更近的标记位置")
            continue
        plan = [q1, s[0]]
        # 先抬到 8cm 再水平过去，别贴着桌面横扫
        high = ik_aligned(kin, q1, np.array([p_tcp[0], p_tcp[1], max(p_tcp[2], p_marker[2] + 0.10)]),
                          offset, lo, hi)
        if high is not None:
            plan = [q1, high[0], s[0]]
        print(" ② 指尖会移到标记正上方 %.0fmm 处（低速）…" % (a.gap * 1000))
        ok, msg = arm.move_joints(plan, [0.0] + [3.0] * (len(plan) - 1))
        if not ok:
            print("   ❌ 移动失败：%s" % msg)
            continue
        time.sleep(0.6)
        print(" ③ 键盘微调：让【指尖正好压在标记中心】上（判不准就换 0.5mm 步长）")
        print("    w/s = x±   a/d = y±   r/f = z±   1/2/3 = 步长 0.5/1/5mm")
        print("    调好后按 Enter 记录；按 q 放弃这一点")
        # 复用 jog.py 的 Jogger：它以「当前位姿 + 增量」算 IK，姿态保持不变，
        # 所以微调过程中工具不会翻腕，指尖方向一直是竖直向下。
        from jog import Jogger, _getch
        jg = Jogger(cfg, kin, arm)
        step = 0.001
        accepted = False
        while True:
            kk = _getch()
            if kk in ("\n", "\r"):
                accepted = True
                break
            if kk in ("q", "Q", "\x03"):
                break
            if kk == "1":
                step = 0.0005
                print("    步长 0.5mm")
                continue
            if kk == "2":
                step = 0.001
                print("    步长 1mm")
                continue
            if kk == "3":
                step = 0.005
                print("    步长 5mm")
                continue
            d = {"w": (step, 0, 0), "s": (-step, 0, 0),
                 "a": (0, step, 0), "d": (0, -step, 0),
                 "r": (0, 0, step), "f": (0, 0, -step)}.get(kk)
            if d is None:
                continue
            okm, msgm = jg.translate(d, max_jump=0.3)
            print("    " + ("✓ " + jg.describe().replace("\n", "  ") if okm
                            else "✗ " + msgm))
        if not accepted:
            print("   已放弃这一点")
            continue
        spin(0.4)
        if state["q"] is None:
            print("   ❌ 关节角丢了，跳过")
            continue
        q2 = state["q"].copy()
        p_true = tip_position(kin, q2, offset)
        err = p_true - p_tcp
        err[2] = 0.0          # 高度由 --gap 决定，不参与水平偏差统计
        print("   指令位置 [%.4f %.4f] / 实测指尖 [%.4f %.4f]"
              % (p_tcp[0], p_tcp[1], p_true[0], p_true[1]))
        print("   偏 差 dx=%+.1fmm dy=%+.1fmm（指尖" % (err[0] * 1000, err[1] * 1000) +
              ("偏 +x/y" if err[0] + err[1] > 0 else "偏 -x/y") + "）")
        results.append({"commanded": p_tcp.tolist(), "actual": p_true.tolist(),
                        "err_xy_m": err[:2].tolist(), "note": note})
        # 撤离
        back = ik_aligned(kin, q2, np.array([p_tcp[0], p_tcp[1], p_tcp[2] + 0.10]),
                          offset, lo, hi)
        if back is not None:
            arm.move_joints([q2, back[0]], [0.0, 3.0])

    _stop_spin.set()
    time.sleep(0.15)
    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 62)
    if not results:
        print(" 没采到有效点，什么都没测成")
        return 1
    E = np.asarray([r["err_xy_m"] for r in results], float) * 1000.0
    mean = E.mean(axis=0)
    dev = E - mean
    spread = float(np.max(np.linalg.norm(dev, axis=1))) if len(E) > 1 else 0.0
    print(" 打点结果（%d 点）" % len(E))
    for i, e in enumerate(E):
        print("   #%d  dx=%+6.1f  dy=%+6.1f mm" % (i + 1, e[0], e[1]))
    print(" 平均偏差  dx=%+.1f  dy=%+.1f mm" % (mean[0], mean[1]))
    print(" 去均值后最大残差 %.1f mm（这就是「补不掉的那部分」）" % spread)
    print("=" * 62)

    if spread < 5.0:
        # err = 真实位置 − 相机算出来的位置；感知发布的是「算出来的」，
        # 所以要加上 err 才对得上真实，故 bias = +mean(err)
        bias = [mean[0] / 1000.0, mean[1] / 1000.0]
        print(" ✅ 结论：偏差基本是【常数】，可以补偿。把下面填进 real_params.yaml：")
        print()
        print("   perception:")
        print("     xy_bias_m: [%.4f, %.4f]" % (bias[0], bias[1]))
        print()
        print("   （符号已算好：填进去以后感知发布的就是修正后的位置）")
        print("   补偿后按 33/34 号预案直接进抓取；若抓到第 3~4 次又出现单向偏移，")
        print("   说明不是常数，回去重标，别继续补。")
    else:
        print(" ❌ 结论：偏差随位置变化（最大 %.1fmm），这是【旋转/尺度】问题，补不了。" % spread)
        print("    按这个顺序查：")
        print("    1) --marker-size 是不是真量过（板尺寸错 → 整体尺度错）")
        print("    2) tcp_offset 是不是 0.177（错 → 偏差随工具姿态变化）")
        print("    3) 采集位姿的旋转激励够不够（handeye_quality.py --focused）")
        print("    4) 都没有 → 重采重标；仍不行 → 走 fixed_target_xyz 兜底")

    out = os.path.expanduser(a.out)
    try:
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"X": X.tolist(), "marker_size": marker_size,
                       "points": results, "mean_mm": mean.tolist(),
                       "spread_mm": spread}, f, indent=2, ensure_ascii=False)
        print("\n 留档：%s" % out)
    except Exception as e:                         # noqa: BLE001
        print("\n （留档失败：%s）" % e)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""collect_handeye.py —— 手眼标定数据采集（Eye-in-Hand，半自动）

★ 默认认 ArUco 标记（2026-11-03 按现场实际标定板改）
------------------------------------------------------
  现场手里是【A4 打印的 ArUco 标记】（两个黑白二维码方块），不是棋盘格。
  旧版只用 findChessboardCorners，明天会一组都检测不到。
  现在默认 --board aruco：自动遍历常见 ArUco 字典、首次命中后锁定，
  用 estimatePoseSingleMarkers 解算 T_cam←marker；可在每行看到重投影误差。
  棋盘格仍可用：--board chessboard。

  ⚠️ --marker-size 必须用尺子量【外框黑边到黑边】的实际边长（米）。
     A4 打印有 0.5~2% 缩放误差，填错会让整个手眼矩阵带固定尺度偏差。
  ⚠️ A4 纸务必贴到硬板上、用胶带/多孔台面的夹具固定，全程不许动。
     纸翘曲或反光会让角点检测不稳定。

两种工作模式（真机 Ubuntu 不适配示教器 → 推荐 ★ 自动扫位）

  ★ 自动扫位（--sweep，2026-11-04 新增，推荐）
    1. 先跑 jog.py（键盘示教）把臂挪到「相机明显对着标定板」的位姿，退出
    2. python3 recovery/collect_handeye.py --sweep --marker-size 0.047 ...
    3. 机械臂自己走 15 个位姿（9 个平移 + 6 个绕工具轴转），逐点检测，
       有标记就采、没有就跳过；最后自动回起始位姿并保存
    全程约 1.5 分钟，不需要人碰机械臂

  手动（--sweep 不加）
    1. 用 jog.py / 程序把机械臂移到一个位姿，确保标记完整出现在彩色图里
    2. 回到终端按回车 → 记录【法兰位姿】+【标定板在相机系的位姿】
    3. 重复 12~15 次（姿态一定要多样！见下方注意）
    4. 按 q 回车结束，自动保存 samples.json

★ 法兰位姿从哪来（2026-11-03 重要修正）
------------------------------------------
  默认 **fk**：用 /joint_states 的当前关节角 + 随包 URDF 做正运动学
      T_base←Link6，和 perception_node.py 走【完全相同】的一套坐标。
      ⚠️ 真机 rm_driver 不发布 TF，旧的纯 tf2 版本会直接报 "TF 读取失败"；
         而且它默认查 base_link→flange，与 URDF 里的 baselink→Link6 也不一致，
         就算有 TF 也会带一个固定偏差。
  可选 **tf**：保留旧的 tf2 lookup_transform 行为（--flange-source tf），
      仅用于确实发布了正确 TF 树的环境（如 Gazebo 仿真）。

前置（真机）：
  - 起相机（提供彩色图 + camera_info）与 rm_driver（提供 /joint_states）
  - 先 `timeout 3 ros2 topic echo /joint_states` 确认有数据
    （没数据 = rm_eco65_config.yaml 的 udp_ip 没填本机 IP）
  - 不需要起 perception_node / grasp_task

用法（现场 A4 ArUco 板）：
  python3 recovery/collect_handeye.py --marker-size 0.05 \
      --out ~/handeye_samples.json

用法（棋盘格板）：
  python3 recovery/collect_handeye.py --board chessboard \
      --cols 9 --rows 6 --square 0.025 --out ~/handeye_samples.json

参数说明：
  --board        : aruco（默认）| chessboard
  --marker-size  : ArUco 外框实测边长（米），默认 0.05，★ 必须实量
  --aruco-dict   : auto（默认，自动遍历并锁定）| DICT_4X4_50 等
  --marker-id    : 只使用指定 id 的标记（多个同时入镜时用）
  --cols/--rows  : 棋盘格【内角点】数，仅 chessboard 模式用
  --square       : 棋盘格方格边长（米），仅 chessboard 模式用
  --camera-prefix: 相机话题前缀，默认从 real_params.yaml 读
  --flange-source: fk（默认，推荐）| tf（旧行为）
  --config       : real_params.yaml 路径（留空自动找）
  --base-frame / --flange-frame : 仅 tf 模式使用

⚠️ 姿态多样性是标定成败的关键：
   必须让相机绕标定板做明显的【旋转】变化（俯仰/偏航都要有），
   只平移不旋转会导致 AX=XB 退化，旋转部分解不出来。
   现场作业区约 1m×1m、方块放机械臂周围近处 → 不必追求大范围/最高处姿态，
   但【标定姿态覆盖的区域必须等于抓取作业区】，否则 DH 误差会让边缘位置偏。

依赖：rclpy, cv2（aruco 模式需 cv2.aruco，云机实测 OpenCV 4.2.0 已带）,
      numpy, sensor_msgs（fk 模式还需要 URDF 与 ik_check.py）
"""

import argparse
import json
import os
import sys
import threading
import time

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo, JointState
except ImportError as e:
    print("缺少依赖: %s" % e)
    print("请先在真机安装: sudo apt install python3-opencv（并 source /opt/ros/foxy/setup.bash）")
    sys.exit(1)

# tf2_ros 只在 --flange-source tf 时才需要；fk 模式（默认）不依赖 TF 树
try:
    from tf2_ros import Buffer, TransformListener
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False

try:
    import cv2
except ImportError:
    print("缺少 cv2，请安装: sudo apt install python3-opencv 或 pip3 install opencv-python")
    sys.exit(1)


def quat_xyzw_to_matrix(x, y, z, w):
    """四元数 (x,y,z,w) -> 3x3 旋转矩阵（不依赖 scipy/tf）"""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat_wxyz(M):
    """3x3 旋转矩阵 -> 四元数 [w,x,y,z]（不依赖 scipy/tf）"""
    tr = M[0, 0] + M[1, 1] + M[2, 2]
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (M[2, 1] - M[1, 2]) / S
        y = (M[0, 2] - M[2, 0]) / S
        z = (M[1, 0] - M[0, 1]) / S
    elif M[0, 0] > M[1, 1] and M[0, 0] > M[2, 2]:
        S = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2.0
        w = (M[2, 1] - M[1, 2]) / S
        x = 0.25 * S
        y = (M[0, 1] + M[1, 0]) / S
        z = (M[0, 2] + M[2, 0]) / S
    elif M[1, 1] > M[2, 2]:
        S = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2.0
        w = (M[0, 2] - M[2, 0]) / S
        x = (M[0, 1] + M[1, 0]) / S
        y = 0.25 * S
        z = (M[1, 2] + M[2, 1]) / S
    else:
        S = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2.0
        w = (M[1, 0] - M[0, 1]) / S
        x = (M[0, 2] + M[2, 0]) / S
        y = (M[1, 2] + M[2, 1]) / S
        z = 0.25 * S
    return [float(w), float(x), float(y), float(z)]


def quat_wxyz_from_matrix(m4):
    return matrix_to_quat_wxyz(m4[:3, :3])


class Collector(Node):
    def __init__(self, base_frame, flange_frame, cols, rows, square,
                 cam_prefix='/camera/d435i', flange_source='fk',
                 kin=None, joint_names=None, joint_topic='/joint_states',
                 board='aruco', marker_size=0.05, aruco_dict='auto',
                 marker_id=None):
        super().__init__('collect_handeye')
        self.base_frame = base_frame
        self.flange_frame = flange_frame
        self.cols, self.rows, self.square = cols, rows, square
        # 话题前缀可配：由 real_params.yaml 的 camera.prefix 传入，
        # 与 d435i_eco65.launch.py 自动反推的 namespace 保持一致
        self.cam_prefix = cam_prefix.rstrip('/')
        self.flange_source = flange_source
        self.board = board                       # 'aruco' | 'chessboard'
        self.marker_size = float(marker_size)    # ArUco 外框边长（米），必须实量
        self.aruco_dict_name = aruco_dict
        self.marker_id = marker_id
        self._dicts = None
        self._locked_dict = None                 # 首次检测到后锁定，避免每帧遍历
        self._locked_id = None                   # ★ 首次检测到后锁定同一个 marker id
        self.kin = kin
        self.joint_names = list(joint_names or
                                ['joint1', 'joint2', 'joint3',
                                 'joint4', 'joint5', 'joint6'])
        self.q = None

        if flange_source == 'fk':
            # 真机 rm_driver 不发 TF → 用 /joint_states + URDF FK 算 T_base←Link6。
            # 与 perception_node.py 完全同一套坐标，不会带 frame 名不一致的固定偏差。
            if kin is None:
                raise RuntimeError('fk 模式需要 UrdfKinematics，检查 URDF/ik_check.py 是否可用')
            self.tf_buffer = None
            self.tf_listener = None
            self.create_subscription(JointState, joint_topic, self.on_js, 10)
        else:
            if not _HAS_TF2:
                raise RuntimeError('tf 模式需要 tf2_ros：sudo apt install ros-foxy-tf2-ros')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.img = None
        self.K = None
        self.D = None
        self.create_subscription(Image, f'{self.cam_prefix}/color/image_raw', self.on_img, 10)
        self.create_subscription(CameraInfo, f'{self.cam_prefix}/color/camera_info', self.on_info, 10)

        self.samples = []
        self._stop = False
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in self.joint_names):
            self.q = np.array([float(d[n]) for n in self.joint_names])

    def on_img(self, m):
        h, w = m.height, m.width
        buf = np.frombuffer(m.data, dtype=np.uint8)
        enc = (m.encoding or '').lower()
        if len(buf) == h * w * 3:
            img = buf.reshape(h, w, 3)
        elif len(buf) == h * w * 4:
            img = buf.reshape(h, w, 4)[:, :, :3]
        else:
            return
        # 统一存成 BGR 供 cv2 使用（ROS 常见 rgb8；bgr8 则不用翻）
        self.img = img.copy() if 'bgr' in enc else img[:, :, ::-1].copy()

    def on_info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64) if m.d else None

    # ------------------------------------------------------------ 板检测
    def _all_dicts(self):
        """返回 [(名字, 字典对象)]；auto 模式下遍历常见 ArUco 字典。"""
        import cv2.aruco as A
        names = ['DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250',
                 'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250',
                 'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250',
                 'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250',
                 'DICT_ARUCO_ORIGINAL']
        out = []
        want = self.aruco_dict_name
        for n in names:
            if want != 'auto' and want.upper() != n:
                continue
            v = getattr(A, n, None)
            if v is None:
                continue
            try:
                d = (A.Dictionary_get(v) if hasattr(A, 'Dictionary_get')
                     else A.getPredefinedDictionary(v))
                out.append((n, d))
            except Exception:                      # noqa: BLE001
                pass
        return out

    def detect(self):
        """返回 (marker_in_camera 4x4 矩阵, 说明)，失败返回 (None, 原因)"""
        if self.img is None:
            return None, "没有收到图像"
        if self.K is None:
            return None, "没有收到相机内参"
        if self.board == 'aruco':
            return self._detect_aruco()
        return self._detect_chessboard()

    def _detect_aruco(self):
        try:
            import cv2.aruco as A
        except ImportError:
            return None, ("cv2.aruco 不可用（缺 opencv-contrib）。"
                          "在真机跑：pip3 install --user opencv-contrib-python==4.2.0.34，"
                          "或改用 --board chessboard")
        if self._dicts is None:
            self._dicts = self._all_dicts()
            if not self._dicts:
                return None, "没有可用的 ArUco 字典（检查 --aruco-dict 名字）"
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        dist = self.D if (self.D is not None and len(self.D) > 0) else None
        candidates = ([self._locked_dict] if self._locked_dict else self._dicts)
        for name, dic in candidates:
            try:
                corners, ids, _ = A.detectMarkers(gray, dic)
            except Exception:                      # noqa: BLE001
                continue
            if ids is None or len(ids) == 0:
                continue
            # ★ 关键：多标记纸必须【每一帧都用同一个物理标记】，否则各组算的是
            #   不同靶标的位置，AX=XB 直接错（而且残差可能还很小，极难发现）。
            #   规则：--marker-id 显式指定 > 首次检测自动锁定 > 都没有才用第一个
            want = self.marker_id if self.marker_id is not None else self._locked_id
            if want is not None:
                hits = [i for i, v in enumerate(ids.ravel()) if int(v) == int(want)]
                if not hits:
                    return None, ("锁定的 marker id=%d 本帧不可见（换回能看到它的位姿，"
                                  "或用 --marker-id 指定另一个）" % int(want))
                pick = hits[0]
            else:
                pick = 0
                self._locked_id = int(ids.ravel()[pick])
                self.get_logger().info(
                    "已锁定 marker id=%d（之后所有样本都用它；id=%s 不再使用）"
                    % (self._locked_id, sorted(set(int(v) for v in ids.ravel()))[:6]))
            c = corners[pick]
            try:
                res = A.estimatePoseSingleMarkers(
                    [c], self.marker_size, self.K, dist)
                rvecs, tvecs = res[0], res[1]
            except Exception as e:                 # noqa: BLE001
                return None, "estimatePoseSingleMarkers 失败：%s" % e
            rvec = rvecs[0]
            tvec = tvecs[0]
            Rm, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = Rm
            T[:3, 3] = tvec.ravel()
            # 逐帧重投影误差（角点质量把关，>2px 提示别采这组）
            s = self.marker_size / 2.0
            obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                           dtype=np.float32)
            proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, dist)
            err = float(np.mean(np.linalg.norm(
                proj.reshape(-1, 2) - c.reshape(-1, 2), axis=1)))
            if self._locked_dict is None:
                self._locked_dict = (name, dic)
                self.get_logger().info("已锁定 ArUco 字典 = %s" % name)
            note = "OK  dict=%s id=%d  重投影 %.2fpx%s" % (
                name, int(ids.ravel()[pick]), err,
                "  ⚠️ 偏大，这组建议重拍" if err > 2.0 else "")
            if err > 2.0:
                # 仍返回，但把警告带进说明，由 grab 打印给用户
                pass
            return T, note
        return None, ("没检测到 ArUco 标记（检查：完整入镜 / 光照均匀 / A4 纸贴平不翘 / "
                      "marker 边长 --marker-size 是否实量 / 字典 --aruco-dict）")

    def _detect_chessboard(self):
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            return None, "没找到棋盘格（检查：完整入镜 / 反光 / 行列数 / 纸贴平）"
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        objp = np.zeros((self.cols * self.rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square
        dist = self.D if (self.D is not None and len(self.D) > 0) else None
        ok, rvec, tvec = cv2.solvePnP(objp, corners, self.K, dist)
        if not ok:
            return None, "solvePnP 失败"
        proj, _ = cv2.projectPoints(objp, rvec, tvec, self.K, dist)
        err = float(np.mean(np.linalg.norm(
            proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)))
        Rm, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = tvec.ravel()
        return T, "OK  重投影 %.2fpx%s" % (err, "  ⚠️ 偏大，这组建议重拍" if err > 2.0 else "")

    def grab(self):
        """采集一组，返回 (dict, msg)"""
        # 1) 法兰位姿 T_base←Link6
        if self.flange_source == 'fk':
            if self.q is None:
                return None, ("收不到 /joint_states —— 确认 rm_driver 已启动，"
                              "且 rm_eco65_config.yaml 的 udp_ip 填的是本机 IP")
            # 每次现算，禁止缓存：机械臂一动这个矩阵就变
            Fb = np.asarray(self.kin.fk(self.q), dtype=float)
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.flange_frame, rclpy.time.Time())
            except Exception as e:                 # noqa: BLE001
                return None, "TF 读取失败 (%s -> %s): %s" % (
                    self.base_frame, self.flange_frame, e)
            t = tf.transform.translation
            q = tf.transform.rotation          # ROS 约定：xyzw
            Fb = np.eye(4)
            Fb[:3, :3] = quat_xyzw_to_matrix(q.x, q.y, q.z, q.w)
            Fb[:3, 3] = [t.x, t.y, t.z]

        # 2) 标定板
        Tc, msg = self.detect()
        if Tc is None:
            return None, msg

        return {
            "flange_in_base": {
                "xyz": [float(v) for v in Fb[:3, 3]],
                "quat_wxyz": quat_wxyz_from_matrix(Fb),
            },
            "marker_in_camera": {
                "xyz": [float(v) for v in Tc[:3, 3]],
                "quat_wxyz": quat_wxyz_from_matrix(Tc),
            },
        }, msg


def _rot_axis(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _sweep_target(kin, lo, hi, q0, offset, dxyz=(0, 0, 0), axis=None, deg=0.0):
    """以 q0 为基准，指尖平移 dxyz / 绕工具轴转 (axis, deg)，返回 IK 解或 None。

    ★ 为什么"绕工具轴转"要保留指尖位置：相机装在腕上，绕指尖转 = 相机绕着
      标定板转，标定板始终在视野里，而法兰姿态明显变化 —— 这正是 AX=XB 需要的
      旋转激励。若绕法兰转，标定板会飞出画面。
    """
    from ik_check import dls_ik
    T = np.asarray(kin.fk(np.asarray(q0, float)), float)
    R, p = T[:3, :3], T[:3, 3]
    tip = R @ np.asarray(offset, float) + p
    R2 = R if axis is None else R @ _rot_axis(axis, deg)
    tip2 = tip + np.asarray(dxyz, float)
    q, ok, err = dls_ik(kin, np.asarray(q0, float), tip2 - R2 @ np.asarray(offset, float),
                        R2, max_iter=400, tol=1e-4, lam=1e-3, max_step=0.12)
    if not ok or err > 5e-3:
        return None, "IK 无解 err=%.4f" % err
    if np.any(q < lo) or np.any(q > hi):
        return None, "超关节限位"
    if float(np.max(np.abs(q - np.asarray(q0, float)))) > 0.75:
        return None, "跳变过大"
    return q, "OK"


def run_sweep(c, a, cfg):
    """自动扫位采集：机械臂自己走一圈预设位姿，每个位姿试一次检测。

    取代原来的"人用示教器摆 12 次"（用户 2026-11-04：Ubuntu 不适配示教器）。
    """
    from arm_backend import make_backend
    arm_cfg = dict(cfg["arm"])
    arm_cfg["joints"] = list(cfg["robot"]["joints"])
    arm_cfg["speed"] = max(1, min(int(arm_cfg.get("speed", 20)), 10))
    arm = make_backend(c, arm_cfg, dry_run=False)

    q0 = c.q.copy() if c.q is not None else None
    if q0 is None:
        print("❌ 收不到 /joint_states：先跑 net_autoconf.py")
        return 3

    offset = np.asarray(cfg["robot"]["tcp_offset"], float)
    lo = np.asarray(cfg["robot"]["joint_limits_lo"], float)
    hi = np.asarray(cfg["robot"]["joint_limits_hi"], float)
    sp, tl = float(a.sweep_span), float(a.sweep_tilt)

    # 15 个候选：9 个平移 + 6 个绕工具轴转（相机绕标记转，视野不丢）
    cand = []
    for d in ((0, 0, 0), (sp, 0, 0), (-sp, 0, 0), (0, sp, 0), (0, -sp, 0),
              (0.75 * sp, 0.75 * sp, 0), (-0.75 * sp, -0.75 * sp, 0),
              (0, 0, 0.05), (0, 0, -0.05)):
        cand.append(("平移 dx=%+.0f dy=%+.0f dz=%+.0f mm" % tuple(v * 1000 for v in d),
                     d, None, 0.0))
    for ax, sg in (("x", 1), ("x", -1), ("y", 1), ("y", -1), ("z", 1), ("z", -1)):
        dd = tl * 2 if ax == "z" else tl
        cand.append(("绕工具%s轴 %+.0f°" % (ax, sg * dd), (0, 0, 0), ax, sg * dd))

    print("\n" + "=" * 56)
    print(" 自动扫位采集：%d 个候选位姿，机械臂自己走" % len(cand))
    print(" 起始位姿（务必确认此时相机能看见标定板）：")
    print("   关节 %s" % np.round(q0, 3).tolist())
    print(" 速度 %d，单点约 4 秒，全程约 %.1f 分钟"
          % (arm_cfg["speed"], len(cand) * 4.5 / 60))
    print("=" * 56)
    try:
        input(" 工作区清空、急停在手边 → 回车开始（Ctrl+C 中止）> ")
    except (EOFError, KeyboardInterrupt):
        return 4

    ok_n = 0
    for i, (label, d, ax, dg) in enumerate(cand):
        q, why = _sweep_target(c.kin, lo, hi, q0, offset, d, ax, dg)
        if q is None:
            print("  [%02d/%d] %-28s 跳过：%s" % (i + 1, len(cand), label, why))
            continue
        ok, msg = arm.move_joints([c.q if c.q is not None else q0, q], [0.0, 3.0])
        if not ok:
            print("  [%02d/%d] %-28s 移动失败：%s" % (i + 1, len(cand), label, msg))
            continue
        time.sleep(0.8)
        s, note = c.grab()
        if s is None:
            time.sleep(0.5)
            s, note = c.grab()
        if s is None:
            print("  [%02d/%d] %-28s 无标记：%s" % (i + 1, len(cand), label, note))
            continue
        c.samples.append(s)
        ok_n += 1
        print("  [%02d/%d] %-28s ✅ 第 %d 组 | %s"
              % (i + 1, len(cand), label, ok_n, note))

    # 回起始位姿，方便人看一眼
    try:
        arm.move_joints([c.q, q0], [0.0, 4.0])
    except Exception:                              # noqa: BLE001
        pass

    print("\n 扫位结束：采到 %d 组（候选 %d 个）" % (ok_n, len(cand)))
    if ok_n < 5:
        print(" ❌ 太少。大概率是起始位姿就看不到标记 —— 先跑 jog.py 把臂挪到"
              "「相机明显对着标定板」的位置，再重来。")
    return save_samples(c, a, cfg)


def save_samples(c, a, cfg):
    """落盘 samples.json（交互模式与扫位模式共用）"""
    if len(c.samples) < 3:
        print("\n❌ 只采到 %d 组，至少需要 3 组（推荐 12~15 组），未保存"
              % len(c.samples))
        return 1
    if a.flange_source == 'fk' and cfg is not None:
        real_base = cfg['robot']['base_frame']      # baselink
        real_tip = cfg['robot']['tip_frame']        # Link6
    else:
        real_base, real_tip = a.base_frame, a.flange_frame
    board_desc = ("棋盘格 %dx%d 内角点，方格 %.4f m"
                  % (a.cols, a.rows, a.square)) if a.board == 'chessboard' else \
                 ("ArUco 标记，字典=%s，实量边长 %.4f m，marker_id=%s"
                  % (c._locked_dict[0] if c._locked_dict else a.aruco_dict,
                     a.marker_size, a.marker_id))
    out = {
        "_readme": ("手眼标定数据（Eye-in-Hand）。%s。"
                    "法兰位姿来源=%s；frame 必须与 FK/感知节点一致。"
                    % (board_desc, a.flange_source)),
        "mode": "eye_in_hand",
        "flange_source": a.flange_source,
        "board": a.board,
        "board_desc": board_desc,
        "frames": {
            "robot_base_frame": real_base,
            "flange_frame": real_tip,
            "camera_optical_frame": "d435i_color_optical_frame",
            "marker_frame": "aruco_on_table" if a.board == 'aruco' else "chessboard_on_table",
        },
        "samples": c.samples,
    }
    path = os.path.expanduser(a.out)
    with open(path, 'w') as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    print("\n✅ 已保存 %d 组到 %s" % (len(c.samples), path))
    print("下一步：python3 handeye_calib.py --input %s --pairs all --write-config" % path)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cols', type=int, default=9)
    ap.add_argument('--rows', type=int, default=6)
    ap.add_argument('--square', type=float, default=0.025)
    ap.add_argument('--base-frame', default='base_link')
    ap.add_argument('--flange-frame', default='flange')
    ap.add_argument('--out', default='~/handeye_samples.json')
    ap.add_argument('--camera-prefix', default=None,
                    help='相机话题前缀。留空=从 real_params.yaml 的 camera.prefix 读'
                         '（launch 会自动反推 namespace，两者永远一致）')
    ap.add_argument('--flange-source', choices=['fk', 'tf'], default='fk',
                    help='法兰位姿来源：fk=用 /joint_states + URDF 正运动学（默认，真机推荐）；'
                         'tf=用 tf2 查 TF 树（旧行为，真机 rm_driver 不发 TF 会失败）')
    ap.add_argument('--config', default=None, help='real_params.yaml 路径（留空自动找）')
    ap.add_argument('--board', choices=['aruco', 'chessboard'], default='aruco',
                    help='标定板类型。默认 aruco（现场手里是 A4 打印的 ArUco 标记）；'
                         '若换成棋盘格用 chessboard')
    ap.add_argument('--marker-size', type=float, default=None,
                    help='ArUco 标记【外框黑边到黑边】的实测边长（米）。'
                         '不传就读 real_params.yaml 的 handeye.marker_size_m；'
                         '都没有才用 0.05。必须用尺子量 A4 打印件（打印有 0.5~2%% 缩放误差）')
    ap.add_argument('--aruco-dict', default='auto',
                    help='ArUco 字典，默认 auto 自动遍历常见字典并锁定；'
                         '也可显式指定如 DICT_4X4_50 / DICT_6X6_250')
    ap.add_argument('--sweep', action='store_true',
                    help='★ 自动扫位：机械臂自己走 15 个预设位姿并逐个检测'
                         '（不需要示教器；会把臂挪到起始位姿后自动开始）')
    ap.add_argument('--sweep-span', type=float, default=0.04,
                    help='扫位的平移幅度（米），默认 0.04')
    ap.add_argument('--sweep-tilt', type=float, default=12.0,
                    help='扫位的绕轴转角（度），默认 12')
    ap.add_argument('--marker-id', type=int, default=None,
                    help='只使用指定 id 的 ArUco 标记（多个标记同时入镜时避免选错）')
    a = ap.parse_args()

    # ---- 从项目配置取 URDF / 关节名 / 话题前缀，保证与感知节点同一套坐标 ----
    cfg, kin = None, None
    cam_prefix = a.camera_prefix
    joint_names = None
    joint_topic = '/joint_states'
    ws = os.environ.get('ROBOT_WS', os.path.expanduser('~/robot_vla_ws'))
    scripts = os.path.join(ws, 'robot_code', 'arm_grasp', 'scripts')
    if os.path.isdir(scripts) and scripts not in sys.path:
        sys.path.insert(0, scripts)
    try:
        from common import load_config, setup_import_paths, find_urdf
        cfg = load_config(a.config)
        setup_import_paths(cfg)
        if a.flange_source == 'fk':
            from ik_check import UrdfKinematics
            kin = UrdfKinematics(find_urdf(cfg), cfg['robot']['base_frame'],
                                 cfg['robot']['tip_frame'])
            joint_names = cfg['robot']['joints']
            joint_topic = cfg['arm'].get('joint_states_topic', '/joint_states')
        if cam_prefix is None:
            cam_prefix = cfg['camera']['prefix']
    except Exception as e:                         # noqa: BLE001
        if a.flange_source == 'fk':
            print("❌ fk 模式加载项目配置/URDF 失败：%s" % e)
            print("   确认 %s 存在，或用 --config <real_params.yaml> 指定；" % ws)
            print("   也可用 --flange-source tf 走旧版（需要 TF 树完整）。")
            return
        print("⚠️ 读不到 real_params.yaml（%s），camera-prefix 用默认值" % e)
    if cam_prefix is None:
        cam_prefix = '/camera/d435i'
    # marker 尺寸：命令行 > 配置 handeye.marker_size_m > 0.05
    if a.marker_size is None:
        try:
            a.marker_size = float((cfg or {}).get('handeye', {}).get('marker_size_m') or 0.05)
            print(" 标记边长取自配置 handeye.marker_size_m = %.4f m" % a.marker_size)
        except Exception:                          # noqa: BLE001
            a.marker_size = 0.05

    rclpy.init()
    c = Collector(a.base_frame, a.flange_frame, a.cols, a.rows, a.square,
                  cam_prefix, a.flange_source, kin, joint_names, joint_topic,
                  a.board, a.marker_size, a.aruco_dict, a.marker_id)
    time.sleep(3)

    print()
    print("=" * 56)
    print(" 手眼标定数据采集（Eye-in-Hand）")
    print("=" * 56)
    if a.board == 'aruco':
        print(" 标定板: ArUco  字典=%s  标记边长=%.4f m（★ 必须实量）"
              % (a.aruco_dict, a.marker_size))
    else:
        print(" 标定板: 棋盘格  内角点 %d x %d  方格边长 %.4f m"
              % (a.cols, a.rows, a.square))
    print(" 相机话题前缀: %s" % cam_prefix)
    if a.flange_source == 'fk':
        print(" 法兰位姿: FK(/joint_states → URDF baselink→Link6)  话题 %s" % joint_topic)
        print(" ⚠️  先确认关节角在刷：timeout 3 ros2 topic echo /joint_states")
    else:
        print(" 法兰位姿: TF %s -> %s" % (a.base_frame, a.flange_frame))
    print()
    board_word = "ArUco 标记" if a.board == 'aruco' else "棋盘格"
    if a.sweep:
        print(" 模式：自动扫位（机械臂会自己动，约 1.5 分钟）")
    else:
        print(" 操作：把机械臂移到一个位姿（%s完整入镜）→ 回这里按回车" % board_word)
        print("       输入 q 回车 = 结束并保存")
        print("       ★ 没有示教器？先跑 jog.py 用键盘摆位，再回来回车")
    print()
    print(" ⚠️  位姿之间必须有明显的【旋转】差异，不能只平移！")
    print(" ⚠️  每组【停稳后】再按回车；A4 纸务必贴平固定，全程不许动")
    print(" ⚠️  每行会打印重投影误差，>2px 的组建议换个位姿重拍")
    print("=" * 56)
    print()

    if a.sweep:
        rc = run_sweep(c, a, cfg)
        c.destroy_node()
        rclpy.shutdown()
        return rc

    while True:
        try:
            cmd = input("已采集 %d 组，按回车采集下一组 (q=结束): " % len(c.samples)).strip()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd.lower() == 'q':
            break
        s, msg = c.grab()
        if s is None:
            print("   ❌ %s —— 换个位姿再来" % msg)
            continue
        c.samples.append(s)
        fb = s['flange_in_base']['xyz']
        mc = s['marker_in_camera']['xyz']
        print("   ✅ 第 %d 组  flange_xyz=[%+.3f %+.3f %+.3f]  marker_xyz=[%+.3f %+.3f %+.3f]  | %s"
              % (len(c.samples), fb[0], fb[1], fb[2], mc[0], mc[1], mc[2], msg))

    rc = save_samples(c, a, cfg)
    c.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())

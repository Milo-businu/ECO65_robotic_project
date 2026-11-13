#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂硬件抽象层 —— 把「仿真」和「真机」的接口差异关在这一层里

为什么需要它（移植最大的断点）
--------------------------------
  仿真：Gazebo + ros2_control → FollowJointTrajectory action
        /rm_group_controller/follow_joint_trajectory

  真机：rm_driver 是【话题式】接口
        /rm_driver/movej_cmd   (rm_ros_interfaces/msg/Movej)
        /rm_driver/movej_result (std_msgs/Bool)

  睿尔曼另给了 rm_control 包把 action 桥接成 CANFD 透传，但它的 C++ 实现有两个硬要求：
    ① goal->trajectory.points[i].velocities[0] / accelerations[0] 必须非空
       （仿真 eff_test.py 只填 positions → 真机直接越界崩溃）
    ② 点数 > 3 才做三次样条插值
  所以本文件的 ActionBackend 会补齐速度/加速度、并按需补足点数。

  ⚠️ 默认 backend = movej（带控制器规划、可 block、最安全）。

关节角反馈
----------
  感知算 FK 必须用【当前】关节角，禁止缓存旧位姿。
  真机 /joint_states 依赖 rm_driver 的 UDP 主动上报：
  config 里 udp_ip 必须是真机 PC 的实际 IP，否则话题永远没数据。
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Optional, List, Sequence

import numpy as np


# ---------------------------------------------------------------- 工具
def _qos_parameters():
    """对应 C++ 的 rclcpp::ParametersQoS()；老版本 rclpy 没有就退回默认。"""
    try:
        from rclpy.qos import qos_profile_parameters
        return qos_profile_parameters
    except ImportError:
        from rclpy.qos import QoSProfile
        return QoSProfile(depth=10)


def _speed(cfg, node=None):
    """取速度百分比，并强制不超过 speed_hard_max（默认 20）。

    为什么要这道闸：rm_driver 的 speed 是百分比，手滑写成 100 就是满速，
    而现场是有线缆限制的 —— 宁可慢，不可快。
    """
    try:
        s = int(cfg.get("speed", 20))
    except Exception:                              # noqa: BLE001
        s = 20
    try:
        cap = int(cfg.get("speed_hard_max", 20))
    except Exception:                              # noqa: BLE001
        cap = 20
    if s > cap:
        if node is not None:
            try:
                node.get_logger().warn(
                    "arm.speed=%d 超过硬上限 %d → 按 %d 执行（改配置请连 speed_hard_max 一起）"
                    % (s, cap, cap))
            except Exception:                      # noqa: BLE001
                pass
        s = cap
    return max(1, s)


class _EventWaiter:
    """线程间等待一次性结果（ROS 回调在 spin 线程里跑）。"""

    def __init__(self):
        self.ev = threading.Event()
        self.value = None

    def set(self, v):
        self.value = v
        self.ev.set()

    def wait(self, timeout):
        if not self.ev.wait(timeout):
            return False, "超时"
        return True, self.value


# ---------------------------------------------------------------- 后端基类
class ArmBackend:
    """统一接口：move_joints(路径点) / current_joints() / stop()"""

    name = "base"

    def __init__(self, node, cfg: dict, dry_run: bool = False):
        self.node = node
        self.cfg = cfg
        self.dry_run = dry_run
        self._last_target: Optional[np.ndarray] = None

    def move_joints(self, waypoints: Sequence[Sequence[float]],
                    times: Optional[Sequence[float]] = None) -> tuple:
        raise NotImplementedError

    def current_joints(self) -> Optional[np.ndarray]:
        raise NotImplementedError

    def stop(self) -> None:
        pass

    def _remember(self, q):
        self._last_target = np.asarray(q, dtype=float).copy()


# ---------------------------------------------------------------- 演练后端
class DryRunBackend(ArmBackend):
    """不接任何硬件，只打印。用于在真机接电之前把整条状态机跑一遍。"""

    name = "dry-run"

    def move_joints(self, waypoints, times=None):
        for i, q in enumerate(waypoints):
            self.node.get_logger().info(
                "[DRY-RUN] 路径点 %d/%d  %s" %
                (i + 1, len(waypoints), np.round(np.asarray(q, float), 4).tolist()))
        self._remember(waypoints[-1])
        return True, "演练完成（未下发）"

    def current_joints(self):
        return self._last_target.copy() if self._last_target is not None else None


# ---------------------------------------------------------------- 真机 movej
class MoveJBackend(ArmBackend):
    """逐点下发 /rm_driver/movej_cmd，等 /rm_driver/movej_result。

    Movej 字段：float32[] joint, uint8 speed, bool block,
                uint8 trajectory_connect, uint8 dof
    """

    name = "movej"

    def __init__(self, node, cfg, dry_run=False):
        super().__init__(node, cfg, dry_run)
        from std_msgs.msg import Bool, Empty
        from rm_ros_interfaces.msg import Movej
        self._Movej = Movej
        self._Empty = Empty
        qos = _qos_parameters()
        self.pub = node.create_publisher(Movej, cfg["movej_cmd_topic"], qos)
        self.stop_pub = node.create_publisher(Empty, cfg.get("stop_cmd_topic",
                                                             "/rm_driver/move_stop_cmd"), 10)
        self.waiter = _EventWaiter()
        node.create_subscription(Bool, cfg["movej_result_topic"],
                                 lambda m: self.waiter.set(bool(m.data)), qos)
        self._joint_source = make_joint_source(node, cfg, self)
        self._timeout = float(cfg.get("move_timeout_s", 25.0))
        self.speed = _speed(cfg, node)

    def move_joints(self, waypoints, times=None):
        # 逐点执行：每点单独规划，靠密集路径点逼近笛卡尔直线
        import time as _t
        for i, q in enumerate(waypoints):
            _t0 = _t.time()
            m = self._Movej()
            m.joint = [float(x) for x in q]
            m.speed = int(self.speed)
            m.block = bool(self.cfg.get("block", True))
            m.trajectory_connect = int(self.cfg.get("trajectory_connect", 0))
            m.dof = int(self.cfg.get("dof", 6))
            self.waiter = _EventWaiter()
            self.pub.publish(m)
            ok, val = self.waiter.wait(self._timeout)
            if not ok:
                return False, "第 %d 点等待 movej_result 超时（检查机械臂是否使能/急停）" % (i + 1)
            if val is False:
                return False, "第 %d 点控制器返回失败（可能超出限位/奇异点）" % (i + 1)
            _dt = _t.time() - _t0
            if _dt > 5.0:      # 慢动作打出来，方便判断是不是速度设太低
                try:
                    self.node.get_logger().info(
                        "   [movej] 第 %d 点用了 %.1fs（speed=%d%%）" % (i + 1, _dt, self.speed))
                except Exception:                  # noqa: BLE001
                    pass
            self._remember(q)
        return True, "OK"

    def current_joints(self):
        return self._joint_source.current()

    def stop(self):
        self.stop_pub.publish(self._Empty())
        self.node.get_logger().warn("已发送 /rm_driver/move_stop_cmd（轨迹急停）")


# ---------------------------------------------------------------- action 后端
class ActionBackend(ArmBackend):
    """FollowJointTrajectory。仿真直接用；真机经 rm_control 桥接。

    ⚠️ 针对 rm_control 的两处适配（不改这里真机会崩）：
       - 每个点补 velocities / accelerations（C++ 直接索引 [0]）
       - 点数 ≤3 时线性补到 4 点以上（C++ 才走三次样条插值分支）
    """

    name = "action"

    def __init__(self, node, cfg, dry_run=False):
        super().__init__(node, cfg, dry_run)
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        self._FJT = FollowJointTrajectory
        self._JTP = JointTrajectoryPoint
        self.client = ActionClient(node, FollowJointTrajectory,
                                   cfg["action_name"])
        self.joint_names = list(cfg.get("joints",
                                        ["joint1", "joint2", "joint3",
                                         "joint4", "joint5", "joint6"]))
        self._joint_source = make_joint_source(node, cfg, self)

    def _pad_points(self, waypoints, times):
        """补点 + 填速度/加速度，返回 (wps, ts)"""
        wps = [np.asarray(q, float) for q in waypoints]
        if times is None:
            ts = [float(2 * i) for i in range(len(wps))]
        else:
            ts = [float(t) for t in times]
        # 补到至少 4 点（rm_control 的样条分支要求 point_num > 3）
        while len(wps) < 4:
            wps.append(wps[-1].copy())
            ts.append(ts[-1] + 1.0)
        return wps, ts

    def move_joints(self, waypoints, times=None):
        wps, ts = self._pad_points(waypoints, times)
        if not self.client.wait_for_server(timeout_sec=15.0):
            return False, "action server 未就绪：%s" % self.cfg["action_name"]
        goal = self._FJT.Goal()
        goal.trajectory.joint_names = self.joint_names
        for q, t in zip(wps, ts):
            p = self._JTP()
            p.positions = [float(x) for x in q]
            p.velocities = [0.0] * 6          # ⚠️ rm_control 会直接索引，不能省
            p.accelerations = [0.0] * 6       # ⚠️ 同上
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)
        waiter = _EventWaiter()
        holder = {}

        def _done(fut):
            waiter.set(fut.result())

        self.client.send_goal_async(goal).add_done_callback(
            lambda g: (g.result().get_result_async().add_done_callback(_done)
                       if g.result().accepted
                       else (holder.__setitem__("rejected", True), waiter.set(None))))
        ok, _ = waiter.wait(timeout=max(ts) + 40)
        if not ok:
            return False, "action 超时"
        if holder.get("rejected"):
            return False, "action 被拒"
        self._remember(wps[-1])
        return True, "OK"

    def current_joints(self):
        return self._joint_source.current()

    def stop(self):
        from std_msgs.msg import Empty
        if not hasattr(self, "stop_pub"):
            self.stop_pub = self.node.create_publisher(
                Empty, self.cfg.get("stop_cmd_topic", "/rm_driver/move_stop_cmd"), 10)
        self.stop_pub.publish(Empty())


# ---------------------------------------------------------------- 关节角来源
class JointStateSource:
    """订阅 /joint_states（rm_driver UDP 主动上报）

    ★ 带新鲜度检查：话题"停更"时返回 None，而不是把最后一帧当当前值。
      因为感知/规划都靠 FK(q)，用旧 q 算出来的坐标会说谎——而且看起来一切正常。
    """

    def __init__(self, node, cfg, max_age: float = 3.0):
        from sensor_msgs.msg import JointState
        self.names = list(cfg["joints"])
        self.q = None
        self.t = 0.0
        self.max_age = float(max_age)
        self._warned = 0.0
        self.node = node
        node.create_subscription(JointState, cfg["joint_states_topic"],
                                 self._on_js, 10)

    def _on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in self.names):
            self.q = np.array([d[n] for n in self.names], dtype=float)
            self.t = time.time()

    def current(self):
        if self.q is None:
            return None
        age = time.time() - self.t
        if age > self.max_age:
            now = time.time()
            if now - self._warned > 5.0:
                self._warned = now
                try:
                    self.node.get_logger().warn(
                        "/joint_states 已经 %.1fs 没更新（>%.0fs）→ 不返回关节角。"
                        "查 rm_driver 是否还活着 / udp_ip 是否变过" % (age, self.max_age))
                except Exception:                  # noqa: BLE001
                    pass
            return None
        return self.q.copy()


class ArmStateSource:
    """请求式查询：发 Empty 到 cmd，读 result。

    不同固件版本的 result 消息字段可能是 joint / joints / joint_position，
    这里自适应解析，避免因为字段名猜错而静默失败。
    """

    def __init__(self, node, cfg):
        from std_msgs.msg import Empty
        self.node = node
        self.names = list(cfg["joints"])
        self.q = None
        self.waiter = _EventWaiter()
        self.cmd_pub = node.create_publisher(Empty, cfg["arm_state_cmd_topic"], 10)
        # result 的具体类型不强依赖，用 AnyMsg 不行，这里尝试导入，失败则报错提示
        try:
            from rm_ros_interfaces.msg import Armcurrentstatus
            node.create_subscription(Armcurrentstatus,
                                     cfg["arm_state_result_topic"],
                                     self._on_state, 10)
        except ImportError:
            node.get_logger().warn(
                "找不到 Armcurrentstatus 消息类型，arm_state 关节源不可用；"
                "请改用 joint_source: joint_states")

    def _on_state(self, m):
        for attr in ("joint", "joints", "joint_position", "joint_state"):
            v = getattr(m, attr, None)
            if v is not None and len(v) >= 6:
                self.q = np.array([float(x) for x in v[:6]])
                self.waiter.set(True)
                return

    def current(self):
        self.waiter = _EventWaiter()
        from std_msgs.msg import Empty
        self.cmd_pub.publish(Empty())
        ok, _ = self.waiter.wait(3.0)
        return self.q.copy() if ok and self.q is not None else None


class OpenLoopSource:
    """用「刚下发的目标角」当反馈。⚠️ 仅演练/调试，真机抓取禁止使用。"""

    def __init__(self, backend):
        self.backend = backend

    def current(self):
        q = getattr(self.backend, "_last_target", None)
        return q.copy() if q is not None else None


def make_joint_source(node, cfg, backend):
    which = cfg.get("joint_source", "joint_states")
    if which == "joint_states":
        return JointStateSource(node, cfg)
    if which == "arm_state":
        return ArmStateSource(node, cfg)
    if which == "open_loop":
        node.get_logger().warn("joint_source=open_loop：开环，禁止用于真机抓取")
        return OpenLoopSource(backend)
    raise ValueError("未知 joint_source: %s" % which)


def make_backend(node, cfg, dry_run: bool = False) -> ArmBackend:
    """工厂：一行切换仿真/真机/演练。"""
    if dry_run:
        return DryRunBackend(node, cfg, dry_run=True)
    which = cfg.get("backend", "movej")
    if which == "movej":
        return MoveJBackend(node, cfg)
    if which == "action":
        return ActionBackend(node, cfg)
    if which == "dry-run":
        return DryRunBackend(node, cfg, dry_run=True)
    raise ValueError("未知 arm.backend: %s" % which)


# ---------------------------------------------------------------- FK 工具
def load_kinematics(cfg: dict):
    """加载仿真里验证过的 UrdfKinematics（正运动学 + DLS 逆解）。

    路径解析统一委托 common，保证「随包自带 URDF」这一份是唯一来源。
    """
    from common import find_urdf, setup_import_paths
    setup_import_paths(cfg)
    urdf = find_urdf(cfg["robot"] if "robot" in cfg else cfg)
    from ik_check import UrdfKinematics  # type: ignore
    r = cfg.get("robot", cfg)
    return UrdfKinematics(urdf, r["base_frame"], r["tip_frame"]), urdf

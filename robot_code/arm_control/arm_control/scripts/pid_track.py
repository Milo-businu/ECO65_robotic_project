#!/usr/bin/env python3
"""pid_track.py —— P3 完整形态：PID 闭环 → Gazebo 机械臂轨迹跟踪
修复：误差做角度归一化(wrap to [-π,π])，解决连续旋转关节 joint6 的环绕跳变。
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

JOINTS = ['joint1','joint2','joint3','joint4','joint5','joint6']
ACTION = '/rm_group_controller/follow_joint_trajectory'
KP, KI, KD = 0.5, 0.3, 0.05
DT = 0.15
MAX_STEP = 0.08
AMP, FREQ = 0.12, 0.25
LO = [-3.10, -3.10, -2.79, -3.10, -3.10, -6.28]
HI = [ 3.10,  2.35,  2.53,  3.10,  3.10,  6.28]

def wrap(a):
    """角度归一化到 [-π, π]：消除 ±2π 环绕造成的虚假大误差"""
    return np.array([math.atan2(math.sin(x), math.cos(x)) for x in a])

class PidTrack(Node):
    def __init__(self):
        super().__init__('pid_track')
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.q = self.q_home = None
        self.integ = np.zeros(6)
        self.prev_e = np.zeros(6)
        self.t0 = None
        self.pending = False
        self.errs = []
        self.get_logger().info('等待 action 服务器 ...')
        self.client.wait_for_server()
        self.get_logger().info('action 就绪，等待关节反馈 ...')
        self.create_timer(DT, self.on_timer)

    def on_joint(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.q = np.array([pos[j] for j in JOINTS])
            if self.q_home is None:
                self.q_home = self.q.copy()
                self.get_logger().info(f'起始关节角: {np.round(self.q, 3)}')

    def on_timer(self):
        if self.q is None or self.q_home is None or self.pending:
            return
        if self.t0 is None:
            self.t0 = self.get_clock().now()
            return
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        q_des = self.q_home + AMP*np.sin(2*math.pi*FREQ*(t+0.30) + np.arange(6)*0.3)  # 相位超前：抵消0.3s总滞后
        e = wrap(q_des - self.q)                            # ← 关键修复
        self.integ = np.clip(self.integ + e*DT, -0.5, 0.5)
        d = (e - self.prev_e)/DT
        self.prev_e = e
        step = np.clip(KP*e + KI*self.integ + KD*d, -MAX_STEP, MAX_STEP)
        q_cmd = np.clip(self.q + step, LO, HI)
        self.errs.append(e.copy())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        p0 = JointTrajectoryPoint()
        p0.positions = [float(x) for x in self.q]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = [float(x) for x in q_cmd]
        p1.time_from_start.sec = 0
        p1.time_from_start.nanosec = int(DT*1e9)
        goal.trajectory.points = [p0, p1]
        self.pending = True
        self.client.send_goal_async(goal).add_done_callback(self.on_goal)

    def on_goal(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.pending = False
            self.get_logger().warn('目标被拒绝')
            return
        gh.get_result_async().add_done_callback(lambda f: setattr(self, 'pending', False))

    def report(self):
        if not self.errs:
            return
        a = np.array(self.errs)
        self.get_logger().info(f'控制周期数: {len(a)}')
        for i, j in enumerate(JOINTS):
            rms = np.sqrt((a[:, i]**2).mean())
            self.get_logger().info(f'  {j}: RMS {rms:.5f} rad  最大 {np.abs(a[:, i]).max():.5f}')
        n = np.linalg.norm(a, axis=1)
        self.get_logger().info(f'总体范数 RMS: {np.sqrt((n**2).mean()):.5f} rad')

def main():
    rclpy.init()
    node = PidTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

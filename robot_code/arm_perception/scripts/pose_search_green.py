#!/usr/bin/env python3
"""pose_search_green.py —— 以"看到绿色"为标准的姿势搜索

与 pose_search.py 的区别：不假设方块位置（方块会动、会消失），
直接看每个姿势下相机图像里的绿色占比，谁看到得多谁胜出。
输出 TOP2 姿势的可复制命令——它们就是双视角测量的两个观测点。
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ACTION = '/rm_group_controller/follow_joint_trajectory'

CANDIDATES = [
    ("A1", [0, 0.3, 0.5, 0, 0, 0]),
    ("A1L", [0.15, 0.3, 0.5, 0, 0, 0]),
    ("A1R", [-0.15, 0.3, 0.5, 0, 0, 0]),
    ("A2", [0, 0.6, 1.0, 0, 0, 0]),
    ("A3", [0, 0.2, 0.35, 0, 0, 0]),
    ("B1", [0, 0.9, 1.2, 0, 0, 0]),
    ("C1", [0, 0.6, 1.0, 0, -0.6, 0]),
    ("C2", [0, 0.9, 1.2, 0, -0.8, 0]),
    ("D1", [0.3, 0.9, 1.2, 0, 0, 0]),
    ("D2", [-0.3, 0.9, 1.2, 0, 0, 0]),
]

def green_fraction(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    r = img[:, :, 0].astype(int); g = img[:, :, 1].astype(int); b = img[:, :, 2].astype(int)
    return ((g > 100) & (g > r + 40) & (g > b + 40)).mean() * 100

def main():
    rclpy.init()
    node = rclpy.create_node('pose_search_green')
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    latest = {'msg': None, 't': 0.0}

    def on_img(msg):
        latest['msg'] = msg
        latest['t'] = time.time()

    node.create_subscription(Image, '/wrist_depth_camera/image_raw', on_img, 10)
    if not client.wait_for_server(timeout_sec=5.0):
        print('action 服务器未就绪——确认仿真已启动')
        return

    def settle(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(node, timeout_sec=0.1)   # 睡眠期间持续处理图像

    results = []
    for name, q in CANDIDATES:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start.sec = 2
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().accepted:
            print(f'{name} goal 被拒绝，跳过')
            continue
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=10.0)
        settle(2.0)                                  # 停稳 + 攒新图像
        fresh = time.time() - latest['t'] < 1.0
        frac = green_fraction(latest['msg']) if fresh else -1.0
        mark = '✅' if frac > 0.3 else ('❌ 0%' if frac == 0 else '⚠️')
        results.append((name, q, frac))
        print(f'{name}  q={q}  绿色占比 {frac:5.2f}%  {mark}')

    results.sort(key=lambda r: -r[2])
    print('\n=== 按绿色占比排序 TOP5 ===')
    for name, q, frac in results[:5]:
        print(f'{name}: {frac:.2f}%')
    good = [r for r in results if r[2] > 0.3]
    if len(good) >= 2:
        for i, (name, q, _) in enumerate(good[:2], 1):
            cmd = ('ros2 action send_goal /rm_group_controller/follow_joint_trajectory '
                   'control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: '
                   '[joint1,joint2,joint3,joint4,joint5,joint6], points: [{positions: ['
                   + ', '.join(str(x) for x in q) + '], time_from_start: {sec: 2, nanosec: 0}}]}}"')
            print(f'\n观测点{i}（{name}）可复制命令：\n{cmd}')
    else:
        print('\n看到绿色的姿势不足 2 个——方块可能被挡住或太远，把结果发我分析')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

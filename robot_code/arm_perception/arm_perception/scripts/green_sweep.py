#!/usr/bin/env python3
"""green_sweep.py —— j2/j3 网格扫描找绿方块（25 个姿势），每姿势打印图像亮度+绿色占比
亮度诊断：均值≈0 = 全黑（渲染坏）；有亮度但绿色 0 = 方块不在视野（位置问题）
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

JOINTS = ['joint1','joint2','joint3','joint4','joint5','joint6']
ACTION = '/rm_group_controller/follow_joint_trajectory'

def main():
    rclpy.init()
    node = rclpy.create_node('green_sweep')
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    latest = {'msg': None, 't': 0.0}

    def on_img(m):
        latest['msg'] = m
        latest['t'] = time.time()

    node.create_subscription(Image, '/wrist_depth_camera/image_raw', on_img, 10)
    if not client.wait_for_server(timeout_sec=5.0):
        print('action 服务器未就绪'); return

    def stats():
        if latest['msg'] is None or time.time() - latest['t'] > 1.5:
            return None
        m = latest['msg']
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
        r = img[:,:,0].astype(int); g = img[:,:,1].astype(int); b = img[:,:,2].astype(int)
        green = ((g > 100) & (g > r + 40) & (g > b + 40)).mean() * 100
        return img.mean(), int(img.max()), green

    def move(q, settle=2.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]
        pt.time_from_start.sec = 2
        goal.trajectory.points = [pt]
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if fut.done() and fut.result().accepted:
            rf = fut.result().get_result_async()
            rclpy.spin_until_future_complete(node, rf, timeout_sec=10.0)
        t0 = time.time()
        while time.time() - t0 < settle:
            rclpy.spin_once(node, timeout_sec=0.1)

    # 零位诊断
    move([0,0,0,0,0,0], 3.0)
    s = stats()
    if s is None:
        print('零位无图像——相机话题有问题'); return
    print(f'零位诊断: 亮度均值 {s[0]:.1f} / 峰值 {s[1]} / 绿色 {s[2]:.2f}%')
    if s[0] < 1.0 and s[1] == 0:
        print('⚠️ 全黑！渲染坏了，先解决 xvfb 再说'); return

    results = []
    for j2 in [0.2, 0.4, 0.6, 0.8, 1.0]:
        for j3 in [0.2, 0.4, 0.6, 0.8, 1.0]:
            move([0, j2, j3, 0, 0, 0])
            s = stats()
            if s is None:
                print(f'j2={j2} j3={j3}  无图像'); continue
            bright, peak, green = s
            results.append((j2, j3, green))
            print(f'j2={j2:.1f} j3={j3:.1f}  亮度{bright:6.1f} 峰值{peak:3d}  绿色{green:5.2f}%  {"✅" if green > 0.3 else ""}')

    results.sort(key=lambda r: -r[2])
    print('\n=== 绿色占比 TOP3 ===')
    for j2, j3, green in results[:3]:
        print(f'j2={j2} j3={j3}  绿色 {green:.2f}%')
    if results and results[0][2] > 0.3:
        j2, j3, _ = results[0]
        print(f'\n最佳姿势命令（j2={j2}, j3={j3}）：\n'
              'ros2 action send_goal /rm_group_controller/follow_joint_trajectory '
              'control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: '
              '[joint1,joint2,joint3,joint4,joint5,joint6], points: [{positions: '
              f'[0, {j2}, {j3}, 0, 0, 0], time_from_start: {{sec: 2, nanosec: 0}}]}}}}"')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

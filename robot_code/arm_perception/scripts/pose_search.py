#!/usr/bin/env python3
"""pose_search.py —— 批量姿势搜索：自动发 goal、等执行、算偏离角与距离，输出排名

原理：光轴 = TF(world->camera_link) 的 +X 轴；方块方向 = (方块位置-相机位置)。
达标：偏离角 < 34°（视场内）且 距离 > 0.25m（避免碰撞与近距盲区）。
用法：仿真启动后直接运行。结束后自动打印最佳姿势的可复制命令。
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from tf2_ros import Buffer, TransformListener

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ACTION = '/rm_group_controller/follow_joint_trajectory'
BOX = np.array([0.4, 0.0, 0.04])

# 候选姿势池：覆盖 立姿/弯姿/腕压/左右转 四类
CANDIDATES = [
    ("A1", [0, 0.3, 0.5, 0, 0, 0]),
    ("A2", [0, 0.6, 1.0, 0, 0, 0]),
    ("A3", [0, 0.2, 0.35, 0, 0, 0]),
    ("B1", [0, 0.9, 1.2, 0, 0, 0]),
    ("B2", [0, 1.2, 1.5, 0, 0, 0]),
    ("C1", [0, 0.6, 1.0, 0, -0.6, 0]),
    ("C2", [0, 0.9, 1.2, 0, -0.8, 0]),
    ("C3", [0, 1.2, 1.5, 0, -1.2, 0]),
    ("D1", [0.3, 0.9, 1.2, 0, 0, 0]),
    ("D2", [-0.3, 0.9, 1.2, 0, 0, 0]),
]

def quat_to_matrix(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def main():
    rclpy.init()
    node = rclpy.create_node('pose_search')
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    buffer = Buffer()
    TransformListener(buffer, node)
    if not client.wait_for_server(timeout_sec=5.0):
        print('action 服务器未就绪——确认仿真已启动')
        return
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
        time.sleep(1.5)                       # 等振动衰减
        try:
            tf = buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())
        except Exception as exc:
            print(f'{name} TF 失败: {exc}')
            continue
        t, qq = tf.transform.translation, tf.transform.rotation
        p = np.array([t.x, t.y, t.z])
        axis = quat_to_matrix(qq.x, qq.y, qq.z, qq.w) @ np.array([1.0, 0.0, 0.0])
        d = BOX - p
        dist = float(np.linalg.norm(d))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(axis, d / dist))))))
        ok = ang < 34.0 and dist > 0.25
        results.append((name, q, ang, dist, ok))
        print(f'{name}  q={q}  偏离角{ang:5.1f}°  距离{dist:.2f}m  {"✅ 达标" if ok else "❌"}')
    results.sort(key=lambda r: r[2])
    print('\n=== 按偏离角排序 TOP5 ===')
    for name, q, ang, dist, ok in results[:5]:
        print(f'{name}: 角度{ang:.1f}°  距离{dist:.2f}m  {"达标" if ok else ""}')
    if results and results[0][2] < 34.0 and results[0][3] > 0.25:
        name, q, _, _, _ = results[0]
        cmd = ('ros2 action send_goal /rm_group_controller/follow_joint_trajectory '
               'control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: '
               '[joint1,joint2,joint3,joint4,joint5,joint6], points: [{positions: ['
               + ', '.join(str(x) for x in q) + '], time_from_start: {sec: 2, nanosec: 0}}]}}"')
        print(f'\n最佳姿势 {name}，可直接复制运行：\n{cmd}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

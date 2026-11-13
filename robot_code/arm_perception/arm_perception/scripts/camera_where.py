#!/usr/bin/env python3
"""camera_where.py —— 诊断：相机光轴指向哪？方块在不在视场内？

原理：Gazebo 相机沿自身坐标系 +X 轴看，水平视场半角 0.6 rad(±34°)。
方法：从 TF 取 world->camera_link 变换，算光轴向量与"相机->方块"方向的夹角。
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

BOX_POS = np.array([0.4, 0.0, 0.04])   # 方块位置（spawn 时定的）
HFOV_HALF = 0.6                         # 水平视场半角（弧度）

def quat_to_matrix(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

class CameraWhere(Node):
    def __init__(self):
        super().__init__('camera_where')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(2.0, self.on_timer)

    def on_timer(self):
        try:
            tf = self.buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'等待 TF world->camera_link：{exc}')
            return
        t, q = tf.transform.translation, tf.transform.rotation
        p = np.array([t.x, t.y, t.z])
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        axis = R @ np.array([1.0, 0.0, 0.0])          # 相机光轴（世界系）
        d = BOX_POS - p
        dist = np.linalg.norm(d)
        if dist < 1e-6:
            return
        direction = d / dist
        angle = math.acos(max(-1.0, min(1.0, float(np.dot(axis, direction)))))
        self.get_logger().info(
            f'相机位置 [{p[0]:.2f} {p[1]:.2f} {p[2]:.2f}]  '
            f'光轴 [{axis[0]:.2f} {axis[1]:.2f} {axis[2]:.2f}]  '
            f'距方块 {dist:.2f}m  偏离角 {math.degrees(angle):.1f}°  '
            f'{"✅ 在视场内" if angle < HFOV_HALF else "❌ 在视场外(需<34°)"}')

def main():
    rclpy.init()
    node = CameraWhere()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

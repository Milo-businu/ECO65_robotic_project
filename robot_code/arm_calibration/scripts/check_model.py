#!/usr/bin/env python3
"""check_model.py —— P1 模型校验工具（ECO65-6FB）

作用：读取仿真里真实的 TF，打印基座到末端的位姿，
      用于和 MDH 手算值对比，确认模型骨架正确。

用法（仿真要先启动）：
  source /opt/ros/foxy/setup.bash
  python3 ~/robot_vla_ws/src/arm_calibration/check_model.py
"""
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

BASE_FRAME = 'baselink'   # 官方 rm_description 的根系名（不是 base_link）
TIP_FRAME = 'Link6'       # 末端（无 tool0）

class ModelCheck(Node):
    def __init__(self):
        super().__init__('model_check')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(2.0, self.on_timer)

    def on_timer(self):
        try:
            tf = self.buffer.lookup_transform(BASE_FRAME, TIP_FRAME, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'等待 TF（{BASE_FRAME} -> {TIP_FRAME}）：{exc}')
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self.get_logger().info(
            f'{BASE_FRAME} -> {TIP_FRAME}  位置 x={t.x:.4f} y={t.y:.4f} z={t.z:.4f} | '
            f'四元数 x={q.x:.4f} y={q.y:.4f} z={q.z:.4f} w={q.w:.4f}'
        )

def main():
    rclpy.init()
    node = ModelCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

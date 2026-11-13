#!/usr/bin/env python3
"""see_green.py —— 机械臂的第一眼：订阅腕部相机彩色图，数值化检测绿色方块

无 GUI 验证法（云端适配）：不显示图像，统计"绿色像素占比"——
  方块在画面里 → 占比明显 >0
  相机朝天/朝地 → 占比 ≈ 0
用法：仿真启动后运行，Ctrl+C 退出
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

TOPIC = '/wrist_depth_camera/image_raw'

class SeeGreen(Node):
    def __init__(self):
        super().__init__('see_green')
        self.n = 0
        self.create_subscription(Image, TOPIC, self.on_image, 10)
        self.get_logger().info(f'等待 {TOPIC} ...')

    def on_image(self, msg):
        self.n += 1
        if self.n % 15 != 1:      # 每秒只报一次（15Hz 采样 1/15）
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        r = img[:, :, 0].astype(int)
        g = img[:, :, 1].astype(int)
        b = img[:, :, 2].astype(int)
        green = (g > 100) & (g > r + 40) & (g > b + 40)
        frac = green.mean() * 100
        verdict = '✅ 看到绿方块！' if frac > 0.5 else '（视野里没有绿色）'
        self.get_logger().info(
            f'帧{self.n:4d}  {msg.width}x{msg.height} {msg.encoding}  绿色占比 {frac:5.2f}%  {verdict}')

def main():
    rclpy.init()
    node = SeeGreen()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

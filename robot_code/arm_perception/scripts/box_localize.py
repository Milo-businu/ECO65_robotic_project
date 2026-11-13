#!/usr/bin/env python3
"""box_localize.py —— 绿方块 3D 定位（感知 + 手眼 TF 的合龙测试）

链路：彩色图找绿色像素质心(u,v) → 深度点云取该像素 3D 点(相机系) → TF 转世界系。
验证方法：双视角一致性——两个不同姿势估计的世界坐标应一致（不依赖硬编码真值，
          物体被撞到哪儿都不影响结论）。真值仅作参考打印。

用法：仿真启动 + 手臂停在能看到方块的姿势，运行约 15 秒后 Ctrl+C 看均值。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformListener

IMG_TOPIC = '/wrist_depth_camera/image_raw'
PTS_TOPIC = '/wrist_depth_camera/points'
BOX_REF = np.array([0.4, 0.0, 0.04])   # 仅作参考（方块若被撞动此项失效，属正常）
MAX_DT = 0.3      # 图像与点云允许的时间差(秒)
MIN_GREEN = 50    # 绿色像素少于此值认为没看见

def quat_to_matrix(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def cloud_to_xyz(msg):
    """按字段偏移解析 PointCloud2 -> (N,) 结构化数组（零外部依赖）"""
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'],
                   'formats': [np.float32] * 3,
                   'offsets': [off['x'], off['y'], off['z']],
                   'itemsize': msg.point_step})
    return np.frombuffer(msg.data, dtype=dt)

class BoxLocalize(Node):
    def __init__(self):
        super().__init__('box_localize')
        self.img = None
        self.pts = None
        self.count = 0
        self.ests = []
        self.create_subscription(Image, IMG_TOPIC, self.on_image, 10)
        self.create_subscription(PointCloud2, PTS_TOPIC, self.on_points, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.get_logger().info(f'等待 {IMG_TOPIC} 与 {PTS_TOPIC} ...')

    def on_image(self, msg):
        self.img = msg
        self.try_localize()

    def on_points(self, msg):
        self.pts = msg

    def try_localize(self):
        im, pc = self.img, self.pts
        if im is None or pc is None:
            return
        ti = im.header.stamp.sec + im.header.stamp.nanosec * 1e-9
        tp = pc.header.stamp.sec + pc.header.stamp.nanosec * 1e-9
        if abs(ti - tp) > MAX_DT:
            return
        img = np.frombuffer(im.data, dtype=np.uint8).reshape(im.height, im.width, -1)
        r = img[:, :, 0].astype(int); g = img[:, :, 1].astype(int); b = img[:, :, 2].astype(int)
        green = (g > 100) & (g > r + 40) & (g > b + 40)
        if green.sum() < MIN_GREEN:
            return
        ys, xs = np.nonzero(green)
        u, v = int(xs.mean()), int(ys.mean())
        cloud = cloud_to_xyz(pc)   # 非结构化点云：数据仍按图像行优先顺序排列
        n = im.width * im.height
        if cloud.shape[0] != n:
            self.get_logger().warn(f'点云数量 {cloud.shape[0]} 与图像 {im.width}x{im.height} 不符，跳过')
            return
        idx = v * im.width + u     # 行优先展开：第v行第u列 -> 第 v*宽+u 个点
        p_opt = np.array([cloud[idx]['x'], cloud[idx]['y'], cloud[idx]['z']], dtype=float)
        # 点云是光学系约定(z前向,x右,y下)，先转到 camera_link 系(x前向,y左,z上)
        M = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        p_cam = M @ p_opt
        if not np.all(np.isfinite(p_cam)) or np.linalg.norm(p_cam) < 1e-6:
            return                      # 该像素无有效深度
        try:
            tf = self.buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())  # 用最新TF：点云戳是仿真时钟，TF是墙钟，时钟源不一致
        except Exception as exc:
            self.get_logger().warn(f'等待 TF：{exc}')
            return
        t, q = tf.transform.translation, tf.transform.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        p_world = R @ p_cam + np.array([t.x, t.y, t.z])
        self.count += 1
        self.ests.append(p_world)
        if self.count % 10 == 1:
            self.get_logger().info(
                f'#{self.count:3d} 像素质心({u},{v}) 相机系[{p_cam[0]:.3f} {p_cam[1]:.3f} {p_cam[2]:.3f}]m  '
                f'世界系[{p_world[0]:.3f} {p_world[1]:.3f} {p_world[2]:.3f}]m  '
                f'(参考值差 {np.linalg.norm(p_world - BOX_REF)*100:.1f} cm)')
        if len(self.ests) % 20 == 0:
            a = np.array(self.ests[-20:])
            self.get_logger().info(
                f'  近20次均值 [{a[:,0].mean():.3f} {a[:,1].mean():.3f} {a[:,2].mean():.3f}]  '
                f'抖动std [{a[:,0].std()*100:.1f} {a[:,1].std()*100:.1f} {a[:,2].std()*100:.1f}] cm')

def main():
    rclpy.init()
    node = BoxLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.ests:
            a = np.array(node.ests)
            node.get_logger().info(
                f'>>> 本次样本 {len(a)}  均值 [{a[:,0].mean():.3f} {a[:,1].mean():.3f} {a[:,2].mean():.3f}]  '
                f'std [{a[:,0].std()*100:.1f} {a[:,1].std()*100:.1f} {a[:,2].std()*100:.1f}] cm')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

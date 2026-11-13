#!/usr/bin/env python3
"""probe_spawn.py —— 把探针方块自动生成在相机视野正中央

原理：从 TF 取相机光轴（world->camera_link 的 +X），算它与地面 z=0.04 的交点，
在该点 spawn 一个绿方块（entity 名 probe_box）。看哪种哪，绝不脱靶。
"""
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def quat_to_matrix(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

rclpy.init()
node = rclpy.create_node('probe_spawn')
buffer = Buffer()
TransformListener(buffer, node)

tf = None
for _ in range(50):                       # 最多等 5 秒
    try:
        tf = buffer.lookup_transform('world', 'camera_link', rclpy.time.Time())
        break
    except Exception:
        rclpy.spin_once(node, timeout_sec=0.1)
if tf is None:
    print('拿不到 TF——确认仿真已启动')
    raise SystemExit(1)

t, q = tf.transform.translation, tf.transform.rotation
p = np.array([t.x, t.y, t.z])
axis = quat_to_matrix(q.x, q.y, q.z, q.w) @ np.array([1.0, 0.0, 0.0])
print(f'相机位置 [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}]  光轴 [{axis[0]:.3f} {axis[1]:.3f} {axis[2]:.3f}]')

if axis[2] > -0.1:
    print('⚠️ 相机几乎不朝下看，光轴与地面交点太远——把此输出发我')
    raise SystemExit(1)

tt = (0.04 - p[2]) / axis[2]              # 与 z=0.04 平面的交点
target = p + tt * axis
print(f'光轴落点（方块将生成在此）: x={target[0]:.3f}  y={target[1]:.3f}  距离 {tt:.2f}m')

r = subprocess.run(['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-file', '/home/ubuntu/robot_vla_ws/models/target_box/model.sdf',
                    '-entity', 'probe_box',
                    '-x', f'{target[0]:.3f}', '-y', f'{target[1]:.3f}', '-z', '0.05'],
                   capture_output=True, text=True, timeout=30)
print(r.stdout.strip() or r.stderr.strip())

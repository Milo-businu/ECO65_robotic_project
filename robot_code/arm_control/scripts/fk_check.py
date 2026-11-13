#!/usr/bin/env python3
"""fk_check.py —— 纯 numpy 版 FK：直接解析 URDF 算正运动学，与仿真 TF 对账

原理：URDF 里每个关节 = 固定偏移(origin xyz/rpy) × 绕轴旋转(axis, q)，
把 6 个关节的 4x4 矩阵连乘，就得到末端位姿——这就是 FK 的全部。
"""
import math
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

URDF_PATH = '/tmp/eco65_6fb.urdf'
BASE_FRAME = 'baselink'
TIP_FRAME = 'Link6'

def rpy_to_matrix(r, p, y):
    """固定轴欧拉角 roll/pitch/yaw -> 旋转矩阵"""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]])

def axis_angle_matrix(axis, angle):
    """轴角 -> 旋转矩阵（罗德里格斯公式）"""
    x, y, z = axis / np.linalg.norm(axis)
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [x*x*C+c,   x*y*C-z*s, x*z*C+y*s],
        [y*x*C+z*s, y*y*C+c,   y*z*C-x*s],
        [z*x*C-y*s, z*y*C+x*s, z*z*C+c]])

def xyz_rpy_to_matrix(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T

class UrdfKinematics:
    """解析 URDF，抽出 baselink -> Link6 的关节链，提供 FK"""
    def __init__(self, path, base, tip):
        root = ET.parse(path).getroot()
        self.chain = []   # [(关节名, 子连杆, 固定变换T0, 轴, 类型)]
        link = base
        while link != tip:
            found = None
            for j in root.findall('joint'):
                if j.find('parent').attrib['link'] == link:
                    found = j
                    break
            if found is None:
                raise RuntimeError(f'从 {link} 找不到子关节，检查帧名')
            o = found.find('origin')
            xyz = np.array([float(v) for v in (o.attrib.get('xyz', '0 0 0') if o is not None else '0 0 0').split()])
            rpy = np.array([float(v) for v in (o.attrib.get('rpy', '0 0 0') if o is not None else '0 0 0').split()])
            a = found.find('axis')
            axis = np.array([float(v) for v in a.attrib['xyz'].split()]) if a is not None else np.array([0., 0., 1.])
            child = found.find('child').attrib['link']
            self.chain.append((found.attrib['name'], child, xyz_rpy_to_matrix(xyz, rpy), axis, found.attrib['type']))
            link = child
        self.movable = [c[0] for c in self.chain if c[4] in ('revolute', 'continuous')]

    def fk(self, q_map):
        """q_map: {关节名: 角度(弧度)} -> 4x4 末端位姿"""
        T = np.eye(4)
        for name, child, T0, axis, jtype in self.chain:
            if jtype in ('revolute', 'continuous'):
                Tj = np.eye(4)
                Tj[:3, :3] = axis_angle_matrix(axis, float(q_map[name]))
                T = T @ (T0 @ Tj)
            else:
                T = T @ T0
        return T

class FkCheck(Node):
    def __init__(self):
        super().__init__('fk_check')
        self.kin = UrdfKinematics(URDF_PATH, BASE_FRAME, TIP_FRAME)
        self.get_logger().info(f'URDF 关节链解析成功，可动关节: {self.kin.movable}')
        self.q_map = None
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(3.0, self.on_timer)

    def on_joint(self, msg):
        self.q_map = dict(zip(msg.name, msg.position))

    def on_timer(self):
        if self.q_map is None:
            self.get_logger().warn('等待 /joint_states ...')
            return
        T = self.kin.fk(self.q_map)
        p_kdl = T[:3, 3]
        try:
            tf = self.buffer.lookup_transform(BASE_FRAME, TIP_FRAME, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'等待 TF：{exc}')
            return
        t = tf.transform.translation
        p_tf = np.array([t.x, t.y, t.z])
        err = np.linalg.norm(p_kdl - p_tf)
        self.get_logger().info(
            f'FK:  [{p_kdl[0]:.4f} {p_kdl[1]:.4f} {p_kdl[2]:.4f}]  '
            f'TF:  [{p_tf[0]:.4f} {p_tf[1]:.4f} {p_tf[2]:.4f}]  '
            f'误差: {err*1000:.3f} mm'
        )

def main():
    rclpy.init()
    node = FkCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

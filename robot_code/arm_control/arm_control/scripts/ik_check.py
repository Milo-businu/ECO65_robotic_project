#!/usr/bin/env python3
"""ik_check.py —— IK 修正版：离开奇异位形起步 + 真阻尼 + 步长裁剪"""
import math
import xml.etree.ElementTree as ET
import numpy as np

URDF_PATH = '/tmp/eco65_6fb.urdf'
BASE_FRAME, TIP_FRAME = 'baselink', 'Link6'

def rpy_to_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]])

def axis_angle_matrix(axis, angle):
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

def rotvec_from_matrix(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if abs(ang) < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*math.sin(ang))
    return axis * ang

class UrdfKinematics:
    def __init__(self, path, base, tip):
        root = ET.parse(path).getroot()
        self.chain = []
        link = base
        while link != tip:
            found = None
            for j in root.findall('joint'):
                if j.find('parent').attrib['link'] == link:
                    found = j
                    break
            if found is None:
                raise RuntimeError(f'从 {link} 找不到子关节')
            o = found.find('origin')
            xyz = np.array([float(v) for v in (o.attrib.get('xyz','0 0 0') if o is not None else '0 0 0').split()])
            rpy = np.array([float(v) for v in (o.attrib.get('rpy','0 0 0') if o is not None else '0 0 0').split()])
            a = found.find('axis')
            axis = np.array([float(v) for v in a.attrib['xyz'].split()]) if a is not None else np.array([0.,0.,1.])
            child = found.find('child').attrib['link']
            self.chain.append((found.attrib['name'], child, xyz_rpy_to_matrix(xyz, rpy), axis, found.attrib['type']))
            link = child
        self.movable = [c[0] for c in self.chain if c[4] in ('revolute','continuous')]
        self.n = len(self.movable)

    def fk(self, q):
        T = np.eye(4)
        i = 0
        for name, child, T0, axis, jtype in self.chain:
            if jtype in ('revolute', 'continuous'):
                Tj = np.eye(4)
                Tj[:3, :3] = axis_angle_matrix(axis, float(q[i]))
                i += 1
                T = T @ (T0 @ Tj)
            else:
                T = T @ T0
        return T

def numeric_jacobian(kin, q, eps=1e-6):
    J = np.zeros((6, kin.n))
    for i in range(kin.n):
        qp, qm = list(q), list(q)
        qp[i] += eps; qm[i] -= eps
        Tp, Tm = kin.fk(qp), kin.fk(qm)
        J[:3, i] = (Tp[:3,3] - Tm[:3,3]) / (2*eps)
        R_rel = Tp[:3,:3] @ Tm[:3,:3].T
        J[3:, i] = rotvec_from_matrix(R_rel) / (2*eps)
    return J

def dls_ik(kin, q_init, p_des, R_des, max_iter=500, tol=1e-4, lam=1e-3, max_step=0.15):
    """修复点：lam 1e-3（真阻尼）、每步关节角增量裁剪 ≤0.15 rad（防过冲）、tol 1e-4"""
    q = list(q_init)
    for _ in range(max_iter):
        T = kin.fk(q)
        e = np.concatenate([p_des - T[:3,3], rotvec_from_matrix(R_des @ T[:3,:3].T)])
        if np.linalg.norm(e) < tol:
            return q, True, float(np.linalg.norm(e))
        J = numeric_jacobian(kin, q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam*lam*np.eye(6), e)
        step = np.max(np.abs(dq))
        if step > max_step:
            dq = dq * (max_step / step)
        q = [max(-2*math.pi, min(2*math.pi, qi + dqi)) for qi, dqi in zip(q, dq)]
    T = kin.fk(q)
    e = np.concatenate([p_des - T[:3,3], rotvec_from_matrix(R_des @ T[:3,:3].T)])
    return q, False, float(np.linalg.norm(e))

def main():
    kin = UrdfKinematics(URDF_PATH, BASE_FRAME, TIP_FRAME)
    # 修复点：初始猜测让肘部微弯（q2=0.2, q3=0.3），离开肘部奇异位形
    q_init = [0.0, 0.2, 0.3, 0.0, 0.0, 0.0]
    T0 = kin.fk(q_init)
    print(f'起点末端: [{T0[0,3]:.4f} {T0[1,3]:.4f} {T0[2,3]:.4f}]')
    p_des = T0[:3,3] + np.array([0.05, 0.02, -0.03])
    R_des = T0[:3,:3]
    print(f'目标末端: [{p_des[0]:.4f} {p_des[1]:.4f} {p_des[2]:.4f}]')
    q_sol, ok, err = dls_ik(kin, q_init, p_des, R_des)
    print(f'IK 收敛: {ok}   最终误差: {err*1000:.4f}')
    print(f'解出关节角(度): {[round(math.degrees(x), 2) for x in q_sol]}')
    Tc = kin.fk(q_sol)
    print(f'FK 回验位置误差: {np.linalg.norm(Tc[:3,3] - p_des)*1000:.4f} mm')

if __name__ == '__main__':
    main()

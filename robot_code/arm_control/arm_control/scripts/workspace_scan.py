#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
workspace_scan.py —— ECO65 工作空间扫描（可达空间 + 灵巧度）

核心思路（纯数学，完全不需要动机械臂）：
  1) 直接解析 URDF 拿运动链与关节限位（不用 MDH 手填参数 —— P2 教训：以 URDF 为准）
  2) 在关节限位内蒙特卡洛采样 N 组关节角
  3) FK 算末端位置 -> 点云 = 可达空间
  4) 对候选点换 K 个末端姿态求 IK，全解出的才算"灵巧点"

准备（在新设备上）：
  sb                                   # 加载工作空间
  urdf                                 # 生成 /tmp/eco65_6fb.urdf（每次重启都要重跑）
  sudo apt install -y python3-matplotlib   # 出图用（没装也能跑，只是不出 PNG）

用法：
  python3 workspace_scan.py                          # 默认 20 万点 + 灵巧度评估
  python3 workspace_scan.py --no-dex                 # 只要可达空间，几秒钟
  python3 workspace_scan.py --n 500000 --out ~/ws_out
  python3 workspace_scan.py --tip Link6 --dex-points 300 --dex-ori 12

输出（默认目录 ~/workspace_scan_out）：
  reachable_points.npy / reachable.csv   可达空间点云
  reachable_3d.png                       三维散点
  reachable_views.png                    三视图投影（XY / XZ / YZ）
  dexterity.csv                          候选点灵巧度评分（x,y,z,score）
  dexterity_slice.png                    灵巧度切片着色图
  stats.txt                              数值统计
"""

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

# 服务器上实测的零位 FK 黄金指纹（米），用来给本机结果做交叉验证
GOLDEN_ZERO_POS = np.array([-0.086, 0.021, 0.772])


# ---------------------------------------------------------------- 基础数学

def rpy_to_R(rpy):
    """URDF 的 rpy（roll/pitch/yaw）转旋转矩阵，顺序 R = Rz(y) @ Ry(p) @ Rx(r)"""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_axis_batch(axis, q):
    """绕任意轴旋转，向量化。axis:(3,)  q:(N,) -> (N,3,3)（罗德里格斯公式）"""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        axis = np.array([0.0, 0.0, 1.0])
    else:
        axis = axis / n
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    c = np.cos(q)[:, None, None]
    s = np.sin(q)[:, None, None]
    I = np.eye(3)[None, :, :]
    outer = np.outer(axis, axis)[None, :, :]
    return I * c + K[None, :, :] * s + outer * (1.0 - c)


def rotvec_from_R(R):
    """旋转矩阵 -> 旋转向量（轴 * 角），用于姿态误差"""
    tr = (np.trace(R) - 1.0) / 2.0
    ang = math.acos(max(-1.0, min(1.0, tr)))
    skew = np.array([R[2, 1] - R[1, 2],
                     R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]])
    if ang < 1e-8:
        return 0.5 * skew
    s = 2.0 * math.sin(ang)
    if abs(s) < 1e-12:      # 接近 pi 的翻转，退化处理
        return skew * 0.5
    return skew / s * ang


def random_rotation(rng):
    """均匀随机旋转矩阵（Shoemake 四元数采样），保证 SO(3) 上均匀"""
    u1, u2, u3 = rng.random(3)
    a = math.sqrt(1.0 - u1)
    b = math.sqrt(u1)
    qw, qx = a * math.sin(2 * math.pi * u2), a * math.cos(2 * math.pi * u2)
    qy, qz = b * math.sin(2 * math.pi * u3), b * math.cos(2 * math.pi * u3)
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


# ---------------------------------------------------------------- URDF 解析

def parse_urdf(path):
    """读出所有关节：父/子连杆、原点 xyz+rpy、轴、限位"""
    if not os.path.isfile(path):
        sys.exit("找不到 URDF：%s\n先跑一下 alias 里的 urdf 命令生成它。" % path)
    root = ET.parse(path).getroot()
    joints = []
    for j in root.findall('joint'):
        jtype = j.get('type')
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        o = j.find('origin')
        xyz = np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()]) if o is not None else np.zeros(3)
        rpy = np.array([float(v) for v in (o.get('rpy') or '0 0 0').split()]) if o is not None else np.zeros(3)
        ax = j.find('axis')
        axis = np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else np.array([0.0, 0.0, 1.0])
        lim = j.find('limit')
        if lim is not None and lim.get('lower') is not None:
            lo, hi = float(lim.get('lower')), float(lim.get('upper'))
        else:
            lo, hi = -math.pi, math.pi
        joints.append(dict(name=j.get('name'), type=jtype, parent=parent, child=child,
                           xyz=xyz, rpy=rpy, axis=axis, lo=lo, hi=hi))
    return joints


def build_chain(joints, base, tip=None):
    """从 base 一路往下走，串起运动链（优先走能动关节，fixed 关节也保留）"""
    children = {}
    for jd in joints:
        if jd['type'] in ('revolute', 'continuous', 'prismatic', 'fixed'):
            children.setdefault(jd['parent'], []).append(jd)
    chain, cur = [], base
    while True:
        cands = children.get(cur, [])
        if not cands:
            break
        nxt = None
        for c in cands:                       # 优先能动关节
            if c['type'] in ('revolute', 'continuous', 'prismatic'):
                nxt = c
                break
        if nxt is None:
            nxt = cands[0]
        chain.append(nxt)
        cur = nxt['child']
        if tip and cur == tip:                # 到达指定末端就停（丢掉后面的 fixed 法兰）
            break
        if len(chain) > 24:
            break
    return chain


# ---------------------------------------------------------------- 运动学

class Kinematics:
    def __init__(self, chain):
        self.chain = chain
        self.dof_idx = [i for i, jd in enumerate(chain)
                        if jd['type'] in ('revolute', 'continuous', 'prismatic')]
        self.ndof = len(self.dof_idx)
        self.lo = np.array([chain[i]['lo'] for i in self.dof_idx])
        self.hi = np.array([chain[i]['hi'] for i in self.dof_idx])
        # 静态原点变换
        self.T0 = []
        for jd in chain:
            T = np.eye(4)
            T[:3, :3] = rpy_to_R(jd['rpy'])
            T[:3, 3] = jd['xyz']
            self.T0.append(T)

    def fk(self, q):
        """q: (N,dof) -> (N,4,4)，批量正解"""
        q = np.atleast_2d(np.asarray(q, dtype=float))
        N = q.shape[0]
        T = np.tile(np.eye(4), (N, 1, 1))
        k = 0
        for j, jd in enumerate(self.chain):
            To = self.T0[j]
            if jd['type'] in ('revolute', 'continuous', 'prismatic') and jd['type'] != 'prismatic':
                R = rot_axis_batch(jd['axis'], q[:, k])
                M = np.zeros((N, 4, 4))
                M[:, :3, :3] = np.einsum('ij,njk->nik', To[:3, :3], R)
                M[:, :3, 3] = To[:3, 3]
                M[:, 3, 3] = 1.0
                k += 1
            else:                              # fixed 关节 / 移动关节按 0 位处理
                M = np.tile(To, (N, 1, 1))
                if jd['type'] == 'prismatic':
                    k += 1
            T = np.einsum('nij,njk->nik', T, M)
        return T

    def fk_one(self, q):
        return self.fk(np.asarray(q, dtype=float).reshape(1, -1))[0]

    def jacobian(self, q, eps=1e-6):
        """数值雅可比（6 x dof）：上 3 行线速度，下 3 行角速度"""
        T0 = self.fk_one(q)
        p0, R0 = T0[:3, 3], T0[:3, :3]
        J = np.zeros((6, self.ndof))
        for c, i in enumerate(self.dof_idx):
            qp = np.array(q, dtype=float)
            qp[i] += eps
            Tp = self.fk_one(qp)
            J[:3, c] = (Tp[:3, 3] - p0) / eps
            J[3:, c] = rotvec_from_R(Tp[:3, :3] @ R0.T) / eps
        return J

    def ik_dls(self, p_t, R_t, q0, iters=60, lam=0.05, pos_tol=1e-4, rot_tol=5e-3):
        """阻尼最小二乘数值 IK（奇异点附近靠 lam 扛住）"""
        q = np.array(q0, dtype=float)
        for _ in range(iters):
            T = self.fk_one(q)
            e_p = np.asarray(p_t) - T[:3, 3]
            e_r = rotvec_from_R(np.asarray(R_t) @ T[:3, :3].T)
            if np.linalg.norm(e_p) < pos_tol and np.linalg.norm(e_r) < rot_tol:
                return q, True
            J = self.jacobian(q)
            err = np.concatenate([e_p, e_r])
            A = J @ J.T + (lam ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(A, err)
            q = q + np.clip(dq, -0.3, 0.3)                 # 限制单步步长，防炸
            q = np.clip(q, self.lo, self.hi)               # 始终待在限位内
        T = self.fk_one(q)
        ok = (np.linalg.norm(np.asarray(p_t) - T[:3, 3]) < pos_tol
              and np.linalg.norm(rotvec_from_R(np.asarray(R_t) @ T[:3, :3].T)) < rot_tol)
        return q, ok


# ---------------------------------------------------------------- 主流程

def thin_to_grid(pts, qs, step):
    """按体素稀释点云：每个占用格子留一个质心，并记住"到达过这个格子"的关节姿态

    记下 qs 很关键 —— 后面求 IK 时拿它当初始猜测，收敛率比随机初值高得多，
    否则会把"其实灵巧"的点误判成不灵巧（假阴性）。
    """
    if len(pts) == 0:
        return pts, qs
    key = np.floor(pts / step).astype(np.int64)
    _, uniq_idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    sums = np.zeros((len(uniq_idx), 3))
    qsum = np.zeros((len(uniq_idx), qs.shape[1]))
    cnt = np.zeros(len(uniq_idx))
    np.add.at(sums, inv, pts)
    np.add.at(qsum, inv, qs)
    np.add.at(cnt, inv, 1.0)
    return sums / cnt[:, None], qsum / cnt[:, None]


def main():
    ap = argparse.ArgumentParser(description='ECO65 工作空间扫描：可达空间 + 灵巧度')
    ap.add_argument('--urdf', default='/tmp/eco65_6fb.urdf')
    ap.add_argument('--base', default='baselink')
    ap.add_argument('--tip', default=None, help='末端连杆名，默认一路走到最后')
    ap.add_argument('--n', type=int, default=200000, help='采样点数')
    ap.add_argument('--no-dex', action='store_true', help='跳过灵巧度评估（快）')
    ap.add_argument('--dex-points', type=int, default=200, help='灵巧度候选点数')
    ap.add_argument('--dex-ori', type=int, default=12, help='每点随机姿态数')
    ap.add_argument('--dex-iters', type=int, default=60, help='IK 迭代上限')
    ap.add_argument('--dex-restarts', type=int, default=2, help='每个姿态的 IK 重试次数（防假阴性）')
    ap.add_argument('--grid', type=float, default=0.05, help='候选点体素边长(米)')
    ap.add_argument('--out', default=os.path.expanduser('~/workspace_scan_out'))
    ap.add_argument('--seed', type=int, default=42)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    os.makedirs(args.out, exist_ok=True)

    joints = parse_urdf(args.urdf)
    chain = build_chain(joints, args.base, args.tip)
    if not chain:
        sys.exit('没走出运动链，检查 --base / --tip 名称（真实帧名是 baselink / Link6）')
    kin = Kinematics(chain)

    print('=' * 66)
    print('运动链（%d 个关节，其中 %d 个能动）' % (len(chain), kin.ndof))
    for jd in chain:
        if jd['type'] in ('revolute', 'continuous', 'prismatic'):
            print('  %-28s %-10s 限位 [%7.3f, %7.3f] rad' % (jd['name'], jd['type'], jd['lo'], jd['hi']))
        else:
            print('  %-28s %-10s (fixed)' % (jd['name'], jd['type']))
    print('末端连杆：%s' % chain[-1]['child'])

    # ---- 自检：零位 FK 对黄金指纹（这是本脚本唯一可信的"真值"来源）
    q_zero = np.zeros(kin.ndof)
    p_zero = kin.fk_one(q_zero)[:3, 3]
    d = np.linalg.norm(p_zero - GOLDEN_ZERO_POS)
    print('\n自检：零位 FK = [%7.4f, %7.4f, %7.4f] m' % tuple(p_zero))
    print('      黄金指纹 = [%7.4f, %7.4f, %7.4f] m，偏差 %.2f mm' % (*GOLDEN_ZERO_POS, d * 1000))
    if d > 0.005:
        print('      ⚠ 偏差 >5mm：末端链可能多走/少走了 fixed 关节，试试 --tip Link6')
    else:
        print('      ✅ 对齐，运动链可信')

    # ---- 1) 可达空间
    print('\n[1/3] 采样 %d 组关节角 ...' % args.n)
    Q = rng.uniform(kin.lo, kin.hi, size=(args.n, kin.ndof))
    P = kin.fk(Q)[:, :3, 3]
    np.save(os.path.join(args.out, 'reachable_points.npy'), P)
    np.savetxt(os.path.join(args.out, 'reachable.csv'), P, delimiter=',',
               header='x,y,z (meter)', comments='')

    r = np.linalg.norm(P, axis=1)
    print('      点云 %d 个，XYZ 范围：' % len(P))
    for ax, nm in enumerate('XYZ'):
        print('        %s: [%6.3f, %6.3f] m' % (nm, P[:, ax].min(), P[:, ax].max()))
    print('      离基座距离：%.3f ~ %.3f m（说明书标称工作半径 0.627 m）' % (r.min(), r.max()))

    # ---- 2) 灵巧度
    dex_rows = []
    if not args.no_dex:
        print('\n[2/3] 灵巧度评估：%d 个候选点 × 每点 %d 个随机姿态 ...' % (args.dex_points, args.dex_ori))
        cand, qcand = thin_to_grid(P, Q, args.grid)
        if len(cand) > args.dex_points:
            sel = rng.choice(len(cand), args.dex_points, replace=False)
            cand, qcand = cand[sel], qcand[sel]
        print('      候选点 %d 个（体素 %.2f m 稀释后）' % (len(cand), args.grid))
        for i, p in enumerate(cand):
            ok = 0
            for _ in range(args.dex_ori):
                R_t = random_rotation(rng)
                success = False
                for r_ in range(args.dex_restarts):
                    if r_ == 0:
                        # 种子：采样时真正到过这个格子附近的姿态（收敛率远高于随机）
                        q0 = qcand[i] + rng.normal(0, 0.10, kin.ndof)
                    else:
                        q0 = rng.uniform(kin.lo, kin.hi)
                    q0 = q0 + np.where(np.abs(q0) < 1e-3, 0.05, 0.0)   # 躲开零位奇异
                    _, okk = kin.ik_dls(p, R_t, np.clip(q0, kin.lo, kin.hi), iters=args.dex_iters)
                    if okk:
                        success = True
                        break
                ok += int(success)
            dex_rows.append((p[0], p[1], p[2], ok / args.dex_ori))
            if (i + 1) % 25 == 0:
                print('        已评 %d/%d' % (i + 1, len(cand)), flush=True)
        dex = np.array(dex_rows)
        np.savetxt(os.path.join(args.out, 'dexterity.csv'), dex, delimiter=',',
                   header='x,y,z,dexterity_score', comments='')
        n_dex = int((dex[:, 3] >= 0.999).sum())
        print('      灵巧点（全部姿态可达）%d / %d = %.1f%%' % (n_dex, len(dex), 100.0 * n_dex / len(dex)))
        print('      ⚠ score 是"下界"：IK 没解出来 ≠ 一定到不了（数值解可能卡在局部）。')
        print('        想更可信就加大 --dex-restarts / --dex-iters，或用几何 IK 交叉验证。')
    else:
        print('\n[2/3] 跳过灵巧度（--no-dex）')

    # ---- 3) 出图
    print('\n[3/3] 出图 ...')
    try:
        import matplotlib
        matplotlib.use('Agg')
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 注册 '3d' 投影
        import matplotlib.pyplot as plt

        sub = P if len(P) <= 30000 else P[rng.choice(len(P), 30000, replace=False)]

        fig = plt.figure(figsize=(7.5, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(sub[:, 0], sub[:, 1], sub[:, 2], s=1, alpha=0.15, c='#378ADD')
        ax.scatter(*GOLDEN_ZERO_POS, s=40, c='#E24B4A', marker='o', label='zero pose')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title('Reachable workspace (N=%d)' % args.n)
        ax.legend()
        fig.tight_layout(); fig.savefig(os.path.join(args.out, 'reachable_3d.png'), dpi=130)
        plt.close(fig)

        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        for a, (i, j, nm) in zip(axes, [(0, 1, 'X-Y'), (0, 2, 'X-Z'), (1, 2, 'Y-Z')]):
            a.scatter(sub[:, i], sub[:, j], s=1, alpha=0.15, c='#378ADD')
            a.set_xlabel(nm.split('-')[0] + ' (m)'); a.set_ylabel(nm.split('-')[1] + ' (m)')
            a.set_title(nm + ' projection'); a.set_aspect('equal')
        fig.tight_layout(); fig.savefig(os.path.join(args.out, 'reachable_views.png'), dpi=130)
        plt.close(fig)

        if dex_rows:
            dex = np.array(dex_rows)
            fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
            sc = axes[0].scatter(dex[:, 0], dex[:, 1], c=dex[:, 3], s=26, cmap='viridis', vmin=0, vmax=1)
            axes[0].set_xlabel('X (m)'); axes[0].set_ylabel('Y (m)')
            axes[0].set_title('Dexterity map (X-Y, all Z)'); axes[0].set_aspect('equal')
            plt.colorbar(sc, ax=axes[0], label='IK success rate')
            sc2 = axes[1].scatter(dex[:, 0], dex[:, 2], c=dex[:, 3], s=26, cmap='viridis', vmin=0, vmax=1)
            axes[1].set_xlabel('X (m)'); axes[1].set_ylabel('Z (m)')
            axes[1].set_title('Dexterity map (X-Z, all Y)'); axes[1].set_aspect('equal')
            plt.colorbar(sc2, ax=axes[1], label='IK success rate')
            fig.tight_layout(); fig.savefig(os.path.join(args.out, 'dexterity_slice.png'), dpi=130)
            plt.close(fig)
        print('      图已存到 %s' % args.out)
    except ImportError:
        print('      ⚠ 没装 matplotlib，跳过出图（数据已存 CSV/NPY）')
        print('        装它：sudo apt install -y python3-matplotlib')

    # ---- 统计落盘
    with open(os.path.join(args.out, 'stats.txt'), 'w') as f:
        f.write('samples: %d\n' % args.n)
        f.write('zero FK: %.6f %.6f %.6f (golden %.3f %.3f %.3f, dev %.2f mm)\n'
                % (*p_zero, *GOLDEN_ZERO_POS, d * 1000))
        for ax_, nm in enumerate('XYZ'):
            f.write('%s range: %.4f ~ %.4f\n' % (nm, P[:, ax_].min(), P[:, ax_].max()))
        f.write('radius: %.4f ~ %.4f\n' % (r.min(), r.max()))
        if dex_rows:
            dex = np.array(dex_rows)
            f.write('dexterity points: %d, full-dexterous: %d (%.1f%%)\n'
                    % (len(dex), int((dex[:, 3] >= 0.999).sum()),
                       100.0 * (dex[:, 3] >= 0.999).sum() / len(dex)))

    print('\n完成。结果目录：%s' % args.out)
    print('一句话：点云=可达空间，dexterity.csv 里 score=1.0 的点连起来≈灵巧空间。')


if __name__ == '__main__':
    main()

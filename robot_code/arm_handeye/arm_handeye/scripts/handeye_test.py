#!/usr/bin/env python3
"""handeye_test.py v2 —— 手眼标定 AX=XB（Tsai-Lenz 修正版）+ 双模式自测

v1 bug 修复：修正罗德里格斯参数 P 必须用 tan(θ/2)·n，
   v1 误用 2sin(θ/2)·n，导致旋转解差一个 2cos(θ/2) 因子（轴对、角错）。

两种模式：
  eye_in_hand（默认，相机装腕部，标定板固定在工作台）：
      A_rel = A_j⁻¹·A_i，X = 相机相对末端（hand→eye）
  eye_to_hand（相机固定，标定板装末端）：
      A_rel = A_j·A_i⁻¹，X = 相机相对基座（eye→base）

自测：已知 X_true 反造数据，求解对比，误差应 < 1e-6
"""
import math
import numpy as np

def skew(v):
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def rot_to_axis_angle(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    s = math.sin(ang)
    if abs(s) < 1e-6:
        if abs(ang) < 1e-12:
            return np.zeros(3), 0.0
        W = R + np.eye(3)
        i = int(np.argmax(np.diag(W)))
        n = W[:, i]
        return (n / np.linalg.norm(n) if np.linalg.norm(n) > 1e-12 else n), ang
    n = np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / (2*s)
    return (n / np.linalg.norm(n) if np.linalg.norm(n) > 1e-12 else n), ang

def axis_angle_to_rot(n, ang):
    nn = np.linalg.norm(n)
    if nn < 1e-15:
        return np.eye(3)
    n = n / nn
    K = skew(n)
    return np.eye(3) + math.sin(ang)*K + (1 - math.cos(ang))*(K @ K)

def rand_pose(rng, t_scale=0.5, ang_max=1.2):
    """随机位姿；角度限制在 <π/2 附近，保证相对旋转角 <π（tan 参数的安全区）"""
    n = rng.normal(size=3); n /= np.linalg.norm(n)
    T = np.eye(4)
    T[:3, :3] = axis_angle_to_rot(n, rng.uniform(0.3, ang_max))
    T[:3, 3] = rng.uniform(-t_scale, t_scale, 3)
    return T

def solve_handeye(A_list, B_list):
    """Tsai-Lenz 两步法（tan 形式）：解 A·X = X·B"""
    M, b = [], []
    for A, B in zip(A_list, B_list):
        nA, thA = rot_to_axis_angle(A[:3, :3])
        nB, thB = rot_to_axis_angle(B[:3, :3])
        PA = math.tan(thA / 2) * nA            # ← v1 的 bug 在这里：写成了 2*sin(θ/2)
        PB = math.tan(thB / 2) * nB
        M.append(skew(PA + PB))
        b.append(PB - PA)
    PX, *_ = np.linalg.lstsq(np.vstack(M), np.concatenate(b), rcond=None)
    s = np.linalg.norm(PX)
    if s < 1e-12:
        RX = np.eye(3)
    else:
        RX = axis_angle_to_rot(PX / s, 2 * math.atan(s))   # θ = 2·atan(|P|)
    M2, b2 = [], []
    for A, B in zip(A_list, B_list):
        M2.append(A[:3, :3] - np.eye(3))
        b2.append(RX @ B[:3, 3] - A[:3, 3])
    tX, *_ = np.linalg.lstsq(np.vstack(M2), np.concatenate(b2), rcond=None)
    X = np.eye(4)
    X[:3, :3] = RX
    X[:3, 3] = tX
    return X

def make_data(mode, seed=0, n_pose=6):
    """合成数据：按模式构造 A_rel / B_rel，返回它们和 X_true"""
    rng = np.random.default_rng(seed)
    A_abs = [rand_pose(rng, 0.6) for _ in range(n_pose)]        # 末端在基座系
    if mode == 'eye_in_hand':
        X_true = rand_pose(rng, 0.15)                           # 相机相对末端
        C = rand_pose(rng, 0.8)                                 # 标定板固定在基座系
        B_abs = [np.linalg.inv(A @ X_true) @ C for A in A_abs]  # 板在相机系
    else:
        X_true = rand_pose(rng, 0.8)                            # 相机相对基座
        Z = rand_pose(rng, 0.1)                                 # 板相对末端
        B_abs = [np.linalg.inv(X_true) @ A @ Z for A in A_abs]
    A_rel, B_rel = [], []
    for k in range(1, n_pose):
        if mode == 'eye_in_hand':
            A_rel.append(np.linalg.inv(A_abs[k]) @ A_abs[0])
        else:
            A_rel.append(A_abs[k] @ np.linalg.inv(A_abs[0]))
        B_rel.append(B_abs[k] @ np.linalg.inv(B_abs[0]))
    return A_rel, B_rel, X_true

def self_test(mode, seed=0, n_pose=6):
    A_rel, B_rel, X_true = make_data(mode, seed, n_pose)
    X_est = solve_handeye(A_rel, B_rel)
    pos_err = float(np.linalg.norm(X_est[:3, 3] - X_true[:3, 3]))
    rot_err = abs(rot_to_axis_angle(X_est[:3, :3] @ X_true[:3, :3].T)[1])
    ok = pos_err < 1e-6 and rot_err < 1e-6
    print(f'  平移误差 {pos_err:.3e} m    旋转误差 {rot_err:.3e} rad    {"通过 ✅" if ok else "未通过 ❌"}')
    return ok

def main():
    print('=== 手眼标定 AX=XB 自测（Tsai-Lenz tan 形式，修正版）===')
    for mode in ('eye_in_hand', 'eye_to_hand'):
        print(f'--- 模式: {mode} ---')
        results = [self_test(mode, seed=s) for s in range(5)]
        print(f'  小计：{sum(results)}/5 通过')

if __name__ == '__main__':
    main()

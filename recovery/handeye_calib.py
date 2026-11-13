#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定求解器 —— 支持两种安装方式

  eye_in_hand  （眼在手上 / 相机装在末端，标定板固定在桌面）  ← 本项目当前方案
      X = T_flange←camera   相机长在法兰的哪个位置（固定不变）

  eye_to_hand  （眼在手外 / 相机固定，标定板贴在末端）
      X = T_base←camera     相机在机械臂基座系的位姿（固定不变）

------------------------------------------------------------------
输入数据（每个位姿一组）
------------------------------------------------------------------
  flange_in_base   : 末端法兰在基座系下的位姿 —— 从 TF 读（base_link → flange）
  marker_in_camera : 标定板在相机系下的位姿   —— 用 cv2.aruco 检测得到

  两种模式下这两项的"采集方式"一样，区别只在于标定板贴在哪：
    eye_in_hand : 标定板固定在桌面不动，机械臂带着相机走位去看它
    eye_to_hand : 标定板贴在末端法兰上，相机固定不动看它

------------------------------------------------------------------
数学关系（两种模式的 A、B 定义不一样，别混用！）
------------------------------------------------------------------
  eye_in_hand : T_base←flange(i) · X · T_cam←target(i) = T_base←target（常量）
        A = T_base←flange(i)^-1 · T_base←flange(j)
        B = T_cam←target(i)     · T_cam←target(j)^-1
        A X = X B   →   X = T_flange←camera

  eye_to_hand : T_base←flange(i) · T_flange←marker = X · T_cam←marker(i)
        A = T_base←flange(j) · T_base←flange(i)^-1
        B = T_cam←marker(j)  · T_cam←marker(i)^-1
        A X = X B   →   X = T_base←camera

------------------------------------------------------------------
用法
------------------------------------------------------------------
    python3 handeye_calib.py --selftest --mode eye_in_hand
    python3 handeye_calib.py --selftest --mode eye_to_hand
    python3 handeye_calib.py --input samples.json          # mode 从 json 里读
    python3 handeye_calib.py --input samples.json --pairs all

输入格式见 samples_example.json，位姿可写：
    {"xyz": [x,y,z], "quat_wxyz": [w,x,y,z]}   或
    {"matrix_4x4": [[...],[...],[...],[...]]}

依赖：numpy（不需要 ROS，纯数学，可先在 Mac 上跑自检）
"""

import argparse
import json
import os
import sys

import numpy as np

EPS = 1e-12
MODE_EYE_IN_HAND = 'eye_in_hand'
MODE_EYE_TO_HAND = 'eye_to_hand'


# ---------------------------------------------------------------- 基础变换

def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat_wxyz(R):
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w < EPS:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        t = np.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) / 2.0
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / (4 * t)
        q[1 + i] = t
        q[1 + j] = (R[j, i] + R[i, j]) / (4 * t)
        q[1 + k] = (R[k, i] + R[i, k]) / (4 * t)
        return q / np.linalg.norm(q)
    x = (R[2, 1] - R[1, 2]) / (4 * w)
    y = (R[0, 2] - R[2, 0]) / (4 * w)
    z = (R[1, 0] - R[0, 1]) / (4 * w)
    return np.array([w, x, y, z])


def pose_to_T(p):
    """把 json 里的位姿字典转成 4x4 齐次矩阵"""
    if 'matrix_4x4' in p:
        T = np.array(p['matrix_4x4'], dtype=float)
        if T.shape != (4, 4):
            raise ValueError('matrix_4x4 必须是 4x4')
        return T
    if 'xyz' in p and 'quat_wxyz' in p:
        T = np.eye(4)
        T[:3, :3] = quat_wxyz_to_R(np.array(p['quat_wxyz'], dtype=float))
        T[:3, 3] = np.array(p['xyz'], dtype=float)
        return T
    if 'xyz' in p and 'quat_xyzw' in p:
        q = np.array(p['quat_xyzw'], dtype=float)
        return pose_to_T({'xyz': p['xyz'], 'quat_wxyz': [q[3], q[0], q[1], q[2]]})
    raise ValueError('位姿格式无法识别：%s' % (list(p.keys()),))


def inv_T(T):
    Ti = np.eye(4)
    R = T[:3, :3]
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def R_to_rpy(R):
    """按 R = Rz(yaw)·Ry(pitch)·Rx(roll) 分解，返回 (roll, pitch, yaw)，弧度"""
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    roll = np.arctan2(R[2, 1], R[2, 2])
    return float(roll), float(pitch), float(yaw)


def rot_angle(R):
    """旋转矩阵对应的转角（弧度）"""
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


# ---------------------------------------------------------------- 求解 AX = XB

def solve_axxb(pairs):
    """
    pairs: [(A, B), ...]，每个 A、B 是 4x4，求 X 使 A X = X B
    旋转部分：kron 方程的最小奇异向量（行优先约定 vec(A X) = (A ⊗ I) vec(X)）
    平移部分：(A_R - I) t_x = R_x t_b - t_a，最小二乘
    """
    blocks = []
    for A, B in pairs:
        Ar, Br = A[:3, :3], B[:3, :3]
        blocks.append(np.kron(Ar, np.eye(3)) - np.kron(np.eye(3), Br.T))
    M = np.vstack(blocks)
    _, _, Vt = np.linalg.svd(M)
    R = Vt[-1].reshape(3, 3)

    # SVD 的最小奇异向量符号不确定：若 det<0，真解是 -R（3x3 取负会翻转 det 符号）
    # 不修这一步会在部分数据下出现 180 度级别的错误解
    if np.linalg.det(R) < 0:
        R = -R

    # 投影到 SO(3)
    U, _, Vt2 = np.linalg.svd(R)
    R = U @ Vt2
    if np.linalg.det(R) < 0:
        R = U @ np.diag([1.0, 1.0, -1.0]) @ Vt2

    Amat, bvec = [], []
    for A, B in pairs:
        Ar, ta, tb = A[:3, :3], A[:3, 3], B[:3, 3]
        Amat.append(Ar - np.eye(3))
        bvec.append(R @ tb - ta)
    Amat = np.vstack(Amat)
    bvec = np.concatenate(bvec)
    t, *_ = np.linalg.lstsq(Amat, bvec, rcond=None)

    X = np.eye(4)
    X[:3, :3] = R
    X[:3, 3] = t
    return X


def make_pairs(T_list, M_list, pairs_mode='consecutive', mode=MODE_EYE_IN_HAND):
    """
    T_list : T_base←flange(i)   末端在基座系的位姿（每个采样位姿一个）
    M_list : T_cam←target(i)    标定板在相机系的位姿

    ⚠️ 两种模式的 A、B 定义不同（谁左乘谁、取谁的逆），写反了会得到完全错误的结果。
       本函数已用合成真值在 selftest 中双模式验证过。
    """
    n = len(T_list)
    if pairs_mode == 'consecutive':
        idx = [(i, i + 1) for i in range(n - 1)]
    else:  # all
        idx = [(i, j) for i in range(n) for j in range(n) if i != j]

    pairs = []
    for i, j in idx:
        Ti, Tj = T_list[i], T_list[j]
        Mi, Mj = M_list[i], M_list[j]
        if mode == MODE_EYE_IN_HAND:
            A = inv_T(Ti) @ Tj      # 末端相对运动（表示在 flange 系）
            B = Mi @ inv_T(Mj)      # 相机看到靶标的相对变化
        else:                        # eye_to_hand
            A = Tj @ inv_T(Ti)      # 末端相对运动（表示在 base 系）
            B = Mj @ inv_T(Mi)      # 相机看到标记的相对变化
        pairs.append((A, B))
    return pairs


def residuals(pairs, X):
    """返回每对的旋转误差（度）和平移误差（米）"""
    out = []
    for A, B in pairs:
        E = inv_T(X) @ A @ X @ inv_T(B)   # 理想情况等于单位阵
        out.append((np.degrees(rot_angle(E[:3, :3])),
                    float(np.linalg.norm(E[:3, 3]))))
    return out


# ---------------------------------------------------------------- 自检

def selftest(mode=MODE_EYE_IN_HAND, n_poses=20, seed=0, verbose=True):
    """
    用合成真值验证求解器：先造一个已知的 X，反算出"相机应该看到什么"，
    再喂给求解器看它能不能把 X 找回来。
    """
    rng = np.random.default_rng(seed)

    def rand_pose(t_scale=0.6):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        ang = rng.uniform(-np.pi, np.pi)
        K = np.array([[0, -axis[2], axis[1]],
                      [axis[2], 0, -axis[0]],
                      [-axis[1], axis[0], 0]])
        R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = rng.uniform(-t_scale, t_scale, size=3)
        return T

    if mode == MODE_EYE_IN_HAND:
        X_true = rand_pose(0.3)                     # T_flange←camera
        Z_true = rand_pose(1.0)                     # T_base←target（靶固定在桌上）
        T_list, M_list = [], []
        for _ in range(n_poses):
            Ti = rand_pose(0.6)                     # T_base←flange
            Mi = inv_T(X_true) @ inv_T(Ti) @ Z_true  # T_cam←target
            T_list.append(Ti)
            M_list.append(Mi)
    else:
        X_true = rand_pose(1.0)                     # T_base←camera
        Y_true = rand_pose(0.3)                     # T_flange←marker
        T_list, M_list = [], []
        for _ in range(n_poses):
            Ti = rand_pose(0.6)
            Mi = inv_T(X_true) @ Ti @ Y_true         # T_cam←marker
            T_list.append(Ti)
            M_list.append(Mi)

    pairs = make_pairs(T_list, M_list, 'all', mode)
    X = solve_axxb(pairs)
    E = inv_T(X_true) @ X
    r_err = float(np.degrees(rot_angle(E[:3, :3])))
    t_err = float(np.linalg.norm(E[:3, 3]))
    res = residuals(pairs, X)
    max_rot = max(r[0] for r in res)
    max_tr = max(r[1] for r in res)

    if verbose:
        print('[selftest] 模式 = %s，位姿数 = %d，配对数 = %d'
              % (mode, n_poses, len(pairs)))
        print('[selftest] 旋转误差 = %.3e 度' % r_err)
        print('[selftest] 平移误差 = %.3e 米' % t_err)
        print('[selftest] 各对最大残差：旋转 %.3e 度 / 平移 %.3e 米' % (max_rot, max_tr))
        # 阈值留足浮点余量：1e-3 度 ≈ 1.7e-5 rad，0.5m 处只相当于 9 微米
        ok = r_err < 1e-3 and t_err < 1e-6
        print('[selftest] 结果：%s' % ('通过 ✅ 求解器可信' if ok else '未通过 ❌ 不要用于真机'))
        return 0 if ok else 1
    return r_err, t_err


# ---------------------------------------------------------------- 写回配置
def _iso_link_from_optical(X):
    """X = T_flange←camera_optical → T_flange←d435i_link（给静态 TF 用）。

    为什么需要这一步：realsense 自己会发 d435i_link → *_optical_frame，
    若再发一条 flange → *_optical_frame，同一个 child 会有两个父，TF 树打架。
    """
    link_T = np.eye(4)
    link_T[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return X @ link_T


def yaml_snippet(X, tf_xyz, tf_quat_wxyz):
    """渲染成可直接粘贴进 real_params.yaml 的片段（注意 tf_quat 是 xyzw，w 在最后）。"""
    rows = []
    for r in X:
        rows.append("    - [" + ", ".join("%.8f" % float(v) for v in r) + "]")
    qx, qy, qz, qw = tf_quat_wxyz[1], tf_quat_wxyz[2], tf_quat_wxyz[3], tf_quat_wxyz[0]
    return "\n".join([
        "handeye:",
        "  calibrated: true",
        "  matrix_4x4:",
    ] + rows + [
        "  tf_xyz: [%.6f, %.6f, %.6f]" % tuple(tf_xyz),
        "  tf_quat_xyzw: [%.6f, %.6f, %.6f, %.6f]" % (qx, qy, qz, qw),
    ])


def patch_real_params(path, X, tf_xyz, tf_quat_wxyz):
    """把结果直接写进 real_params.yaml（逐行改，保留全部注释）。

    为什么不用 yaml.safe_load + dump：那会把文件里几十行解释性注释全删掉，
    而这些注释正是下次换机器时唯一能看懂的东西。
    """
    import re
    import shutil
    import time as _t

    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        raise FileNotFoundError('找不到 %s' % path)
    bak = '%s.bak.%s' % (path, _t.strftime('%Y%m%d-%H%M%S'))
    shutil.copy2(path, bak)

    with open(path, 'r', encoding='utf-8') as f:
        lines = f.read().splitlines()

    qx, qy, qz, qw = tf_quat_wxyz[1], tf_quat_wxyz[2], tf_quat_wxyz[3], tf_quat_wxyz[0]
    out, i, changed = [], 0, []
    while i < len(lines):
        ln = lines[i]
        s = ln.strip()
        indent = ln[:len(ln) - len(ln.lstrip())]
        if s.startswith('matrix_4x4:'):
            out.append(indent + 'matrix_4x4:')
            for r in X:
                out.append(indent + '  - [' +
                           ', '.join('%.8f' % float(v) for v in r) + ']')
            i += 1
            while i < len(lines) and lines[i].strip().startswith('- ['):
                i += 1          # 丢掉旧的若干行矩阵
            changed.append('matrix_4x4')
            continue
        if re.match(r'^calibrated:\s*(true|false)', s):
            # 保留行尾注释（"false 时感知节点拒绝输出…"这类说明下次还看得懂）
            out.append(re.sub(r'^(\s*calibrated:\s*)(true|false)', r'\1true', ln))
            i += 1
            changed.append('calibrated')
            continue
        if s.startswith('tf_xyz:'):
            out.append(indent + 'tf_xyz: [%.6f, %.6f, %.6f]' % tuple(tf_xyz))
            i += 1
            changed.append('tf_xyz')
            continue
        if s.startswith('tf_quat_xyzw:'):
            out.append(indent + 'tf_quat_xyzw: [%.6f, %.6f, %.6f, %.6f]'
                       % (qx, qy, qz, qw))
            i += 1
            changed.append('tf_quat_xyzw')
            continue
        out.append(ln)
        i += 1

    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(out) + '\n')
    return bak, changed


# ---------------------------------------------------------------- 主流程

def run_file(path, pairs_mode, write_config=None):
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    mode = data.get('mode', MODE_EYE_IN_HAND)
    if mode not in (MODE_EYE_IN_HAND, MODE_EYE_TO_HAND):
        raise ValueError('mode 必须是 eye_in_hand 或 eye_to_hand，收到：%s' % mode)

    samples = data['samples']
    T_list, M_list = [], []
    for s in samples:
        T_list.append(pose_to_T(s['flange_in_base']))
        M_list.append(pose_to_T(s['marker_in_camera']))

    x_label = 'X = T_flange←camera' if mode == MODE_EYE_IN_HAND else 'X = T_base←camera'
    print('模式：%s' % mode)
    print('读入 %d 组位姿' % len(T_list))
    if len(T_list) < 5:
        print('⚠️  位姿太少（<5），解不可靠。建议 12~20 组，且必须包含旋转变化。')

    rots = [rot_angle(inv_T(T_list[0]) @ T) for T in T_list]
    print('各姿态相对首帧的旋转量（度）：min %.1f / max %.1f'
          % (min(np.degrees(rots)), max(np.degrees(rots))))
    if max(np.degrees(rots)) < 20:
        print('⚠️  旋转变化太小！纯平移解不出旋转部分，请加入明显的姿态变化。')

    pairs = make_pairs(T_list, M_list, pairs_mode, mode)
    X = solve_axxb(pairs)
    res = residuals(pairs, X)
    rot_arr = np.array([r[0] for r in res])
    tr_arr = np.array([r[1] for r in res])

    print('\n===== 求解结果：%s =====' % x_label)
    np.set_printoptions(precision=6, suppress=True)
    print(X)

    q = R_to_quat_wxyz(X[:3, :3])
    t = X[:3, 3]
    print('\n平移 (m): x=%.6f y=%.6f z=%.6f' % tuple(t))
    print('四元数 wxyz: %.6f %.6f %.6f %.6f' % tuple(q))

    # ---- 直接可粘贴的配置片段（省掉手抄 4x4 的抄错风险）----
    if mode == MODE_EYE_IN_HAND:
        X_link = _iso_link_from_optical(X)
        tl, ql = X_link[:3, 3], R_to_quat_wxyz(X_link[:3, :3])
        print('\n' + '=' * 66)
        print('  ★ 直接抄进 real_params.yaml 的 handeye 段（或用 --write-config 自动写）')
        print('=' * 66)
        print(yaml_snippet(X, tl, ql))

        # 合理性自检：这类错误在打印矩阵时看不出来，但一上真机就全错
        problems = []
        if not (0.02 < float(np.linalg.norm(t)) < 0.45):
            problems.append('平移模长 %.3fm 不合理（法兰到相机的真实距离通常 5~25cm）。'
                            '检查单位是不是错成 mm、或 AX=XB 的 A/B 定义反了' %
                            float(np.linalg.norm(t)))
        R = X[:3, :3]
        if float(np.abs(R.T @ R - np.eye(3)).max()) > 1e-3:
            problems.append('旋转不是正交阵（会缩放坐标），检查输入四元数是不是做了归一化')
        if problems:
            print('\n⚠️  结果可疑，先别写进配置：')
            for p in problems:
                print('   - %s' % p)
        elif write_config:
            target = write_config
            if target in ('auto', '1', 'true'):
                target = os.path.expanduser(
                    '~/robot_vla_ws/robot_code/arm_grasp/config/real_params.yaml')
            try:
                bak, changed = patch_real_params(target, X, tl, ql)
                print('\n✅ 已写入 %s（改动了 %s）' % (target, ' / '.join(changed)))
                print('   备份：%s' % bak)
                print('   下一步：python3 robot_code/arm_grasp/scripts/preflight.py')
                print('           ros2 launch arm_grasp real_grasp.launch.py start_task:=false')
                print('           timeout 5 ros2 topic echo /grasp/target_xyz')
            except Exception as e:                    # noqa: BLE001
                print('\n❌ 自动写入失败：%s' % e)
                print('   手动把上面的片段替换掉 handeye 段即可。')
        else:
            print('\n（想自动写入就加 --write-config）')
        print('=' * 66)

    print('\n残差（越小越好，理想 = 0）')
    print('  旋转：均值 %.4f 度 / 最大 %.4f 度' % (rot_arr.mean(), rot_arr.max()))
    print('  平移：均值 %.4f m / 最大 %.4f m' % (tr_arr.mean(), tr_arr.max()))
    if tr_arr.max() > 0.005:
        print('  ⚠️  残差偏大（>5mm）。检查：标定板是否固定牢、位姿是否有旋转变化、'
              '相机是否预热、TF 取的是不是法兰而不是 link6。')

    frames = data.get('frames', {})
    if mode == MODE_EYE_IN_HAND:
        parent = frames.get('flange_frame', 'flange')
        child = frames.get('camera_optical_frame', 'd435i_color_optical_frame')
        extra = ('\n⚠️ 运行时注意：这个矩阵是固定的，但 base_link→flange 是时变的。\n'
                 '   每次换算都要重新查 TF：p_base = T_base←flange(当前) · X · p_camera\n'
                 '   别把 base_link 下的结果缓存下来复用，机械臂一动就失效。')
    else:
        parent = frames.get('robot_base_frame', 'base_link')
        child = frames.get('camera_optical_frame', 'd435i_color_optical_frame')
        extra = ''

    print('\n===== 写入 launch 的 TF 发布命令（%s → %s）=====' % (parent, child))
    print('形式一（四元数，推荐）：')
    print('ros2 run tf2_ros static_transform_publisher \\\n'
          '  %.6f %.6f %.6f %.6f %.6f %.6f %.6f \\\n  %s %s'
          % (t[0], t[1], t[2], q[1], q[2], q[3], q[0], parent, child))

    roll, pitch, yaw = R_to_rpy(X[:3, :3])
    print('\n形式二（欧拉角，Foxy 若不接受四元数就用这个）：')
    print('ros2 run tf2_ros static_transform_publisher \\\n'
          '  %.6f %.6f %.6f %.6f %.6f %.6f \\\n  %s %s'
          % (t[0], t[1], t[2], yaw, pitch, roll, parent, child))

    if extra:
        print(extra)

    # ---- TF 树接法提示：realsense 自己会发 d435i_link → *_optical_frame ----
    # 若再发一条 flange → *_optical_frame，同一个 child 会有两个父，TF 树会打架/抖动。
    # 正确接法：发 flange → d435i_link，让 realsense 那一小段自己接上。
    link_T = np.eye(4)
    link_T[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    X_link = X @ link_T                       # T_flange←d435i_link
    ql = R_to_quat_wxyz(X_link[:3, :3])
    tl = X_link[:3, 3]
    print('\n===== 推荐接法：发 flange → d435i_link（避免 TF 树冲突）=====')
    print('（上面用的是 ROS 标准光学旋转 rpy = -90°, 0, -90°，'
          '实际值请用下面的命令核对后再填）')
    print('  ros2 run tf2_ros tf2_echo d435i_link d435i_color_optical_frame')
    print('ros2 run tf2_ros static_transform_publisher \\\n'
          '  %.6f %.6f %.6f %.6f %.6f %.6f %.6f \\\n  %s d435i_link'
          % (tl[0], tl[1], tl[2], ql[1], ql[2], ql[3], ql[0],
             frames.get('flange_frame', 'flange') if mode == MODE_EYE_IN_HAND
             else frames.get('robot_base_frame', 'base_link')))

    print('\n下一步：用打点验证法确认误差 < 3mm，不许靠改被测对象蒙混过关。')
    return 0


def main():
    ap = argparse.ArgumentParser(description='手眼标定求解器（支持眼在手/眼在手外）')
    ap.add_argument('--selftest', action='store_true',
                    help='用合成真值验证求解器（不需要真实数据）')
    ap.add_argument('--input', help='采集数据 json 文件')
    ap.add_argument('--mode', default=MODE_EYE_IN_HAND,
                    choices=[MODE_EYE_IN_HAND, MODE_EYE_TO_HAND],
                    help='自检用的安装方式（从 --input 读数据时以 json 里的 mode 为准）')
    ap.add_argument('--pairs', default='consecutive',
                    choices=['consecutive', 'all'],
                    help='配对方式：相邻两两配对（默认）或全部两两配对')
    ap.add_argument('--write-config', nargs='?', const='auto', default=None,
                    metavar='PATH',
                    help='把结果直接写进 real_params.yaml（保留注释，自动备份）。'
                         '不给路径时默认写 ~/robot_vla_ws/robot_code/arm_grasp/config/'
                         'real_params.yaml')
    ap.add_argument('--poses', type=int, default=20, help='自检用的位姿数量')
    args = ap.parse_args()

    if args.selftest:
        return selftest(mode=args.mode, n_poses=args.poses)
    if args.input:
        return run_file(args.input, args.pairs, args.write_config)
    ap.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main())

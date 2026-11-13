#!/usr/bin/env python3
"""grasp_v3.py —— 修正光学系约定后的完整抓取闭环

修正汇总：
  1) 关节限位用 URDF 真值（J6 实为 ±2.9656）
  2) 绕光轴搜滚转角，找全限位可行的 IK 解
  3) 点云是 ROS 光学系(z前/x右/y下)，转本体系: p_body = (p_z, -p_x, -p_y)
  4) 夹爪 TCP 是 Link6 沿工具轴 0.20m 的固定偏置（gripper_tcp 链有分叉，
     ik_check 的链式 FK 走不了）→ 统一用 TIP=Link6，目标点减去 R@offset
流程：感知定位 -> IK 预抓取/抓取/抬升 -> 执行 -> 回验
（夹爪闭合因 finger 关节未注册暂跳过）
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState, PointCloud2
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from threading import Thread

sys.path.insert(0, '/home/ubuntu/robot_vla_ws/src/arm_control/scripts')
from ik_check import UrdfKinematics, dls_ik          # noqa: E402

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ACTION = '/rm_group_controller/follow_joint_trajectory'
FINGERS = ['finger_left_joint', 'finger_right_joint']
GACTION = '/gripper_controller/follow_joint_trajectory'
LO = np.array([-3.1923, -2.61666, -2.61666, -2.61666, -2.96555, -2.96555])
HI = np.array([3.1923, 2.442, 2.61666, 2.61666, 2.96555, 2.96555])

WORK_AIM = np.array([0.40, 0.0, 0.02])
P_CAM    = np.array([0.20, 0.25, 0.30])
CAM_OFF  = np.array([0.070, 0.0, 0.035])  # 相机相对 Link6(法兰旁 +x 侧，厚 25mm)
# 相机相对 Link6 的【姿态】：rpy=(0, -1.24, pi)，把相机光轴(局部 x)转到 Link6 +z
# 并向夹爪轴线内倾 ~19°，光轴在 z=0.195 处正对两指尖。
# ⚠️ localize 里不能直接 CAM_OFF+p_body（那要求两系同姿态）
_c, _s = np.cos(-1.2915), np.sin(-1.2915)
_Ry = np.array([[_c, 0.0, _s], [0.0, 1.0, 0.0], [-_s, 0.0, _c]])
CAM_ROT  = np.diag([-1.0, -1.0, 1.0]) @ _Ry   # Rz(pi) @ Ry(-1.24)
# 相机光轴在 Link6 下的方向（Gazebo 相机沿 link +x 看），相对工具轴内倾 19°
CAM_AXIS = CAM_ROT @ np.array([1.0, 0.0, 0.0])
TCP_OFF  = np.array([0.0, 0.0, 0.195])  # 夹爪 TCP = 指尖，相对 Link6
BOX_HALF = 0.02      # 4cm 方块半边长（已知几何，非位置真值）
# 铰指角（弧度）：左指 +θ 张开、右指 -θ 张开
# 开口 = 0.032 + 0.05*(sin θ_L - sin θ_R)
#   θ=(0.35,-0.35) -> 66mm 张开；θ=(0.06,-0.06) -> 38mm，对 40mm 方块略过盈
GRIP_OPEN  = (0.20, -0.20)
GRIP_CLOSE = (-0.08, 0.08)   # box指：开口36mm，对42.1mm投影宽过盈3mm；kp=500->1.5N/侧
APPROACH = 0.10
LIFT     = 0.12


def orthonormal_basis(z_axis):
    """以给定方向作为【工具 z 轴】(夹爪纵轴/末端旋转轴) 构造正交基。
    注：真机夹爪沿末端旋转轴同轴延伸，故姿态约束是 z 轴指向目标方向。"""
    z = z_axis / np.linalg.norm(z_axis)
    up = np.array([0.0, 0.0, 1.0])
    if abs(z @ up) > 0.95:          # 接近竖直时换参考轴，避免除零
        up = np.array([1.0, 0.0, 0.0])
    x = up - z * (z @ up)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)              # 保证右手系 x × y = z
    return np.column_stack([x, y, z])


def rot_x(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_z(phi):
    """绕工具 z 轴(夹爪纵轴)滚转 —— 真机夹爪绕自身纵轴调整开合朝向"""
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def ik_with_roll(kin, q_init, p_tip_des, axis_dir, offset, n_try=13,
                 phi_center=None, phi_span=None):
    """光轴方向固定、绕光轴搜滚转；offset = 末端工具点相对 Link6 的固定偏置
    phi_center/phi_span：给定则只在上一解的滚转角附近搜，
    保证相邻路径点的 joint6 连续（否则会突然大跳，把仿真炸掉）"""
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    R_base = orthonormal_basis(axis_dir)
    if phi_center is None or phi_span is None:
        phis = np.linspace(-np.pi, np.pi, n_try + 1)[:-1]
    else:
        phis = np.linspace(phi_center - phi_span, phi_center + phi_span,
                           n_try)
    best = None
    for phi in phis:
        R = R_base @ rot_z(phi)
        flange = np.asarray(p_tip_des) - R @ offset
        q, ok, err = dls_ik(kin, q_init, flange, R)
        if not ok or err > 5e-3:
            continue
        if any(x < lo or x > hi for x, lo, hi in zip(q, LO, HI)):
            continue
        if best is None or np.max(np.abs(q)) < np.max(np.abs(best[0])):
            best = (q, R, phi)
    return best


def _rot_axis(axis, theta):
    """绕任意轴旋转（Rodrigues）"""
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(theta), np.sin(theta)
    C = 1.0 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]])


def ik_camera_aim(kin, q_init, p_cam, target, offset, cam_axis,
                  n_try=13, phi_center=None, phi_span=None):
    """让【相机光轴】正对目标，而不是让工具轴正对目标。

    相机装在法兰旁、光轴相对工具轴内倾（本机约 19°）。若沿用
    ik_with_roll（工具轴对准目标），画面会整体偏 19°，目标挤到视野边缘，
    点云中值出现 10~20mm 的系统性偏差（实测直接导致下降撞到方块）。

    做法：先求把 cam_axis 转到目标方向的基准姿态 R0，再绕目标方向搜滚转 θ，
    取全限位可行且关节角最小的解。返回值与 ik_with_roll 一致：(q, R6, θ)。
    """
    d = np.asarray(target, float) - np.asarray(p_cam, float)
    d = d / np.linalg.norm(d)
    a = np.asarray(cam_axis, float)
    a = a / np.linalg.norm(a)
    v = np.cross(a, d)
    c = float(np.dot(a, d))
    if c > 0.999999:
        R0 = np.eye(3)
    elif c < -0.999999:
        perp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        R0 = _rot_axis(np.cross(a, perp), np.pi)
    else:
        K = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
        R0 = np.eye(3) + K + np.outer(v, v) / (1.0 + c)
    if phi_center is None or phi_span is None:
        thetas = np.linspace(-np.pi, np.pi, n_try + 1)[:-1]
    else:
        thetas = np.linspace(phi_center - phi_span, phi_center + phi_span, n_try)
    best = None
    for th in thetas:
        R6 = _rot_axis(d, th) @ R0
        if np.linalg.norm(R6 @ a - d) > 1e-6:      # 数值自检：光轴确实对准目标
            continue
        t6 = np.asarray(p_cam, float) - R6 @ np.asarray(offset, float)
        q, ok, err = dls_ik(kin, q_init, t6, R6)
        if not ok or err > 5e-3:
            continue
        if any(x < lo or x > hi for x, lo, hi in zip(q, LO, HI)):
            continue
        if best is None or np.max(np.abs(q)) < np.max(np.abs(best[0])):
            best = (q, R6, th)
    return best


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'],
                   'formats': [np.float32] * 3,
                   'offsets': [off['x'], off['y'], off['z']],
                   'itemsize': msg.point_step})
    return np.frombuffer(msg.data, dtype=dt)


class G(Node):
    def __init__(self):
        super().__init__('grasp_v3')
        self.js = None; self.img = None; self.pts = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Image, '/wrist_depth_camera/image_raw',
                                 self.on_img, 10)
        self.create_subscription(PointCloud2, '/wrist_depth_camera/points',
                                 self.on_pts, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.gclient = ActionClient(self, FollowJointTrajectory, GACTION)

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        self.jsd = d
        self.jsv = dict(zip(m.name, m.velocity))
        if all(j in d for j in JOINTS):
            self.js = np.array([d[j] for j in JOINTS])

    def on_img(self, m): self.img = m
    def on_pts(self, m): self.pts = m

    def execute(self, wps, ts):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for q, t in zip(wps, ts):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)
        from threading import Event
        ev = Event(); holder = {}
        def _done(fut):
            holder['res'] = fut.result(); ev.set()
        self.client.send_goal_async(goal).add_done_callback(
            lambda g: g.result().get_result_async().add_done_callback(_done)
            if g.result().accepted else (holder.__setitem__('rejected', True),
                                         ev.set()))
        if not ev.wait(timeout=max(ts) + 30):
            return False, '超时'
        if holder.get('rejected'):
            return False, '被拒'
        return True, 'OK'

    def nan(self):
        """物理是否炸了（关节速度出现 NaN/inf）"""
        v = getattr(self, 'jsv', {})
        bad = [k for k, x in v.items() if not np.isfinite(x)]
        return bad

    def set_gripper(self, left, right, secs=1.5):
        """夹爪走 JointTrajectoryController(action)：发送 + 等到执行完成"""
        if not self.gclient.wait_for_server(timeout_sec=5.0):
            return '夹爪 action 未就绪'
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = FINGERS
        p = JointTrajectoryPoint()
        p.positions = [float(left), float(right)]
        p.time_from_start.sec = int(secs)
        p.time_from_start.nanosec = int((secs - int(secs)) * 1e9)
        goal.trajectory.points.append(p)
        from threading import Event
        ev = Event(); h = {}
        self.gclient.send_goal_async(goal).add_done_callback(
            lambda g: g.result().get_result_async().add_done_callback(
                lambda f: (h.__setitem__('r', f.result()), ev.set()))
            if g.result().accepted else (h.__setitem__('no', True), ev.set()))
        ok = ev.wait(timeout=secs + 15)
        time.sleep(0.5)
        fl, fr = self.fingers()
        fs = (f'L={fl:+.3f} R={fr:+.3f}' if fl is not None else '读数缺失')
        return ('OK' if (ok and not h.get('no')) else '失败') + ' ' + fs

    def fingers(self):
        d = getattr(self, 'jsd', {})
        return (d.get('finger_left_joint'), d.get('finger_right_joint'))


def localize(node, t6, R6, seconds=20):
    ests = []
    t_run = time.time()
    while time.time() - t_run < seconds:
        time.sleep(0.05)
        im, pc = node.img, node.pts
        if im is None or pc is None:
            continue
        img = np.frombuffer(im.data, np.uint8).reshape(
            im.height, im.width, -1)[:, :, :3]
        r = img[:, :, 0].astype(int); g = img[:, :, 1].astype(int)
        b = img[:, :, 2].astype(int)
        mask = (g > 100) & (g > r + 40) & (g > b + 40)
        if int(mask.sum()) < 20:
            continue
        cloud = cloud_xyz(pc)
        vs, us = np.where(mask)
        idxs = vs * im.width + us
        idxs = idxs[(idxs >= 0) & (idxs < len(cloud))]
        xs = cloud['x'][idxs].astype(float)
        ys = cloud['y'][idxs].astype(float)
        zs = cloud['z'][idxs].astype(float)
        fin = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        fin &= (xs*xs + ys*ys + zs*zs) > 1e-12
        if fin.sum() < 5:
            continue
        # 不用中值：相机内倾 ~19° 会同时看到顶面与侧面，中值会被侧面点带偏
        # （实测偏 x+13mm / y-17mm，直接导致下降撞到方块）。
        # 4cm 立方体几何已知，改为全部有效点转世界系后取稳健包围盒中心。
        pb = np.column_stack([zs[fin], -xs[fin], -ys[fin]])  # 光学系->本体系
        pw = pb @ (R6 @ CAM_ROT).T + (t6 + R6 @ CAM_OFF)
        z_top = np.percentile(pw[:, 2], 99)      # 顶面高度（几何已知，最稳）
        top = pw[pw[:, 2] > z_top - 0.004]       # 只保留顶面点
        if len(top) < 5:
            top = pw
        cx = 0.5 * (np.percentile(top[:, 0], 3) + np.percentile(top[:, 0], 97))
        cy = 0.5 * (np.percentile(top[:, 1], 3) + np.percentile(top[:, 1], 97))
        ests.append(np.array([cx, cy, z_top]))
        time.sleep(0.15)
    return np.array(ests) if ests else None


def main():
    kin = UrdfKinematics('/tmp/eco65_6fb.urdf', 'baselink', 'Link6')
    rclpy.init(); n = G()
    Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    if not n.client.wait_for_server(timeout_sec=20.0):
        print('action 未就绪'); return
    t0 = time.time()
    while time.time() - t0 < 40 and n.js is None:
        time.sleep(0.2)
    if n.js is None:
        print('无 joint_states'); return
    def chk(tag):
        b = n.nan()
        print('  [物理检查] %s: %s' % (tag, '正常' if not b else f'NaN! {b}'))

    q = n.js.copy()
    q_std = np.array([0.0, 0.2, 0.3, 0.0, 0.0, 0.0])
    print('回标准姿势:', n.execute([q, q_std], [0, 4])[1]); time.sleep(3)
    q = n.js.copy()
    # 先张开夹爪，避免下降过程碰方块
    print('夹爪张开:', n.set_gripper(*GRIP_OPEN)); time.sleep(2)

    # ---- 1) 观测姿势：相机光轴对准作业区 ----
    sol = ik_with_roll(kin, q, P_CAM, WORK_AIM - P_CAM, CAM_OFF)
    if sol is None:
        print('观测姿势无全限位解'); return
    q_view = sol[0]
    print('移动到观测姿势:', n.execute([q, q_view], [0, 5])[1])
    time.sleep(5)
    t0 = time.time()
    while time.time() - t0 < 45 and (n.img is None or n.pts is None):
        time.sleep(0.3)
    if n.img is None or n.pts is None:
        print('相机数据不足'); return

    # ---- 2) 感知定位（FK 提供相机位姿，不依赖 TF） ----
    q_act = n.js.copy() if n.js is not None else q_view
    T6 = kin.fk(q_act)
    t6, R6 = T6[:3, 3], T6[:3, :3]
    a = localize(n, t6, R6, 20)
    if a is None or len(a) < 3:
        print(f'感知失败 样本{0 if a is None else len(a)}'); return
    mean = a.mean(axis=0); std = a.std(axis=0)
    print(f'>>> 感知目标 [{mean[0]:.3f} {mean[1]:.3f} {mean[2]:.3f}] '
          f'样本{len(a)} std[{std[0]*100:.1f} {std[1]*100:.1f} '
          f'{std[2]*100:.1f}]cm')
    if std[0] > 0.02 or std[1] > 0.02:
        print('定位抖动过大，中止'); return
    bx, by, bz = mean
    bz_c = bz - BOX_HALF   # 相机看到的是方块顶面，TCP 要对准方块中心
    print(f'    方块中心(感知) [{bx:.3f} {by:.3f} {bz_c:.3f}]')

    # ---- 3) IK 规划（TCP 目标 + 工具竖直向下，滚转搜索） ----
    # 关节空间插值会让夹爪横扫过工作区撞飞方块 → 用笛卡尔直线下降
    down = np.array([0.0, 0.0, -1.0])
    top_t = np.array([bx, by, bz_c + 0.25])          # 方块正上方 25cm
    s_top = ik_with_roll(kin, q_act, top_t, down, TCP_OFF)
    if s_top is None:
        print('高点无全限位 TCP 解，中止'); return
    wps = [q_act, s_top[0]]
    ts = [0, 6]
    cur = s_top[0]
    phi = s_top[2]                                   # 顶部的滚转角
    N = 6
    for i in range(1, N + 1):                        # 直线下降到抓取点
        z = top_t[2] + (bz_c - top_t[2]) * i / N
        s = ik_with_roll(kin, cur, np.array([bx, by, z]), down, TCP_OFF,
                         n_try=7, phi_center=phi, phi_span=0.30)
        if s is None:                                # 附近搜不到再全局搜
            s = ik_with_roll(kin, cur, np.array([bx, by, z]), down, TCP_OFF)
        if s is None:
            print(f'下降第{i}步无解，中止'); return
        cur = s[0]; phi = s[2]
        wps.append(cur); ts.append(6 + 3 * i)
    q_grasp = cur
    # 抬升同样走笛卡尔直线（关节插值会横扫）
    lift_wps = [q_grasp]
    lift_ts = [0]
    c2 = q_grasp
    M = 4
    for i in range(1, M + 1):
        z = bz_c + LIFT * i / M
        s = ik_with_roll(kin, c2, np.array([bx, by, z]), down, TCP_OFF,
                         n_try=7, phi_center=phi, phi_span=0.30)
        if s is None:
            s = ik_with_roll(kin, c2, np.array([bx, by, z]), down, TCP_OFF)
        if s is None:
            print(f'抬升第{i}步无解，就地抬升中止'); break
        c2 = s[0]; phi = s[2]; lift_wps.append(c2); lift_ts.append(3 * i)
    q_lift = lift_wps[-1]
    print(f'路径点 {len(wps)} 个，直线下降 {N} 步，直线抬升 {len(lift_wps)-1} 步')
    print('  相邻路径点最大关节跳变(rad): ' + ' '.join(
        '%.2f' % np.max(np.abs(np.array(wps[i]) - np.array(wps[i - 1])))
        for i in range(1, len(wps))))
    print('  路径点与限位余量:')
    for i, w in enumerate(wps):
        marg = np.minimum(np.array(w) - LO, HI - np.array(w))
        print('   w%d %s  余量min=%.3f%s' %
              (i, np.round(w, 3), marg.min(),
               '  <-- 贴限位!' if marg.min() < 0.03 else ''))

    print('执行 高点->直线下降（逐段执行，便于定位）:')
    for i in range(1, len(wps)):
        dt = max(1.0, ts[i] - ts[i - 1])
        ok, msg = n.execute([wps[i - 1], wps[i]], [0, dt])
        print('  段%d/%d: %s' % (i, len(wps) - 1, msg))
        print('    实际 %s  目标 %s  手指 %s' %
              (np.round(n.js, 3), np.round(wps[i], 3),
               [None if x is None else round(x, 3) for x in n.fingers()]))
        b = n.nan()
        print('    [物理检查] 段%d: %s' % (i, '正常' if not b else f'NaN! {b}'))
        if b:
            print('物理在下降途中炸掉，中止'); return
    chk('下降到位')
    time.sleep(2)
    # 诊断开关：DEL_BLOCK=1 删除方块；=2 把方块挪到远处（保留世界里的接触体）
    import os
    import subprocess
    if os.environ.get('DEL_BLOCK') == '1':
        subprocess.run(['gz', 'model', '-m', 'target_box', '-d'],
                       capture_output=True, timeout=8)
        print('(诊断) 已删除方块，下面只测夹爪空合')
        time.sleep(1)
    elif os.environ.get('DEL_BLOCK') == '2':
        subprocess.run(['gz', 'model', '-m', 'target_box', '-d'],
                       capture_output=True, timeout=8)
        time.sleep(1)
        subprocess.run(['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-entity', 'target_box', '-file',
                        os.path.expanduser(
                            '~/robot_vla_ws/models/target_box/model.sdf'),
                        '-x', '0.65', '-y', '0.30', '-z', '0.0'],
                       capture_output=True, timeout=20)
        print('(诊断) 方块已挪到 (0.65, 0.30)，远离夹爪')
        time.sleep(2)

    # ① 先静置观察：确认"只是保持位姿"会不会自己炸
    print('--- 静置观察 20 秒（只保持，不动夹爪）---')
    for k in range(4):
        time.sleep(5)
        b = n.nan()
        print('   +%2ds 手指 %s  %s' %
              ((k + 1) * 5,
               [None if x is None else round(x, 3) for x in n.fingers()],
               '正常' if not b else 'NaN! ' + str(b)))
        if b:
            print('保持位姿就炸了，中止'); return

    # ② 一次性闭合到夹紧位，20Hz 采样抓 NaN 前后的帧
    samples = []
    stop = {'on': False}

    def sampler():
        while not stop['on']:
            d = getattr(n, 'jsd', {}); v = getattr(n, 'jsv', {})
            samples.append((time.time(),
                            d.get('finger_left_joint'), d.get('finger_right_joint'),
                            v.get('finger_left_joint'), v.get('finger_right_joint'),
                            d.get('joint2'), v.get('joint2')))
            time.sleep(0.05)

    Thread(target=sampler, daemon=True).start()
    print('一次性闭合到 %.2f' % GRIP_CLOSE[0])
    r = n.set_gripper(*GRIP_CLOSE, secs=3.0)
    print('闭合结果:', r)
    time.sleep(1.0)
    stop['on'] = True
    bad = [i for i, s in enumerate(samples)
           if s[3] is None or not np.isfinite(s[3])]
    if bad:
        i0 = bad[0]
        print('  --- NaN 出现前 8 帧 / 后 4 帧 (t, L, R, vL, vR, j2, vj2) ---')
        for s in samples[max(0, i0 - 8):i0 + 4]:
            print('   %+.2f  L=%-9s R=%-9s vL=%-10s vR=%-10s j2=%-8s vj2=%s'
                  % (s[0] - samples[0][0],
                     'None' if s[1] is None else '%.4f' % s[1],
                     'None' if s[2] is None else '%.4f' % s[2],
                     'None' if s[3] is None else '%.4f' % s[3],
                     'None' if s[4] is None else '%.4f' % s[4],
                     'None' if s[5] is None else '%.4f' % s[5],
                     'None' if s[6] is None else '%.4f' % s[6]))
    else:
        print('  全程无 NaN，末帧:', samples[-1])
    chk('夹紧后')
    fl, fr = n.fingers()
    print(f'  手指实际角度 L={fl} R={fr}')
    print('执行 抬升（逐段）:')
    for i in range(1, len(lift_wps)):
        dt = max(1.0, lift_ts[i] - lift_ts[i - 1])
        ok, msg = n.execute([lift_wps[i - 1], lift_wps[i]], [0, dt])
        print('  抬升段%d: %s' % (i, msg))
        b = n.nan()
        print('    [物理检查] 抬升段%d: %s' % (i, '正常' if not b else f'NaN! {b}'))
        if b:
            break
    chk('抬升后')
    time.sleep(3)

    qf = n.js.copy() if n.js is not None else q_lift
    Tf = kin.fk(qf)
    tcp = Tf[:3, 3] + Tf[:3, :3] @ TCP_OFF
    print(f'结束 TCP [{tcp[0]:.3f} {tcp[1]:.3f} {tcp[2]:.3f}] '
          f'水平偏差(相对感知目标) '
          f'{np.hypot(tcp[0]-bx, tcp[1]-by)*1000:.1f}mm')
    print('== 抓取流程结束 ==')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

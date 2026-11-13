#!/usr/bin/env python3
"""eff_test.py —— 力矩(effort)驱动的硬件对齐抓取闭环

关键改变：手指不再用 position 接口（那是运动学置位，ODE 不产生接触力，
导致既夹不住也触发不了 grasp_fix）。改用 effort 接口 + 恒力矩夹持，
等价于真机 MS42DC 的「电流限制」夹持：手指顶到方块就停，力恒定。
"""
import math
import os
import re
import sys
import time
import subprocess
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)                                              # grasp_v3.py 与本脚本同目录
sys.path.insert(0, os.path.expanduser('~/robot_vla_ws/src/arm_control/scripts'))
import grasp_v3 as V                      # noqa: E402

CMD = '/gripper_controller/commands'

# ---- 夹爪控制参数 ----
OPEN_T   = 0.20     # 张开目标角 -> 开口 58mm
PRE_T    = 0.04     # 预夹目标角 -> 开口 ~42mm（单边留 1mm）
TAU_MOVE = 0.020    # 移动用恒定力矩
TAU_GRIP = 0.030    # 夹持力矩 -> 指尖 0.03/0.05 = 0.6N/侧
TAU_MAX  = 0.060
KD       = 0.030    # 阻尼项（限速 ≈ TAU_MOVE/KD）
LIFT     = 0.10


def opening_mm(theta):
    return 2000.0 * (0.023 + 0.05 * np.sin(theta) - 0.004 * np.cos(theta))


def ik_aligned(kin, q_init, p_tip, offset):
    down = np.array([0.0, 0.0, -1.0])
    best = None
    # 真机夹爪沿末端旋转轴延伸 -> 姿态约束是【工具 z 轴】指向下方
    for xg in (np.array([1.0, 0, 0]), np.array([-1.0, 0, 0]),
               np.array([0, 1.0, 0]), np.array([0, -1.0, 0])):
        yg = np.cross(down, xg)          # 保证右手系 xg × yg = down
        R = np.column_stack([xg, yg, down])
        q, ok, err = V.dls_ik(kin, q_init, np.asarray(p_tip) - R @ offset, R)
        if not ok or err > 5e-3:
            continue
        if any(x < lo or x > hi for x, lo, hi in zip(q, V.LO, V.HI)):
            continue
        if best is None or np.max(np.abs(q)) < np.max(np.abs(best[0])):
            best = (q, R)
    return best


def box_pose():
    try:
        out = subprocess.run(['gz', 'model', '-m', 'target_box', '-p'],
                             capture_output=True, text=True, timeout=15).stdout
    except Exception as e:
        return None, 'gz 失败 %s' % e
    for line in out.splitlines():
        nums = re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', line)
        if len(nums) >= 3:
            return np.array([float(x) for x in nums[:3]]), line.strip()
    return None, (out.strip()[:200] or '空输出')


def show(tag):
    p, raw = box_pose()
    print('  %s: ' % tag + ('获取失败 [%s]' % raw if p is None
                            else '[%.4f %.4f %.4f]' % (p[0], p[1], p[2])))
    return p


class G(Node):
    def __init__(self):
        super().__init__('eff_test')
        self.q = {}
        self.v = {}
        self.img = None
        self.pts = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Image, '/wrist_depth_camera/image_raw', self.on_img, 10)
        self.create_subscription(PointCloud2, '/wrist_depth_camera/points', self.on_pts, 10)
        self.client = ActionClient(self, FollowJointTrajectory, V.ACTION)
        self.pub = self.create_publisher(Float64MultiArray, CMD, 10)
        self._tau = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def on_js(self, m):
        self.q = dict(zip(m.name, m.position))
        self.v = dict(zip(m.name, m.velocity))

    def on_img(self, m): self.img = m
    def on_pts(self, m): self.pts = m

    def arm(self):
        val = [self.q.get(j) for j in V.JOINTS]
        return None if any(x is None for x in val) else np.array(val)

    def left(self): return self.q.get('finger_left_joint', 0.0)
    def lvel(self): return self.v.get('finger_left_joint', 0.0)

    def _loop(self):
        m = Float64MultiArray()
        while True:
            m.data = [float(self._tau), float(-self._tau)]
            try:
                self.pub.publish(m)
            except Exception:
                pass
            time.sleep(0.02)

    def tau(self, t): self._tau = max(-TAU_MAX, min(TAU_MAX, t))

    def nan(self):
        return any(not np.isfinite(x) for x in self.q.values()) if self.q else False

    def execute(self, wps, ts):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = V.JOINTS
        for q, t in zip(wps, ts):
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in q]
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal.trajectory.points.append(p)
        ev = threading.Event(); holder = {}

        def _done(fut):
            holder['res'] = fut.result(); ev.set()
        self.client.send_goal_async(goal).add_done_callback(
            lambda g: (g.result().get_result_async().add_done_callback(_done)
                       if g.result().accepted
                       else (holder.__setitem__('rejected', True), ev.set())))
        if not ev.wait(timeout=max(ts) + 40):
            return False, '超时'
        if holder.get('rejected'):
            return False, '被拒'
        return True, 'OK'


def move_to(n, theta_t, secs, label):
    """恒力矩 + 阻尼项：等价于真机「恒定电流、顶到就停」"""
    t0 = time.time(); i = 0
    while time.time() - t0 < secs:
        err = theta_t - n.left()
        if abs(err) < 0.005:
            t = -KD * n.lvel()
        else:
            t = math.copysign(TAU_MOVE, err) - KD * n.lvel()
        n.tau(t)
        time.sleep(0.02); i += 1
        if i % 25 == 0:
            print('    %s %+.1fs 目标%+.3f 实际%+.4f 开口%.1fmm 力矩%+.4f'
                  % (label, time.time() - t0, theta_t, n.left(),
                     opening_mm(n.left()), t))
    return n.left()


def main():
    _URDF = os.environ.get('ECO65_URDF', '/tmp/eco65_6fb.urdf')
    kin = V.UrdfKinematics(_URDF, 'baselink', 'Link6')
    print('TCP_OFF =', V.TCP_OFF, ' BOX_HALF =', V.BOX_HALF)
    rclpy.init()
    n = G()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
    if not n.client.wait_for_server(timeout_sec=25.0):
        print('!! arm action 未就绪'); return
    t0 = time.time()
    while time.time() - t0 < 40 and n.arm() is None:
        time.sleep(0.2)
    if n.arm() is None:
        print('!! 无 joint_states'); return
    print('手指关节:', [k for k in n.q if 'finger' in k] or '没有！')

    b0 = show('起始')

    # ===== 阶段1：张开 =====
    print('\n===== 阶段1：力矩驱动张开 =====')
    move_to(n, OPEN_T, 4.0, '张开')
    print('  张开到位 开口=%.1fmm' % opening_mm(n.left()))

    # ===== 阶段2：观测 + 感知 =====
    print('\n===== 阶段2：观测与感知 =====')
    print('回标准姿势:', n.execute([n.arm(), np.array([0.0, 0.2, 0.3, 0, 0, 0])], [0, 4])[1])
    time.sleep(3)
    sol = V.ik_with_roll(kin, n.arm(), V.P_CAM, V.WORK_AIM - V.P_CAM, V.CAM_OFF)
    if sol is None:
        print('!! 观测姿势无解'); return
    print('到观测姿势:', n.execute([n.arm(), sol[0]], [0, 5])[1]); time.sleep(5)
    t0 = time.time()
    while time.time() - t0 < 45 and (n.img is None or n.pts is None):
        time.sleep(0.3)
    if n.img is None or n.pts is None:
        print('!! 相机数据不足'); return
    q_act = n.arm()
    T6 = kin.fk(q_act)
    a = V.localize(n, T6[:3, 3], T6[:3, :3], 20)
    if a is None or len(a) < 3:
        print('!! 感知失败'); return
    mean = a.mean(axis=0)
    print('>>> 感知顶面 [%.3f %.3f %.3f] 样本%d' % (mean[0], mean[1], mean[2], len(a)))
    bx, by, bz = mean
    bz_c = bz - V.BOX_HALF
    print('    方块中心 [%.3f %.3f %.3f]' % (bx, by, bz_c))

    # ===== 阶段3：下降 =====
    top_t = np.array([bx, by, bz_c + 0.16])   # 夹爪加长到19.5cm后，TCP高于约0.22就无解；0.18留余量
    s_top = ik_aligned(kin, q_act, top_t, V.TCP_OFF)
    if s_top is None:
        print('!! 高点无对齐解'); return
    print('\n===== 阶段3：移到高点并笛卡尔下降 =====')
    print('移到高点:', n.execute([q_act, s_top[0]], [0, 6])[1]); time.sleep(2)
    btop = show('到高点后')
    # 判据必须用【真值 vs 真值】：感知位置 bx/by 本身带数毫米系统偏差，
    # 拿真值去和感知值比会必然误报（实测 y 偏 12mm > 阈值 10mm，
    # 而方块真值始终是 0.4000/-0.0000，根本没被碰到）。
    if (btop is not None and b0 is not None
            and np.hypot(btop[0] - b0[0], btop[1] - b0[1]) > 0.01):
        print('  !! 到高点途中方块被碰到，中止'); return

    cur = s_top[0]
    N = 6
    wps = [cur]; ts = [0]
    for i in range(1, N + 1):
        z = top_t[2] + (bz_c - top_t[2]) * i / N
        s = ik_aligned(kin, cur, np.array([bx, by, z]), V.TCP_OFF)
        if s is None:
            print('!! 下降第%d步无解' % i); return
        cur = s[0]; wps.append(cur); ts.append(2 * i)
    q_grasp = cur
    s_lift = ik_aligned(kin, q_grasp, np.array([bx, by, bz_c + LIFT]), V.TCP_OFF)
    if s_lift is None:
        print('!! 抬升无解'); return
    print('执行下降:', n.execute(wps, ts)[1]); time.sleep(2)
    bb = show('下降后')
    Tg = kin.fk(n.arm())
    tcp = Tg[:3, 3] + Tg[:3, :3] @ V.TCP_OFF
    print('  下降后 TCP = %s  (目标中心 [%.3f %.3f %.3f])' % (np.round(tcp, 4), bx, by, bz_c))
    if (bb is not None and b0 is not None
            and np.hypot(bb[0] - b0[0], bb[1] - b0[1]) > 0.01):
        print('  !! 下降途中方块被碰到，中止'); return

    # ===== 阶段4：预夹 + 恒力矩夹持 =====
    print('\n===== 阶段4：预夹 -> 恒力矩夹持 =====')
    move_to(n, PRE_T, 3.0, '预夹')
    print('  预夹到位 开口=%.1fmm（方块 40mm，单边余 %.1fmm）'
          % (opening_mm(n.left()), (opening_mm(n.left()) - 40) / 2))
    print('  施加恒定夹持力矩 %.3f N·m（指尖约 %.2f N/侧）'
          % (TAU_GRIP, TAU_GRIP / 0.05))
    # 恒力矩闭合 + 堵转检测：顶到方块后改为「保持角度 + 小偏置力」
    # 真机同样靠电流/速度判断夹持到位，不会一路硬顶到机械限位
    t0 = time.time(); stall = 0; hold = None
    while time.time() - t0 < 4.0:
        v = n.lvel()
        stall = stall + 1 if abs(v) < 0.02 else 0
        if stall > 15 and hold is None:
            hold = n.left()
            print('  >> 堵转，停在 %.4f rad，开口 %.1fmm' % (hold, opening_mm(hold)))
        if hold is None:
            n.tau(-TAU_GRIP - KD * v)
        else:
            n.tau(0.15 * (hold - n.left()) - KD * v - 0.015)
        time.sleep(0.02)
    th = n.left()
    print('  夹持 4s 后 左指=%+.4f 开口=%.1fmm' % (th, opening_mm(th)))
    bc = show('夹持后')

    # ===== 阶段5：保持夹持力矩并抬升 =====
    print('\n===== 阶段5：保持夹持力矩，笛卡尔抬升 =====')
    wps_l = [q_grasp]; ts_l = [0]
    cur_l = q_grasp
    for i in range(1, 6):
        z = bz_c + LIFT * i / 5.0
        s = ik_aligned(kin, cur_l, np.array([bx, by, z]), V.TCP_OFF)
        if s is None:
            print('  !! 抬升第%d步无解，停在第%d步' % (i, i - 1)); break
        cur_l = s[0]; wps_l.append(cur_l); ts_l.append(4.0 * i)
    print('笛卡尔直线抬升 %d 步' % (len(wps_l) - 1))
    print('执行抬升:', n.execute(wps_l, ts_l)[1])
    for i in range(1, 41):
        if hold is None:
            n.tau(-TAU_GRIP - KD * n.lvel())
        else:
            n.tau(0.15 * (hold - n.left()) - KD * n.lvel() - 0.015)
        time.sleep(0.5)
        p, _ = box_pose()
        print('  +%.1fs 左指=%+.4f 开口=%.1fmm 方块=%s'
              % (i * 0.5, n.left(), opening_mm(n.left()),
                 '失败' if p is None else np.round(p, 4)))
    time.sleep(3)
    ba = show('抬升后')

    print('\n===== 结论 =====')
    for tag, v in (('起始', b0), ('下降后', bb), ('夹持后', bc), ('抬升后', ba)):
        print('  %s: %s' % (tag, None if v is None else np.round(v, 4)))
    if b0 is not None and ba is not None:
        dz = ba[2] - b0[2]
        print('  dz = %+.4f m -> %s' % (dz, '★ 夹起来了！' if dz > 0.02 else '没夹起来'))
    print('  NaN:', '有' if n.nan() else '无')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

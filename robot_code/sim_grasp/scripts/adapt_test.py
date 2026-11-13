#!/usr/bin/env python3
"""adapt_test.py —— 自适应：先判别 velocity 接口到底是「位置驱动」还是「速度驱动」，再抓

背景：本环境插件导出的指令接口名字和 URDF 声明的对不上。
  当前 URDF 声明 position -> 插件实际导出 finger_*_joint/velocity。
  所以用 velocity 控制器去 claim 它，然后实测它到底怎么解释收到的数据：
    发常量 0.20 后角度稳定在 0.20    -> 它其实是位置驱动（只是名字叫 velocity）
    发常量 0.20 后角度一路冲到限位    -> 它是真速度驱动
两种情况都能做力受限夹持（ODE 电机都受 <limit effort> 约束），只是控制律不同。
"""
import re
import os
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
OPEN = 0.20
CLOSE = -0.03      # 压深 ~2.5mm：够建立接触即可，夹持靠 grasp_fix 固定关节
LIFT = 0.10
KP = 8.0
VMAX = 0.6


def opening_mm(theta):
    # 铰链已内移到 y=±0.023（原 ±0.026），θ=0 时开口 39mm → 均匀压入 1mm 的面接触
    return 2000.0 * (0.023 + 0.05 * np.sin(theta) - 0.004 * np.cos(theta))


def ik_aligned(kin, q_init, p_tip, offset):
    """强制夹爪捏合轴(gripper y)与世界 x 或 y 轴平行

    原因：方块是轴对齐的 40mm 立方体，对角长 56.6mm；张开只有 58mm。
    如果滚转角随便选，捏合方向斜着切过方块棱角 -> 余量仅 0.7mm，必撞。
    真机上同样必须让夹爪对齐物体表面。
    """
    down = np.array([0.0, 0.0, -1.0])          # 工具轴指向下
    best = None
    for yg in (np.array([1.0, 0, 0]), np.array([-1.0, 0, 0]),
               np.array([0, 1.0, 0]), np.array([0, -1.0, 0])):
        zg = np.cross(down, yg)
        R = np.column_stack([down, yg, zg])
        flange = np.asarray(p_tip) - R @ offset
        q, ok, err = V.dls_ik(kin, q_init, flange, R)
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
        return None, f'gz 失败 {e}'
    for line in out.splitlines():
        nums = re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', line)
        if len(nums) >= 3:
            return np.array([float(x) for x in nums[:3]]), line.strip()
    return None, (out.strip()[:200] or '空输出')


def show(tag):
    p, raw = box_pose()
    print(f'  {tag}: ' + ('获取失败 [%s]' % raw if p is None
                          else '[%.4f %.4f %.4f]' % (p[0], p[1], p[2])))
    return p


class G(Node):
    def __init__(self):
        super().__init__('adapt_test')
        self.q = {}
        self.img = None
        self.pts = None
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Image, '/wrist_depth_camera/image_raw',
                                 self.on_img, 10)
        self.create_subscription(PointCloud2, '/wrist_depth_camera/points',
                                 self.on_pts, 10)
        self.client = ActionClient(self, FollowJointTrajectory, V.ACTION)
        self.pub = self.create_publisher(Float64MultiArray, CMD, 10)
        self._tgt = None
        threading.Thread(target=self._loop, daemon=True).start()

    def on_js(self, m): self.q = dict(zip(m.name, m.position))
    def on_img(self, m): self.img = m
    def on_pts(self, m): self.pts = m

    def arm(self):
        v = [self.q.get(j) for j in V.JOINTS]
        return None if any(x is None for x in v) else np.array(v)

    def left(self): return self.q.get('finger_left_joint', 0.0)

    def nan(self):
        return any(not np.isfinite(x) for x in self.q.values()) if self.q else False

    def _loop(self):
        m = Float64MultiArray()
        while True:
            if self._tgt is None:
                m.data = [0.0, 0.0]
            else:
                m.data = [float(self._tgt), float(-self._tgt)]
            try:
                self.pub.publish(m)
            except Exception:
                pass
            time.sleep(0.02)

    def hold(self, v): self._tgt = v
    def relax(self): self._tgt = None

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


def trace(n, secs, label):
    t0 = time.time(); i = 0
    while time.time() - t0 < secs:
        time.sleep(0.5); i += 1
        print(f'  {label}+{i*0.5:.1f}s 左指={n.left():+.4f} '
              f'开口={opening_mm(n.left()):.1f}mm')


def main():
    kin = V.UrdfKinematics('/tmp/eco65_6fb.urdf', 'baselink', 'Link6')
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

    # ================= 阶段0：判别接口语义 =================
    print('\n===== 阶段0：判别 velocity 接口语义 =====')
    print('  持续发常量 0.20（3 秒），观察手指角度')
    n.hold(OPEN)
    trace(n, 3.0, '发0.20')
    a1 = n.left()
    print(f'  发 0.20 后角度={a1:+.4f}')
    # 判据只看稳定值：位置驱动会精确停在 0.20；速度驱动会一路冲向限位 0.5
    if abs(a1 - OPEN) < 0.03:
        mode = 'POS'
        print('  => 判定：位置驱动（发给它的是目标角）')
    elif abs(a1) > 0.28:
        mode = 'VEL'
        print('  => 判定：真速度驱动（0.20rad/s 一路冲到限位）')
    else:
        mode = 'POS'
        print(f'  => 判定存疑（{a1:+.4f}），按位置驱动处理')
    if abs(a1) < 0.01:
        print('  !! 手指完全没动，接口无效'); return

    def grip(theta, secs=1.5):
        """斜坡逼近目标角：真机夹爪也是受控速度闭合，瞬间跳目标会砸飞方块"""
        if mode == 'POS':
            start = n.left()
            steps = max(1, int(secs / 0.05))
            print('  --- 斜坡闭合逐点日志(每 50ms) ---')
            for i in range(1, steps + 1):
                tgt = start + (theta - start) * i / steps
                n.hold(tgt)
                pb, _ = box_pose()
                print('    目标%+.4f 实际%+.4f 开口%6.1fmm 方块%s'
                      % (tgt, n.left(), opening_mm(n.left()),
                         'None' if pb is None else np.round(pb, 4)))
                time.sleep(0.05)
            print('  --- 斜坡结束 ---')
        else:
            n._pv = theta
            # P 控制放进 _loop 需要改；这里用简单逼近
            for _ in range(int(3.0 / 0.05)):
                v = max(-VMAX, min(VMAX, KP * (theta - n.left())))
                m = Float64MultiArray(); m.data = [float(v), float(-v)]
                n.pub.publish(m)
                time.sleep(0.05)
        print(f'  夹爪目标 {theta:+.3f} (开口 {opening_mm(theta):.1f}mm) '
              f'实际 {n.left():+.4f}')

    # ================= 阶段1：观测 + 感知 =================
    print('\n===== 阶段1：观测与感知 =====')
    print('回标准姿势:', n.execute([n.arm(), np.array([0.0, 0.2, 0.3, 0, 0, 0])],
                                    [0, 4])[1]); time.sleep(3)
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
    print(f'>>> 感知顶面 [{mean[0]:.3f} {mean[1]:.3f} {mean[2]:.3f}] 样本{len(a)}')
    bx, by, bz = mean
    bz_c = bz - V.BOX_HALF
    print(f'    方块中心 [{bx:.3f} {by:.3f} {bz_c:.3f}]')

    # ================= 阶段2：规划 + 下降 =================
    top_t = np.array([bx, by, bz_c + 0.25])
    s_top = ik_aligned(kin, q_act, top_t, V.TCP_OFF)
    if s_top is None:
        print('!! 高点无对齐解'); return

    print('\n===== 阶段2：张开 -> 移到方块正上方 =====')
    grip(OPEN); time.sleep(2)
    print(f'  张开到位 开口={opening_mm(n.left()):.1f}mm')
    print('移到高点:', n.execute([q_act, s_top[0]], [0, 6])[1]); time.sleep(2)
    btop = show('到高点后')
    if btop is not None and np.hypot(btop[0] - bx, btop[1] - by) > 0.01:
        print('  !! 到高点途中方块被碰到，中止'); return

    cur = s_top[0]
    N = 6
    wps = [cur]; ts = [0]
    for i in range(1, N + 1):
        z = top_t[2] + (bz_c - top_t[2]) * i / N
        s = ik_aligned(kin, cur, np.array([bx, by, z]), V.TCP_OFF)
        if s is None:
            print(f'!! 下降第{i}步无解'); return
        cur = s[0]; wps.append(cur); ts.append(2 * i)
    q_grasp = cur
    s_lift = ik_aligned(kin, q_grasp, np.array([bx, by, bz_c + LIFT]),
                        V.TCP_OFF)
    if s_lift is None:
        print('!! 抬升无解'); return
    print(f'笛卡尔直线下降 {N} 步')
    print('执行下降:', n.execute(wps, ts)[1]); time.sleep(2)
    bb = show('下降后')
    if bb is not None and np.hypot(bb[0] - bx, bb[1] - by) > 0.01:
        print('  !! 下降途中方块被碰到，中止'); return

    # ================= 阶段3：闭合 =================
    print('\n===== 阶段3：闭合抓取 =====')
    grip(CLOSE)
    trace(n, 3.0, '闭合')
    th = n.left()
    print(f'  闭合结束 左指={th:+.4f} 开口={opening_mm(th):.1f}mm '
          f'压深{(40-opening_mm(th))/2:.2f}mm/侧')
    print(f'  {"!! 走到指令角=刚性穿透" if abs(th - CLOSE) < 0.02 else "停在方块上=力矩上限生效(柔顺)"}')
    bc = show('闭合后')

    # ================= 阶段4：抬升 =================
    print('\n===== 阶段4：抬升 =====')
    if mode == 'POS':
        n.hold(CLOSE)
    # 抬升必须也是笛卡尔直线：关节空间插值会让 TCP 走弧线，
    # 刚性手指横向拖拽方块 -> 巨大接触力 -> 弹飞(实测飞出 46m)
    wps_l = [q_grasp]; ts_l = [0]
    cur_l = q_grasp
    NL = 5
    for i in range(1, NL + 1):
        z = bz_c + LIFT * i / NL
        s = ik_aligned(kin, cur_l, np.array([bx, by, z]), V.TCP_OFF)
        if s is None:
            print(f'  !! 抬升第{i}步无解，停在第{i-1}步'); break
        cur_l = s[0]; wps_l.append(cur_l); ts_l.append(4.0 * i)
    print(f'笛卡尔直线抬升 {len(wps_l)-1} 步')
    print('执行抬升:', n.execute(wps_l, ts_l)[1])
    for i in range(1, 41):
        time.sleep(0.5)
        p, _ = box_pose()
        print(f'  +{i*0.5:.1f}s 左指={n.left():+.4f} 方块='
              f'{"失败" if p is None else np.round(p, 4)}')
        if mode == 'VEL':
            v = max(-VMAX, min(VMAX, KP * (CLOSE - n.left())))
            m = Float64MultiArray(); m.data = [float(v), float(-v)]
            n.pub.publish(m)
    time.sleep(3)
    ba = show('抬升后')

    print('\n===== 结论 =====')
    for tag, v in (('起始', b0), ('下降后', bb), ('闭合后', bc), ('抬升后', ba)):
        print(f'  {tag}: {None if v is None else np.round(v, 4)}')
    if b0 is not None and ba is not None:
        dz = ba[2] - b0[2]
        print(f'  dz = {dz:+.4f} m -> {"★ 夹起来了！" if dz > 0.02 else "没夹起来"}')
    print('  接口模式:', mode, ' NaN:', '有' if n.nan() else '无')
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

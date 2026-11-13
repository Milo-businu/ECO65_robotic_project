#!/usr/bin/env python3
"""grasp_closed_loop.py —— P5 闭环验证（云服务器，Python 3.8 侧）

链路: 腕部相机图像 -> VLA HTTP /predict(Py3.11) -> 动作块(15,8)
      -> 增量限幅 -> 两点式轨迹 -> rm_group_controller(位置接口)

安全设计:
  - MAX_STEP 每步限幅(防模型输出暴走)
  - 关节限位裁剪
  - 看门狗(joint_states 断流即停)
  - 总块数上限

验收(工程伦理: 不硬编码真值):
  - /gazebo/model_states 里 target_box 抓取前后位姿对比
"""
import base64, io, json, math, time, urllib.request
from threading import Thread, Event

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from PIL import Image as PILImage

try:
    from gazebo_msgs.msg import ModelStates
except Exception:
    ModelStates = None

# ---------------- 配置 ----------------
URL        = 'http://127.0.0.1:8000/predict'
PROMPT     = 'pick up the green cube'
IMG_TOPIC  = '/wrist_depth_camera/image_raw'
ACTION     = '/rm_group_controller/follow_joint_trajectory'
JOINTS     = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
LO = np.array([-3.106, -3.106, -2.792, -3.106, -3.106, -6.28])
HI = np.array([ 3.106,  2.356,  2.530,  3.106,  3.106,  6.28])
MAX_STEP   = 0.05     # 每步最大关节增量(rad) —— 安全限幅
STEP_DT    = 0.2      # 每步时长(s)
CHUNKS     = 3        # 执行几个动作块（每块 15 步）
JS_TIMEOUT = 5.0      # 看门狗: joint_states 断流秒数
IMG_WAIT   = 20.0     # 等第一帧相机的秒数

def wrap(a):
    return np.array([math.atan2(math.sin(x), math.cos(x)) for x in a])

class Loop(Node):
    def __init__(self):
        super().__init__('grasp_closed_loop')
        self.js = None; self.js_t = 0.0
        self.img = None
        self.models = {}
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.create_subscription(Image, IMG_TOPIC, self.on_img, 1)
        if ModelStates is not None:
            self.create_subscription(ModelStates, '/gazebo/model_states', self.on_models, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.get_logger().info('等待 action 服务器 ...')
        self.client.wait_for_server()
        self.get_logger().info('action 就绪')

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.js = np.array([d[j] for j in JOINTS])
            self.js_t = time.time()

    def on_img(self, m):
        self.img = m

    def on_models(self, m):
        self.models = dict(zip(m.name, m.pose))

    def image_b64(self):
        m = self.img
        if m is None:
            return None
        if m.encoding in ('rgb8', 'rgba8'):
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
            mode = 'RGB'
        elif m.encoding in ('bgr8', 'bgra8'):
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, :3][:, :, ::-1]
            mode = 'RGB'
        elif m.encoding == 'mono8':
            arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width)
            mode = 'L'
        else:
            self.get_logger().warn('未知图像编码: ' + m.encoding)
            return None
        im = PILImage.fromarray(arr, mode).convert('RGB')
        buf = io.BytesIO()
        im.save(buf, 'JPEG', quality=85)
        return base64.b64encode(buf.getvalue()).decode()

    def predict(self, q6, img_b64):
        req = {
            "prompt": PROMPT,
            "exterior_left":  img_b64,     # 没有外部相机，用腕部图充当
            "exterior_right": None,
            "wrist":          img_b64,
            "joint_position": [float(x) for x in q6] + [0.0],   # 7 维: 6 关节 + 占位
            "gripper_position": [0.0],
        }
        r = urllib.request.urlopen(urllib.request.Request(
            URL, data=json.dumps(req).encode(),
            headers={'Content-Type': 'application/json'}), timeout=300)
        resp = json.loads(r.read())
        return np.array(resp['actions'], dtype=np.float64)   # (15, 8)

    def send_step(self, q_now, q_cmd):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        p0 = JointTrajectoryPoint()
        p0.positions = [float(x) for x in q_now]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = [float(x) for x in q_cmd]
        p1.time_from_start.sec = 0
        p1.time_from_start.nanosec = int(STEP_DT * 1e9)
        goal.trajectory.points = [p0, p1]

        # Foxy 的 Future.result() 不阻塞（未完成即返回 None），
        # 必须用 add_done_callback + Event 等待（pid_track 同款写法）
        ev_goal = Event()
        holder = {}

        def _on_goal(fut):
            holder['gh'] = fut.result()
            ev_goal.set()

        self.client.send_goal_async(goal).add_done_callback(_on_goal)
        if not ev_goal.wait(timeout=10.0):
            self.get_logger().warn('发送目标 10s 未受理，跳过该步')
            return False
        gh = holder.get('gh')
        if gh is None or not gh.accepted:
            return False

        ev_res = Event()
        gh.get_result_async().add_done_callback(lambda f: ev_res.set())
        if not ev_res.wait(timeout=STEP_DT + 15.0):
            self.get_logger().warn('等待执行结果超时，继续下一步')
        return True

def box_pose(node, tag):
    p = node.models.get('target_box')
    if p is None:
        node.get_logger().info(f'[{tag}] target_box 位姿不可用（/gazebo/model_states 无该模型）')
        return
    node.get_logger().info(
        f'[{tag}] target_box: x={p.position.x:+.3f} y={p.position.y:+.3f} z={p.position.z:+.3f}')

def main():
    rclpy.init()
    node = Loop()
    spin = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    # ① 等 joint_states
    t0 = time.time()
    while node.js is None and time.time() - t0 < 10:
        time.sleep(0.1)
    if node.js is None:
        node.get_logger().error('10s 内没收到 /joint_states —— sim 是否在跑？')
        return
    q = node.js.copy()
    node.get_logger().info(f'起始关节角: {np.round(q, 3)}')

    # ② 等第一帧相机图像
    t0 = time.time()
    while node.img is None and time.time() - t0 < IMG_WAIT:
        time.sleep(0.2)
    if node.img is None:
        node.get_logger().error(f'{IMG_WAIT}s 内没收到 {IMG_TOPIC} —— 请另开终端先跑: simcam')
        return
    node.get_logger().info(f'相机在线: {node.img.width}x{node.img.height} ({node.img.encoding})')

    # ③ 抓取前真值
    time.sleep(1.0)
    box_pose(node, '抓取前')

    # ④ 主循环: 请求动作块 -> 限幅执行
    for c in range(CHUNKS):
        if time.time() - node.js_t > JS_TIMEOUT:
            node.get_logger().error('看门狗: joint_states 断流，中止')
            break
        img_b64 = node.image_b64()
        if img_b64 is None:
            node.get_logger().warn('拿不到图像帧，跳过本块')
            time.sleep(1.0)
            continue
        t0 = time.time()
        try:
            actions = node.predict(node.js.copy(), img_b64)
        except Exception as e:
            node.get_logger().error(f'/predict 失败: {e}')
            break
        node.get_logger().info(
            f'块{c + 1}/{CHUNKS}: 动作块 {actions.shape}，推理 {time.time() - t0:.1f}s，'
            f'首动作前3维 {np.round(actions[0][:3], 3)}')

        for k in range(actions.shape[0]):
            if time.time() - node.js_t > JS_TIMEOUT:
                node.get_logger().error('看门狗: joint_states 断流，中止')
                return
            delta = np.clip(actions[k][:6], -MAX_STEP, MAX_STEP)
            q_cmd = np.clip(q + delta, LO, HI)
            if not node.send_step(q, q_cmd):
                node.get_logger().warn('目标被拒绝，跳过该步')
                continue
            t_wait = time.time()
            while node.js_t < t_wait and time.time() - t_wait < 2.0:
                time.sleep(0.02)
            q = wrap(node.js.copy())
        node.get_logger().info(f'块{c + 1} 执行完毕，当前关节角 {np.round(q, 3)}')
        time.sleep(0.5)

    # ⑤ 抓取后真值 + 收尾
    time.sleep(1.0)
    box_pose(node, '抓取后')
    node.get_logger().info(f'结束关节角: {np.round(node.js, 3) if node.js is not None else "无"}')
    node.get_logger().info('== P5 闭环验证结束（抓取成功与否以 P6 微调后为准）==')

if __name__ == '__main__':
    main()

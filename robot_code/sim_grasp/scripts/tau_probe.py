import time, threading, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

rclpy.init()
n = Node('tau_probe')
q = {}
n.create_subscription(JointState, '/joint_states', lambda m: q.update(dict(zip(m.name, m.position))), 10)
pub = n.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
time.sleep(2)
print('起始 左指=%s 右指=%s' % (q.get('finger_left_joint'), q.get('finger_right_joint')))
for tau in (0.010, 0.020, -0.020, 0.0):
    for _ in range(int(1.5/0.02)):
        m = Float64MultiArray(); m.data = [float(tau), float(-tau)]
        pub.publish(m); time.sleep(0.02)
    print('力矩%+.3f 1.5s 后  左指=%+.4f 右指=%+.4f'
          % (tau, q.get('finger_left_joint', 0), q.get('finger_right_joint', 0)))

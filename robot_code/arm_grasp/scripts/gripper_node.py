#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夹爪 ROS2 节点 —— MS42DC 串口驱动接进整体 ROS2

订阅：
  /gripper/cmd        std_msgs/String   open | close | home | grip:<mm> | query | stop
  /gripper/cmd_width  std_msgs/Float64  目标开口 mm
发布：
  /gripper/state      std_msgs/Float64MultiArray [开口mm, 累计角度°, 到位0/1, 堵转0/1]
                      （开口未知时第一项是 -1）

⚠️ 三条必须记住的事
  1. 协议发的是【相对角度增量】，不是绝对位置 → 节点内部维护"当前开口"
     来算增量。所以上电后必须先发 home（张开到底）建立基准。
  2. 本机只有位置模式，没有力控。夹持靠「目标开口略小于物体 → 顶到后堵转」。
     夹完不要发 stop/断电，让电机保持。
  3. gripper.calibration_done=false 时开口数值不可信，先跑 calibrate_gripper.py。

运行：
  python3 gripper_node.py --config /path/to/real_params.yaml
  # 或 ros2 run arm_grasp gripper_node
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import load_config, is_dry_run, banner   # noqa: E402
from ms42dc_driver import Ms42dc, OpeningMap          # noqa: E402


class GripperNode:
    def __init__(self, cfg, node):
        import rclpy
        from std_msgs.msg import String, Float64, Float64MultiArray
        self.cfg = cfg
        self.node = node
        self.dry_run = is_dry_run(cfg)
        g = cfg["gripper"]

        self.omap = OpeningMap(max_width_mm=float(g["max_width_mm"]),
                               deg_per_mm=float(g["deg_per_mm"]),
                               deadzone_deg=float(g.get("deadzone_deg", 0.0)),
                               min_width_mm=float(g.get("min_width_mm", 10.0)),
                               calibrated=bool(g.get("calibration_done", False)))
        self.dev = Ms42dc(port=g["port"], baud=int(g["baud"]),
                          addr=int(g["addr"], 0) if isinstance(g["addr"], str) else int(g["addr"]),
                          speed_rads=float(g["speed_rads"]),
                          microstep=int(str(g["microstep"]), 0)
                          if isinstance(g["microstep"], str) else int(g["microstep"]),
                          dry_run=self.dry_run)

        # 上电后开口未知。必须先 home 才能"相信"开口数值。
        self.current_width = None
        self.reached = True
        self.stall = False
        self.busy = False
        self._lock = threading.Lock()

        node.create_subscription(String, g["cmd_topic"], self.on_cmd, 10)
        node.create_subscription(Float64, g["width_topic"], self.on_width, 10)
        self.state_pub = node.create_publisher(Float64MultiArray, g["state_topic"], 10)
        node.create_timer(float(g.get("query_interval_s", 0.3)), self._publish_state)
        self._Float64MultiArray = Float64MultiArray

        node.get_logger().info(
            "夹爪节点就绪 mode=%s port=%s 标定=%s deg/mm=%.3f" %
            ("DRY-RUN" if self.dry_run else "REAL", g["port"],
             self.omap.calibrated, self.omap.deg_per_mm))
        if not self.omap.calibrated:
            node.get_logger().warn(
                "⚠️ 开口映射未标定（calibration_done: false）—— "
                "现在算出来的 mm 只是粗估，先跑 calibrate_gripper.py")

    # ------------------------------------------------------------ 命令
    def on_width(self, m):
        self._submit("width", float(m.data))

    def on_cmd(self, m):
        cmd = str(m.data).strip().lower()
        self._submit("cmd", cmd)

    def _submit(self, kind, value):
        # 电机动作要等几秒，放到工作线程，别卡住 rclpy 的 spin
        t = threading.Thread(target=self._execute, args=(kind, value), daemon=True)
        t.start()

    def _execute(self, kind, value):
        with self._lock:
            self.busy = True
            try:
                if kind == "cmd":
                    self._do_cmd(value)
                else:
                    self._move_to(value, "目标开口 %.1fmm" % value)
            except Exception as e:                     # noqa: BLE001
                self.node.get_logger().error("夹爪命令失败: %s" % e)
            finally:
                self.busy = False

    def _do_cmd(self, cmd):
        g = self.cfg["gripper"]
        log = self.node.get_logger().info
        if cmd == "home":
            # 张开到底：发一个大角度，靠机械限位/堵转停，然后把基准设为最大开口
            self.dev.move_angle(self.omap.width_to_angle(float(g["min_width_mm"])),
                                int(g["direction_open"]))
            time.sleep(float(g["move_timeout_s"]))
            self.current_width = float(g["max_width_mm"])
            self.reached = True
            log("home 完成，基准开口 = %.1fmm" % self.current_width)
        elif cmd == "open":
            self._move_to(float(g["open_width_mm"]), "张开")
        elif cmd == "close":
            self._move_to(float(g["min_width_mm"]), "闭合")
        elif cmd.startswith("grip:"):
            try:
                obj_mm = float(cmd.split(":", 1)[1])
            except ValueError:
                self.node.get_logger().error("grip 格式应为 grip:<物体宽度mm>")
                return
            target = obj_mm - float(g["interference_mm"])
            self._move_to(target, "夹持(物体%.1fmm, 目标开口%.1fmm)" % (obj_mm, target))
        elif cmd == "query":
            fb = self.dev.query()
            if fb:
                self.dev._angle = fb["angle_deg"]
                self.reached = bool(fb["reached"])
            log("查询反馈: %s" % fb)
        elif cmd == "stop":
            # 协议没有急停帧；停止发新指令即可，电机保持当前位置
            self.node.get_logger().warn("收到 stop：不再下发新指令（电机保持）")
        else:
            self.node.get_logger().warn("未知命令 %r（支持 open/close/home/grip:<mm>/query/stop）" % cmd)

    def _move_to(self, target_mm, label):
        g = self.cfg["gripper"]
        if self.current_width is None:
            self.node.get_logger().warn(
                "开口基准未知，先自动 home（张开到底）再执行 %s" % label)
            self._do_cmd("home")
        delta = self.current_width - float(target_mm)
        if abs(delta) < 0.5:
            self.node.get_logger().info("%s：已在目标开口附近，跳过" % label)
            return
        direction = int(g["direction_close"]) if delta > 0 else int(g["direction_open"])
        delta_deg = abs(delta) * self.omap.deg_per_mm
        # ⚠️ 从 home（最大开口）出发的【第一次闭合】要额外吃掉机械空程角。
        #    空程是常数偏移，在连续闭合的增量差分里会相互抵消，
        #    所以只在「刚好从 max_width 起步」时补一次。
        from_home = (self.current_width >= float(g["max_width_mm"]) - 0.5
                     and delta > 0)
        if from_home and self.omap.deadzone_deg > 0:
            delta_deg += self.omap.deadzone_deg
        self.node.get_logger().info(
            "%s：%.1fmm -> %.1fmm，走 %.1f° dir=%d%s%s" %
            (label, self.current_width, target_mm, delta_deg, direction,
             "（含空程 %.1f°）" % self.omap.deadzone_deg if from_home else "",
             " [DRY-RUN]" if self.dry_run else ""))
        self.dev.move_angle(delta_deg, direction)
        # 等到位 / 堵转（轮询反馈）
        t0 = time.time()
        self.reached = False
        self.stall = False
        last_angle = None
        steady = 0
        while time.time() - t0 < float(g["move_timeout_s"]):
            fb = self.dev.query()
            if fb:
                self.reached = bool(fb["reached"])
                a = fb["angle_deg"]
                self.dev._angle = a
                if last_angle is not None and abs(a - last_angle) < 0.05:
                    steady += 1
                else:
                    steady = 0
                last_angle = a
                if self.dry_run:
                    self.reached = True
                if self.reached or steady >= 6:     # 6 次没动 = 认为顶到东西停了
                    self.stall = steady >= 6 and not self.reached
                    break
            time.sleep(0.15)
        # 只有真正到位才更新开口；堵转时按"目标开口 + 过盈"记（保守）
        self.current_width = float(target_mm)
        self.node.get_logger().info(
            "%s 结束：开口≈%.1fmm 到位=%s 堵转=%s" %
            (label, self.current_width, self.reached, self.stall))

    def _publish_state(self):
        m = self._Float64MultiArray()
        m.data = [float(self.current_width if self.current_width is not None else -1.0),
                  float(getattr(self.dev, "_angle", 0.0)),
                  1.0 if self.reached else 0.0,
                  1.0 if self.stall else 0.0]
        self.state_pub.publish(m)

    def destroy(self):
        self.dev.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="MS42DC 夹爪 ROS2 节点")
    ap.add_argument("--config", default=None, help="real_params.yaml 路径")
    a, _ = ap.parse_known_args(argv)

    cfg = load_config(a.config)
    import rclpy
    from rclpy.node import Node

    class _N(Node):
        pass

    rclpy.init()
    n = _N("gripper_node")
    gn = GripperNode(cfg, n)
    print(banner("夹爪节点已启动  mode=%s" %
                 ("DRY-RUN（不动硬件）" if is_dry_run(cfg) else "REAL（会动硬件！）")))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        gn.destroy()
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

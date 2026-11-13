#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS42DC 二指电动夹爪 —— 串口协议底层（不依赖 ROS，可单独跑自测）

轮趣科技 Wheeltec MS42DC 步进电机驱控一体机械爪，手册 V3.0（2025/10/16）
协议（USB 串口 / TTL 通用，固定 11 字节，115200 8N1）：

  [0] 0x7B 帧头        [1] 设备地址(默认 0x01)
  [2] 0x02 位置模式    [3] 转向 1=顺时针=闭合 / 0=逆时针=张开
  [4] 0x20 32 细分     [5][6] 目标角度 ×10 大端（单位 0.1°）
  [7][8] 转速 ×10 大端（单位 0.1 rad/s）
  [9] 前 9 字节异或 BCC   [10] 0x7D 帧尾

反馈默认「请求式」：先发 CMD_QUERY 才回 9 字节：
  [addr, flag(1=到位), spd_h, spd_l, pos_hh, pos_hl, pos_lh, pos_ll, chk]
  角度 = 32 位大端 ÷ 10（度），转速 = 16 位大端 ÷ 10（rad/s）

⚠️ 硬件前置（最容易漏，手册原文强制）：
  1. 供电 10~25V（接口一 XH2.54 4Pin）
  2. USB-C 接电脑
  3. **拨码开关 2 号打到 ON** —— 否则 USB 串口收不到数据
  4. 拨码开关 1 号保持 OFF（烧录用）
  5. 不支持热插拔

⚠️ 本机只有【位置模式】，没有力控。夹持力靠"堵转保护"：
   让目标开口略小于物体宽度，手指顶到物体后电机堵转自动停。
   所以「物体宽度 → 目标角度」的映射必须标定（见 calibrate_gripper.py）。
"""

from __future__ import annotations

import math
import time
from typing import Optional, List, Tuple

HEAD = 0x7B
TAIL = 0x7D
MODE_POS = 0x02
DEFAULT_ADDR = 0x01
DEFAULT_MICROSTEP = 0x20      # 32 细分

# 手册现成帧（BCC 已逐字节复核）
CMD_CLOSE_QUICK = bytes([0x7b, 0x01, 0x02, 0x01, 0x20, 0x46, 0x50, 0x01, 0x2c, 0x62, 0x7d])
CMD_OPEN_QUICK = bytes([0x7b, 0x01, 0x02, 0x00, 0x20, 0x46, 0x50, 0x01, 0x2c, 0x63, 0x7d])
CMD_CLOSE_FULL = bytes([0x7b, 0x01, 0x02, 0x01, 0x20, 0x49, 0x20, 0x00, 0xc8, 0xf8, 0x7d])
CMD_OPEN_FULL = bytes([0x7b, 0x01, 0x02, 0x00, 0x20, 0x49, 0x20, 0x00, 0xc8, 0xf9, 0x7d])
CMD_QUERY = bytes([0x7b, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7a, 0x7d])


def bcc(data: bytes) -> int:
    """BCC 校验 = 所有前置字节的异或和"""
    v = 0
    for b in data:
        v ^= b
    return v


def build_cmd(addr: int, direction: int, angle_deg: float,
              speed_rads: float, microstep: int = DEFAULT_MICROSTEP) -> bytes:
    """拼一条 11 字节位置指令。direction: 1=闭合, 0=张开"""
    pos = int(round(angle_deg * 10))
    spd = int(round(speed_rads * 10))
    if not (0 <= pos <= 0xFFFF):
        raise ValueError("角度超限 %.1f°（单次最大 6553.5°）" % angle_deg)
    if not (0 <= spd <= 0xFFFF):
        raise ValueError("速度超限 %.1f rad/s" % speed_rads)
    body = [HEAD, addr, MODE_POS, direction, microstep,
            (pos >> 8) & 0xFF, pos & 0xFF,
            (spd >> 8) & 0xFF, spd & 0xFF]
    return bytes(body + [bcc(body), TAIL])


def parse_feedback(buf: bytes, addr: int = DEFAULT_ADDR) -> Optional[dict]:
    """从接收缓冲里找一帧合法反馈（9 字节）。找不到返回 None。

    注意手册表 3-15 第二个例子（8c a0）与正文「360 度」差 10 倍，
    疑为笔误；这里按 ÷10 处理（与第一个例子 0e 10 → 360° 自洽）。
    """
    if not buf or len(buf) < 9:
        return None
    for i in range(len(buf) - 8):
        f = buf[i:i + 9]
        if f[0] != addr or bcc(f[:8]) != f[8]:
            continue
        angle = ((f[4] << 24) | (f[5] << 16) | (f[6] << 8) | f[7]) / 10.0
        speed = ((f[2] << 8) | f[3]) / 10.0
        return {"reached": bool(f[1]), "angle_deg": angle,
                "speed_rads": speed, "raw": f.hex(" ")}
    return None


# ---------------------------------------------------------------- 开口映射
class OpeningMap:
    """电机角度(度) <-> 夹爪开口(mm)。带【空程】的分段线性模型。

        angle_deg = deadzone_deg + (max_width_mm - width_mm) * deg_per_mm
        width_mm  = max_width_mm - max(0, angle_deg - deadzone_deg)/deg_per_mm

    为什么要 deadzone_deg（2026-11-03 真机实测发现）
    ------------------------------------------------
    原来假设「从 home 开始，电机一转手指就动」，即强制过原点。
    但真机丝杠/齿轮有机械间隙：电机先空转约 105° 才真正带动手指，
    之后才是严格线性；接近最小开口又会撞机械限位。
    旧模型把这 105° 硬塞进斜率里 → 拟合残差 10mm 以上、deg_per_mm
    被系统性拉偏。现在改成上面的分段模型，实测 8 组数据几乎零残差。

    ⚠️ 标定时从 home 出发的【第一次闭合】要走 deadzone + Δ·k 度；
       之后连续闭合的增量里 deadzone 会相互抵消，只用 Δ·k（见 gripper_node）。
    """

    def __init__(self, max_width_mm: float = 120.0, deg_per_mm: float = 16.36,
                 deadzone_deg: float = 0.0, min_width_mm: float = 10.0,
                 calibrated: bool = False):
        self.max_width_mm = float(max_width_mm)
        self.deg_per_mm = float(deg_per_mm)
        self.deadzone_deg = float(deadzone_deg)
        self.min_width_mm = float(min_width_mm)
        self.calibrated = bool(calibrated)

    def width_to_angle(self, width_mm: float) -> float:
        """从 home 位置出发，走到目标开口需要的【累计角度】。"""
        w = max(self.min_width_mm, min(self.max_width_mm, float(width_mm)))
        a = self.deadzone_deg + (self.max_width_mm - w) * self.deg_per_mm
        return max(0.0, min(6553.5, a))

    def angle_to_width(self, angle_deg: float) -> float:
        """由累计角度反推开口（撞限位时钳到 min_width_mm）。"""
        moved = max(0.0, float(angle_deg) - self.deadzone_deg)
        w = self.max_width_mm - moved / self.deg_per_mm
        return max(self.min_width_mm, min(self.max_width_mm, w))

    def incremental_angle(self, from_width_mm: float, to_width_mm: float) -> float:
        """从 from 走到 to 要发的【增量角度】（deadzone 在差分中抵消）。"""
        return abs(float(from_width_mm) - float(to_width_mm)) * self.deg_per_mm

    @classmethod
    def from_two_points(cls, w1: float, a1: float, w2: float, a2: float,
                        deadzone_deg: float = 0.0) -> "OpeningMap":
        """用两组「实量开口 mm / 电机角度°」求斜率（两点法无法求空程，需外部给定）。"""
        if abs(w1 - w2) < 1e-6:
            raise ValueError("两次标定的开口必须不同")
        k = (a2 - a1) / (w2 - w1)        # 度 / mm（开口变小 → 角度变大，故为负）
        if k >= 0:
            raise ValueError("斜率应为负（开口越小角度越大），得到 %.4f，检查输入顺序" % k)
        k = -k
        # angle = deadzone + (max_width - width) * k，令 width=w1 时 angle=a1
        max_width = w1 + (a1 - deadzone_deg) / k
        m = cls(max_width_mm=max_width, deg_per_mm=k,
                deadzone_deg=deadzone_deg, calibrated=True)
        err = abs(m.width_to_angle(w2) - a2)
        m._fit_error_deg = err           # type: ignore[attr-defined]
        return m

    def fit_error(self) -> float:
        return float(getattr(self, "_fit_error_deg", float("nan")))


# ---------------------------------------------------------------- 串口封装
class Ms42dc:
    """真实串口设备。dry_run=True 时完全不碰串口，用内存模拟。"""

    def __init__(self, port: str = "/dev/motor_serial", baud: int = 115200,
                 addr: int = DEFAULT_ADDR, speed_rads: float = 20.0,
                 microstep: int = DEFAULT_MICROSTEP, dry_run: bool = False,
                 timeout: float = 0.5):
        self.port, self.baud, self.addr = port, baud, addr
        self.speed_rads, self.microstep = speed_rads, microstep
        self.dry_run = dry_run
        self.timeout = timeout
        self._ser = None
        self._angle = 0.0                 # dry-run 里模拟的电机角度
        self._reached = True
        self.last_cmd = None
        if not dry_run:
            self._open()

    def _open(self) -> None:
        try:
            import serial
        except ImportError:
            raise RuntimeError("缺少 pyserial：pip3 install pyserial")
        try:
            self._ser = serial.Serial(
                port=self.port, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=self.timeout)
        except Exception as e:                     # noqa: BLE001
            # 现场最常见的两个原因，直接说清楚，别让人对着 traceback 猜
            raise RuntimeError(
                "打不开串口 %s：%s\n"
                "  ① 被占用：gripper_node / gripper_web 还开着？"
                "先 Ctrl+C 停掉整机 launch 再跑标定（串口独占）\n"
                "  ② 权限/设备名：ls -l /dev/motor_serial；"
                "没这个名就 ls /dev/ttyUSB* 并把 gripper.port 改成实际设备\n"
                "  ③ 拨码开关 2 号没打 ON（1 号保持 OFF）时串口收不到数据"
                % (self.port, e))
        time.sleep(0.1)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def _write(self, frame: bytes) -> None:
        self.last_cmd = frame
        if self.dry_run:
            # 模拟【累计位置】：协议角度是相对增量，闭合(+)/张开(-)累加。
            # 这样 dry-run 也能演练「回读差分」的标定逻辑（真实反馈是累计值）。
            if len(frame) == 11 and frame[2] == MODE_POS:
                delta = ((frame[5] << 8) | frame[6]) / 10.0
                self._angle += delta if frame[3] == 1 else -delta
                self._reached = False
            return
        self._ser.reset_input_buffer()
        self._ser.write(frame)

    def move_angle(self, angle_deg: float, direction: int) -> bytes:
        """下发一条【相对角度增量】指令（这次要走多少度，不是绝对位置）。

        精度说明：角度字段单位是 0.1°（×10 后填 2 字节），300° 精确写成 3000；
        32 细分下步进分辨率约 0.056°/微步，远细于指令分辨率。
        所以「发得准不准」不是瓶颈，会不会【丢步/提前堵转】才是 —— 必须回读确认。
        """
        frame = build_cmd(self.addr, direction, angle_deg, self.speed_rads,
                          self.microstep)
        self._write(frame)
        return frame

    def move_to_width(self, target_width_mm: float, omap: OpeningMap,
                      current_width_mm: float) -> bytes:
        """把开口从 current_width_mm 调到 target_width_mm。

        协议是【相对角度增量】：每次发的角度是"这次要走多少度"，不是绝对位置。
        闭合（target < current）用 direction_close，张开反之。
        """
        delta_mm = float(current_width_mm) - float(target_width_mm)
        delta_deg = abs(delta_mm) * omap.deg_per_mm
        direction = 1 if delta_mm > 0 else 0    # 闭合=1
        return self.move_angle(delta_deg, direction)

    def query(self) -> Optional[dict]:
        """请求式反馈：先发 CMD_QUERY 再读。"""
        self._write(CMD_QUERY)
        if self.dry_run:
            self._reached = True
            return {"reached": True, "angle_deg": self._angle,
                    "speed_rads": 0.0, "raw": "dry-run"}
        time.sleep(0.05)
        data = self._ser.read(64)
        return parse_feedback(data, self.addr)

    def send_raw(self, frame: bytes) -> None:
        """调试用：直接发一帧现成字节。"""
        self._write(frame)

    @staticmethod
    def scan_ports() -> List[str]:
        try:
            import serial.tools.list_ports
            return [p.device for p in serial.tools.list_ports.comports()]
        except ImportError:
            return []


# ---------------------------------------------------------------- 自测
def selftest() -> bool:
    """纯离线自测：协议帧 / BCC / 反馈解析 / 开口映射。不需要硬件。"""
    ok = True

    def check(name: str, cond: bool, extra: str = "") -> None:
        nonlocal ok
        print("  %s %s %s" % ("✅" if cond else "❌", name, extra))
        ok = ok and cond

    print("[1] 手册现成帧 BCC 复核")
    for name, f in (("闭合快速", CMD_CLOSE_QUICK), ("张开快速", CMD_OPEN_QUICK),
                    ("完全闭合", CMD_CLOSE_FULL), ("完全张开", CMD_OPEN_FULL),
                    ("查询状态", CMD_QUERY)):
        check(name, len(f) == 11 and f[0] == HEAD and f[10] == TAIL
              and bcc(f[:9]) == f[9], f.hex(" "))

    print("[2] build_cmd 与手册第 24 页级联示例逐字节比对")
    f = build_cmd(1, 1, 90.0, 10.0)
    expect = bytes([0x7b, 0x01, 0x02, 0x01, 0x20, 0x03, 0x84, 0x00, 0x64, 0xba, 0x7d])
    check("build_cmd(1,1,90,10)", f == expect, f.hex(" "))

    print("[3] 反馈帧解析")
    body = bytes([0x01, 0x01, 0x00, 0x0a, 0x00, 0x00, 0x0e, 0x10])  # 360° / 1.0 rad/s
    frame = body + bytes([bcc(body)])
    fb = parse_feedback(frame)
    check("解析 360° / 1.0 rad/s / 到位",
          fb is not None and abs(fb["angle_deg"] - 360.0) < 1e-6
          and abs(fb["speed_rads"] - 1.0) < 1e-6 and fb["reached"],
          str(fb))

    print("[4] 开口映射与两点标定")
    m = OpeningMap(max_width_mm=120.0, deg_per_mm=16.36)
    check("120mm -> 0°", abs(m.width_to_angle(120.0)) < 1e-9)
    check("10mm -> 1799.6°", abs(m.width_to_angle(10.0) - 1799.6) < 0.01)
    check("往返一致", abs(m.angle_to_width(m.width_to_angle(47.0)) - 47.0) < 1e-6)
    m2 = OpeningMap.from_two_points(100.0, 300.0, 40.0, 1290.0)
    check("两点标定拟合误差 < 1e-6", m2.fit_error() < 1e-6 and m2.calibrated)

    print("[5] dry-run 串口（不碰硬件）")
    d = Ms42dc(dry_run=True)
    d.move_to_width(40.0, m, 90.0)
    check("dry-run 能下发且不打开串口", d._ser is None and d.last_cmd is not None,
          d.last_cmd.hex(" "))
    q = d.query()
    check("dry-run 查询有回包", q is not None and q["reached"])
    d.close()
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)

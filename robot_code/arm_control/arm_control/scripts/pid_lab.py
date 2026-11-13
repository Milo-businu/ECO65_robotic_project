#!/usr/bin/env python3
"""pid_lab.py —— P3 第一步：PID 控制器 + 单关节动力学仿真

被控对象：J·θ̈ + b·θ̇ + m·g·L·cos(θ) = τ
控制律：  τ = Kp·e + Ki·∫e + Kd·ė + 重力前馈(mgl·cos θ)
通过四组参数对比，看清 P/I/D/前馈各自的作用。
"""
import math

class PIDController:
    """单关节 PID：抗积分饱和(i_max) + 输出限幅(out_max)"""
    def __init__(self, kp, ki, kd, dt, out_max, i_max=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.out_max = out_max
        self.i_max = i_max
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target, measured, feedforward=0.0):
        e = target - measured
        self.integral += e * self.dt
        self.integral = max(-self.i_max, min(self.i_max, self.integral))  # 抗积分饱和
        d = (e - self.prev_error) / self.dt
        self.prev_error = e
        u = self.kp * e + self.ki * self.integral + self.kd * d + feedforward
        return max(-self.out_max, min(self.out_max, u))                   # 力矩限幅

def simulate(kp, ki, kd, use_ff=False, duration=4.0, dt=0.001, target=0.3):
    """阶跃响应仿真，返回稳态误差 / 超调 / 稳定耗时"""
    J, b, mgl, tau_max = 0.5, 0.8, 5.7, 30.0
    pid = PIDController(kp, ki, kd, dt, out_max=tau_max)
    theta = omega = 0.0
    n = int(duration / dt)
    step_at = int(0.5 / dt)
    peak, settle = 0.0, None
    for i in range(n):
        tgt = target if i >= step_at else 0.0
        ff = mgl * math.cos(theta) if use_ff else 0.0
        tau = pid.compute(tgt, theta, ff)
        alpha = (tau - b * omega - mgl * math.cos(theta)) / J
        omega += alpha * dt
        theta += omega * dt
        if i >= step_at:
            peak = max(peak, theta)
            if settle is None and abs(tgt - theta) < 0.02 * abs(tgt):
                settle = i * dt - 0.5
    final = theta
    return dict(final=final,
                steady_err=abs(target - final),
                overshoot=max(0.0, (peak - target) / target * 100),
                settle=settle)

def main():
    cases = [
        ("① 只 P  (Kp=20)",            20, 0, 0, False),
        ("② P+D   (Kp=20,Kd=2)",       20, 0, 2, False),
        ("③ P+I+D (Kp=20,Ki=5,Kd=2)",  20, 5, 2, False),
        ("④ 同上 + 重力前馈",           20, 5, 2, True),
    ]
    print(f"{'方案':<26}{'稳态误差(rad)':>13}{'超调(%)':>10}{'稳定耗时(s)':>12}")
    print("-" * 62)
    for name, kp, ki, kd, ff in cases:
        r = simulate(kp, ki, kd, ff)
        st = f"{r['settle']:.2f}" if r['settle'] is not None else "未稳定"
        print(f"{name:<26}{r['steady_err']:>13.5f}{r['overshoot']:>10.1f}{st:>12}")
    print("\n对照：目标 0.3 rad；③ 应基本无静差，④ 应最快。")

if __name__ == '__main__':
    main()

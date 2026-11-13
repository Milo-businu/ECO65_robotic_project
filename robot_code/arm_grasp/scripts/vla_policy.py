#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""VLA 桥接（OpenPI π0.5-DROID）—— 高层策略，可插拔

接口与 recovery/vla_server.py 完全一致：
  POST /predict
    {"prompt", "wrist"(base64 JPEG), "joint_position":[7], "gripper_position":[1]}
  → {"shape":[15,8], "actions":[[...8...] x 15]}

⚠️⚠️ 必须说清楚的架构缺口（17 号文档第 7 节）
  π0 预训练时用的是「第三人称场景视角 + 腕部视角」双图。
  本项目只有腕装相机（Eye-in-Hand），没有固定场景相机。
  vla_server.py 会把腕部图同时塞进 exterior_left/right 槽位，
  **只能验证接口通不通，不能保证抓取效果**。
  想要 VLA 真正可用，加一台几十块的固定 USB 摄像头当 base_0_rgb。

本模块只做「HTTP 调用 + 动作解析」，不涉及任何 ROS，方便离线自测。
用 urllib（标准库），不引入 requests 依赖。
"""

from __future__ import annotations

import base64
import json
import urllib.request
from typing import Optional

import numpy as np


class VLAPolicy:
    def __init__(self, cfg: dict):
        v = cfg["vla"]
        self.url = v["url"]
        self.prompt = v.get("prompt", "pick up the green cube")
        self.timeout = float(v.get("timeout_s", 10.0))
        self.enabled = bool(v.get("enabled", False))

    def health(self) -> dict:
        url = self.url.rsplit("/predict", 1)[0] + "/health"
        with urllib.request.urlopen(url, timeout=3.0) as r:
            return json.loads(r.read().decode("utf-8"))

    def predict(self, wrist_b64: Optional[str], joints, gripper_mm: float,
                prompt: Optional[str] = None) -> Optional[np.ndarray]:
        """返回 actions (N,8)；失败返回 None（调用方应回退到 geometric 策略）。"""
        payload = {
            "prompt": prompt or self.prompt,
            "exterior_left": None,
            "exterior_right": None,
            "wrist": wrist_b64,
            "joint_position": [float(x) for x in joints] + [0.0],   # 6 关节 + 1 占位 = 7
            "gripper_position": [float(gripper_mm) / 100.0],          # mm → 归一化粗映射
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            out = json.loads(r.read().decode("utf-8"))
        if "actions" not in out:
            return None
        return np.asarray(out["actions"], dtype=float)

    @staticmethod
    def action_to_delta(actions: np.ndarray, step: int = 0,
                        scale: float = 0.02) -> np.ndarray:
        """把一行 action 粗映射成 [dx,dy,dz,gripper]（末端增量 + 夹爪）。

        ⚠️ 这是「接口联调用」的近似映射，不是 π0 的官方动作语义。
           真要用 VLA 控制，必须按你微调时的数据格式重写这一段。
        """
        a = np.asarray(actions, float)[step]
        d = np.zeros(4)
        d[:3] = a[:3] * scale
        d[3] = float(a[6]) if len(a) > 6 else 0.0
        return d


def selftest() -> bool:
    """不连服务，只验证动作解析逻辑。"""
    print("  ✅ VLAPolicy 模块可导入（urllib 标准库，无第三方依赖）")
    fake = np.zeros((15, 8))
    fake[0, :3] = [0.5, -0.25, 1.0]
    fake[0, 6] = 0.8
    d = VLAPolicy.action_to_delta(fake)
    ok = np.allclose(d[:3], [0.01, -0.005, 0.02]) and abs(d[3] - 0.8) < 1e-9
    print("  %s 动作 → 末端增量解析  %s" % ("✅" if ok else "❌", np.round(d, 5).tolist()))
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)

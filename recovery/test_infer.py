#!/usr/bin/env python3
"""test_infer.py —— VLA 单次推理自测（不依赖 ROS2，不依赖相机）

用途：装完 OpenPI + 权重后，先跑这个确认模型能出动作块，再起服务。
期望输出：actions shape = (15, 8)

用法：
  cd ~/openpi && \
  MALLOC_CONF=background_thread:false ARROW_DEFAULT_MEMORY_POOL=system \
  ~/miniconda3/envs/openpi/bin/python -u test_infer.py
"""
import pyarrow  # noqa: F401  —— 必须第一行

import os
import time

import numpy as np

CONFIG_NAME = os.environ.get("OPENPI_CONFIG", "pi05_droid")
CKPT_DIR = os.environ.get("OPENPI_CKPT",
                          os.path.expanduser("~/openpi/checkpoints/pi05_droid"))


def main():
    from openpi.training import config as _config
    from openpi.policies import policy_config

    print(f"config={CONFIG_NAME}  ckpt={CKPT_DIR}", flush=True)
    cfg = _config.get_config(CONFIG_NAME)
    t0 = time.time()
    policy = policy_config.create_trained_policy(cfg, CKPT_DIR)
    print(f"策略加载完成，耗时 {time.time() - t0:.1f}s", flush=True)

    # 用随机图当占位（真实使用时换成相机图）
    img = np.random.randint(0, 255, size=(224, 224, 3), dtype=np.uint8)
    obs = {
        "observation/exterior_image_1_left": img,
        "observation/exterior_image_1_right": img,
        "observation/wrist_image_left": img,
        "observation/joint_position": np.zeros(7, dtype=np.float32),
        "observation/gripper_position": np.zeros(1, dtype=np.float32),
        "prompt": "pick up the green cube",
    }

    t0 = time.time()
    out = policy.infer(obs)
    actions = np.asarray(out["actions"])
    print(f"推理耗时 {time.time() - t0:.1f}s", flush=True)
    print(f"actions shape = {actions.shape}", flush=True)
    print(f"首步动作 = {np.round(actions[0], 4)}", flush=True)

    assert actions.shape[0] == 15, f"期望 15 步，实际 {actions.shape[0]}"
    print("OK：推理链路通（首次调用含模型预热，约 30-40s 属正常）")


if __name__ == "__main__":
    main()

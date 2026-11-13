"""test_infer.py —— P5 里程碑：VLA 模型第一次推理
给模型随机图像 + 一句话指令，验证「加载 → 推理 → 输出动作」整条链路。
（随机图像 → 动作无实际意义；本测试只验证链路通不通。）
"""
import pyarrow  # P5 真凶修复：必须最先导入，规避 pyarrow(jemalloc后台线程)与 CUDA 库加载顺序的竞态段错误
import numpy as np
from openpi.training import config as _config
from openpi.policies import policy_config
from openpi.shared import download

print("[1/3] 读取配置（pi05_droid）...")
config = _config.get_config("pi05_droid")

print("[2/3] 加载权重（第一次要几分钟，耐心等）...")
ckpt = download.maybe_download("gs://openpi-assets/checkpoints/pi05_droid")
policy = policy_config.create_trained_policy(config, ckpt)
print("      模型加载完成 ✓")

print("[3/3] 推理（随机图像 + 指令）...")
example = {
    "observation/exterior_image_1_left":  np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8),
    "observation/exterior_image_1_right": np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8),
    "observation/wrist_image_left":       np.random.randint(0, 255, (224, 224, 3), dtype=np.uint8),
    "observation/joint_position": np.zeros(7),
    "observation/gripper_position": np.zeros(1),
    "prompt": "pick up the green cube",
}
result = policy.infer(example)
print("动作输出形状:", result["actions"].shape)
print("第一个动作向量(前5维):", result["actions"][0][:5])
print("=== P5 里程碑达成：模型第一次思考成功 ===")

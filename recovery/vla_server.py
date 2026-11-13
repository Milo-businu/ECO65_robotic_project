#!/usr/bin/env python3
"""vla_server.py —— π0.5-DROID 推理服务（OpenPI，Python 3.11 侧）

用途：旧服务器到期释放后重建的 VLA 服务。接口与原来完全一致，
      ROS2 侧（Python 3.8）的 grasp_closed_loop.py / http_client_test.py 不用改。

接口：
  GET  /health  -> {"status": "ready"}
  POST /predict
      请求体：{"prompt": str,
               "exterior_left":  base64 JPEG 或 null,
               "exterior_right": base64 JPEG 或 null,
               "wrist":          base64 JPEG,
               "joint_position": [7 个 float],   # 6 关节 + 1 占位
               "gripper_position": [1 个 float]}
      返回：  {"shape": [15, 8], "actions": [[...8...] x 15]}

启动（三件套环境变量缺一可能段错误）：
  cd ~/openpi && \
  MALLOC_CONF=background_thread:false ARROW_DEFAULT_MEMORY_POOL=system \
  ~/miniconda3/envs/openpi/bin/python -u vla_server.py

注意：import pyarrow 必须放在文件最前面（pyarrow 的 jemalloc 后台线程会 segfault）。
"""
import pyarrow  # noqa: F401  —— 必须第一行，别挪走

import base64
import io
import os
import logging

import numpy as np
from PIL import Image

from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("vla_server")

# ---------------- 配置 ----------------
CONFIG_NAME = os.environ.get("OPENPI_CONFIG", "pi05_droid")
CKPT_DIR = os.environ.get("OPENPI_CKPT",
                          os.path.expanduser("~/openpi/checkpoints/pi05_droid"))
PORT = int(os.environ.get("OPENPI_PORT", "8000"))
IMG_SIZE = 224

POLICY = None


# ---------------- 工具 ----------------
def decode_image(b64, tag):
    """base64 JPEG -> uint8 (224,224,3) RGB。None/失败则退回全黑图。"""
    if not b64:
        log.warning(f"[{tag}] 图像为空，用全黑图代替")
        return np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
    try:
        raw = base64.b64decode(b64)
        im = Image.open(io.BytesIO(raw)).convert("RGB").resize(
            (IMG_SIZE, IMG_SIZE), Image.BILINEAR)
        return np.asarray(im, dtype=np.uint8)
    except Exception as e:
        log.warning(f"[{tag}] 图像解码失败({e})，用全黑图代替")
        return np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)


def fix_vec(v, n, default=0.0):
    """把输入的关节/夹爪向量补齐或截断到 n 维。"""
    if v is None:
        return np.array([default] * n, dtype=np.float32)
    v = [float(x) for x in v]
    if len(v) < n:
        v = v + [default] * (n - len(v))
    return np.array(v[:n], dtype=np.float32)


# ---------------- 请求体 ----------------
class PredictReq(BaseModel):
    prompt: str = "pick up the green cube"
    exterior_left: str | None = None
    exterior_right: str | None = None
    wrist: str | None = None
    joint_position: list | None = None
    gripper_position: list | None = None


# ---------------- 服务 ----------------
app = FastAPI(title="openpi-vla-server")


@app.on_event("startup")
def _load():
    global POLICY
    log.info(f"加载策略: config={CONFIG_NAME} ckpt={CKPT_DIR}")
    from openpi.training import config as _config
    from openpi.policies import policy_config
    cfg = _config.get_config(CONFIG_NAME)
    # 注意：传 config 对象，不是模块（当年踩过的坑）
    POLICY = policy_config.create_trained_policy(cfg, CKPT_DIR)
    log.info("策略加载完成")


@app.get("/health")
def health():
    return {"status": "ready" if POLICY is not None else "loading",
            "config": CONFIG_NAME, "ckpt": CKPT_DIR}


@app.post("/predict")
def predict(req: PredictReq):
    if POLICY is None:
        return {"error": "policy not loaded yet"}
    wrist = decode_image(req.wrist, "wrist")
    left = decode_image(req.exterior_left or req.wrist, "exterior_left")
    right = decode_image(req.exterior_right or req.wrist, "exterior_right")

    obs = {
        "observation/exterior_image_1_left": left,
        "observation/exterior_image_1_right": right,
        "observation/wrist_image_left": wrist,
        "observation/joint_position": fix_vec(req.joint_position, 7),
        "observation/gripper_position": fix_vec(req.gripper_position, 1),
        "prompt": req.prompt,
    }
    out = POLICY.infer(obs)                      # -> {"actions": (15, 8)}
    actions = np.asarray(out["actions"], dtype=np.float64)
    log.info(f"推理完成: {actions.shape}")
    return {"shape": list(actions.shape), "actions": actions.tolist()}


if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=PORT, log_level="info")

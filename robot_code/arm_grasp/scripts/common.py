#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""公共工具：配置加载、路径解析、手眼矩阵、日志着色

所有脚本都从这里读配置，保证「换机器只改 real_params.yaml 一处」。
"""

from __future__ import annotations

import os
import sys
from typing import Optional

import numpy as np

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "real_params.yaml")


def load_config(path: Optional[str] = None) -> dict:
    """读 real_params.yaml。找不到就直接报错，不静默用默认值（避免悄悄跑错机器）。"""
    try:
        import yaml
    except ImportError:
        raise RuntimeError("缺少 PyYAML：sudo apt install python3-yaml 或 pip3 install pyyaml")
    p = path or os.environ.get("REAL_PARAMS") or DEFAULT_CONFIG
    p = os.path.expanduser(p)
    if not os.path.isfile(p):
        raise FileNotFoundError("找不到配置文件 %s" % p)
    with open(p, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    required = ["profile", "robot", "arm", "camera", "handeye",
                "perception", "gripper", "task", "vla"]
    missing = [k for k in required if k not in cfg]
    if missing:
        raise ValueError("配置缺少顶层段：%s" % missing)
    return cfg


def setup_import_paths(cfg: dict) -> str:
    """把 arm_control/scripts 加进 sys.path，让纯 numpy 的 ik_check 可导入。

    返回实际使用的目录；找不到抛错（不静默降级）。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.expanduser("~/robot_vla_ws/src/arm_control/scripts"),
        os.path.expanduser("~/robot_vla_ws/robot_code/arm_control/scripts"),
        os.path.abspath(os.path.join(here, "..", "..", "arm_control", "scripts")),
    ]
    for d in candidates:
        if os.path.isfile(os.path.join(d, "ik_check.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            return d
    raise ImportError(
        "找不到 arm_control/scripts/ik_check.py（正/逆运动学库）。"
        "确认已按 README把仓库放到 ~/robot_vla_ws/")


def find_urdf(cfg: dict) -> str:
    """优先用【随包自带】的 URDF —— 这样 clone 下来就能跑，不依赖 /tmp 临时文件。

    cfg 可以传完整配置，也可以只传 robot 段。
    """
    r = cfg.get("robot", cfg) if isinstance(cfg, dict) else {}
    p = (r.get("urdf") if isinstance(r, dict) else "") or os.environ.get("ECO65_URDF", "")
    if p and os.path.isfile(os.path.expanduser(p)):
        return os.path.expanduser(p)
    bundled = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "eco65_6fb.urdf")
    for c in (bundled,
              "/tmp/eco65_6fb.urdf",
              os.path.expanduser("~/robot_vla_ws/src/sim_grasp/config/eco65_6fb.urdf"),
              os.path.expanduser("~/robot_vla_ws/robot_code/sim_grasp/config/eco65_6fb.urdf")):
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        "找不到 ECO65 URDF。包内 config/eco65_6fb.urdf 应该随仓库一起 clone 下来；"
        "若确实没有，在真机跑 `urdf` 别名生成 /tmp/eco65_6fb.urdf，"
        "或在 real_params.yaml 的 robot.urdf 填绝对路径")


def quat_wxyz_to_R(q) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def handeye_matrix(cfg: dict, allow_fallback: bool = False) -> np.ndarray:
    """返回 X = T_flange←camera_optical（4x4）。

    ⚠️ 未标定（calibrated: false）时默认【拒绝】，这是故意的：
        拿安装估计值去真机抓取 = 用错误坐标撞东西。
        dry-run 演练才允许 allow_fallback=True。
    """
    h = cfg["handeye"]
    M = h.get("matrix_4x4")
    if M:
        X = np.array(M, dtype=float)
        if X.shape != (4, 4):
            raise ValueError("handeye.matrix_4x4 必须是 4 行 × 4 列")
        return X
    if not h.get("calibrated", False) and not allow_fallback:
        raise RuntimeError(
            "手眼标定未完成（handeye.calibrated: false）。\n"
            "  请按 25 号清单采集 → python3 handeye_calib.py 求解 → 把 4x4 矩阵填进\n"
            "  config/real_params.yaml 的 handeye.matrix_4x4，并把 calibrated 改成 true。\n"
            "  ⚠️ 未标定就抓真机，等于闭着眼睛伸手，必撞。")
    X = np.eye(4)
    X[:3, :3] = quat_wxyz_to_R(h.get("fallback_quat_wxyz", [1, 0, 0, 0]))
    X[:3, 3] = np.asarray(h.get("fallback_xyz", [0, 0, 0]), float)
    return X


def is_dry_run(cfg: dict) -> bool:
    return str(cfg["profile"].get("mode", "dry-run")).lower() != "real"


# ---------------------------------------------------------------- 线缆包络
def cable_envelope(cfg: dict):
    """返回 (lo, hi) 两个 (3,) 数组，表示【法兰/Link6】允许待的长方体；没配就返回 None。

    2026-11-09 真机实测（用户确认）：连接线太短，法兰只能待在
        前方 x ∈ [0, 0.50]m（从基座中心量）
        左右 y ∈ [-0.20, 0.20]m
        高度 z ∈ [0, 0.50]m（从台面量，臂直接装在台面上）
    且 **J1 转动时这个盒子不跟着转** —— 所以它是基座系里的固定长方体，
    不是圆柱。超出去就是拽线。

    ⚠️ 约束对象是【法兰】，不是指尖。工具竖直向下时法兰比指尖高 0.177m，
       也就是说「指尖能到 0.50m」是错的，实际指尖只能到 0.32m 左右。
    """
    s = (cfg or {}).get("safety") or {}
    e = s.get("cable_envelope_m")
    if not e:
        return None
    try:
        lo = np.array([float(e["x"][0]), float(e["y"][0]), float(e["z"][0])])
        hi = np.array([float(e["x"][1]), float(e["y"][1]), float(e["z"][1])])
    except Exception:                              # noqa: BLE001
        return None
    return lo, hi


def safe_window(cfg: dict):
    """相对 home 法兰锚点的安全窗口 (up, out, side)；没配返回 None。

    现场依据（2026-11-09）：相机 USB 线只能在身前垂圈，往上抬/往外伸会把圈拉直。
    抓取任务全程在锚点下方，所以"往下"不限，只限三个方向。
    """
    w = ((cfg or {}).get("safety") or {}).get("safe_window_m")
    if not w:
        return None
    try:
        return (float(w["up"]), float(w["out"]), float(w["side"]))
    except Exception:                              # noqa: BLE001
        return None


def flange_position(kin, q) -> np.ndarray:
    """法兰（tip_frame，本项目是 Link6）在基座系的位置。"""
    return np.asarray(kin.fk(np.asarray(q, float)), float)[:3, 3]



def banner(title: str) -> str:
    return "\n" + "=" * 62 + "\n  " + title + "\n" + "=" * 62


def fmt(v, n: int = 4) -> str:
    return np.round(np.asarray(v, float), n).tolist()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""brain.py —— 端到端识别「大脑」客户端（可选，默认关闭）

定位（先讲清楚，避免误解）
--------------------------
  它**不是**替代 IK/夹爪的 VLA 控制器，而是把「目标在哪」这一步从
  "颜色阈值"换成"开放词汇检测"。分工仍然是：

      brain：图 → 一个框 + 一个名字（"绿方块"/"苹果"/"红色积木"）
      几何：框中心 + 对齐深度 + FK·X → 基座系 3D 点（这套已经验过）

  为什么这么切：让 VLM 直接吐 3D 坐标/动作，精度远不如"2D 框 + 深度反投影"，
  而且一旦它错你没法查。把大模型限制在它最擅长的事（认东西、给框）上，
  误差链才是可控的、可复算的。

为什么是"服务端 + HTTP"
----------------------
  真机是 Ubuntu 20.04 + ROS2 Foxy + Python 3.8，主流推理框架早就不支持 3.8 了。
  所以模型跑在独立环境（conda py3.10 / Ollama / LM Studio / vLLM），
  机器人这边只发一张 JPEG、收一段 JSON。好处：
    · 不动 ROS 环境，不装 torch
    · 换模型 = 改一行 url/model，不用重装系统
    · 模型崩了不影响机械臂（有 fallback_to_color）

3070 8GB 上的现实选择（2026-11 检索，标注为【外验】）
----------------------------------------------------
  · Qwen2.5-VL-3B-Instruct，Q4_K_M：模型 ~1.9GB + mmproj ~1.3GB ≈ **5~6GB 显存**
    → 8GB 能跑，且留得下相机 pipeline。**首选**。
  · Qwen2.5-VL-7B Q4_K_M ≈ 7.1GB，占 89~90% 显存 → 标定/抓取时不建议同机跑。
  · Florence-2-large(0.77B) 全精度 ~1.5GB：开放词汇检测更专一、更快，
    但只会给框、不会"讲"。要"大脑"选上面那个，只要框可选它。
  ⚠️ 以上是外部资料，本机未实测；跑起来后用 `--check` 看延迟再决定用哪个。

用法
----
  # 自检：能不能连通、返回什么、耗时多少（不需要机械臂）
  python3 robot_code/arm_grasp/scripts/brain.py --check --image /tmp/test.jpg
  python3 robot_code/arm_grasp/scripts/brain.py --check          # 用相机当前帧

  # 在配置里切换（perception 段）
  #   target_source: brain      # color | brain
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEFAULT_PROMPT = (
    "You are a robot perception module. Find the {target} in this image. "
    "Reply with ONLY one JSON object, no markdown, no explanation: "
    '{{"found": true, "bbox_2d": [x1, y1, x2, y2], "label": "short name"}} '
    "where the box tightly encloses the object in PIXEL coordinates of this "
    'image (image size {w}x{h}). If the object is absent reply '
    '{{"found": false, "bbox_2d": [], "label": ""}}.'
)


def _extract_json(text):
    """从模型回复里挖出第一个 JSON 对象（模型经常夹带 markdown/解释）。"""
    if not text:
        return None
    m = re.search(r"\{.*?\}", text, re.S)
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except Exception:                              # noqa: BLE001
        return None


def _to_pixels(bbox, w, h):
    """兼容三种常见坐标约定：像素 / 0~1 归一化 / 0~1000 归一化（Qwen 风格）。"""
    b = [float(v) for v in bbox[:4]]
    mx = max(b)
    if mx <= 1.001:
        return [b[0] * w, b[1] * h, b[2] * w, b[3] * h]
    if mx <= 1000.0 and (w > 1000 or h > 1000):
        return [b[0] / 1000.0 * w, b[1] / 1000.0 * h,
                b[2] / 1000.0 * w, b[3] / 1000.0 * h]
    return b


class BrainClient:
    def __init__(self, cfg, log=print):
        b = cfg.get("brain") or {}
        p = cfg.get("perception") or {}
        self.cfg = cfg
        self.log = log
        self.enabled = bool(b.get("enabled", False))
        self.url = str(b.get("url", "http://127.0.0.1:11434/v1/chat/completions"))
        self.model = str(b.get("model", "qwen2.5vl:3b"))
        self.timeout = float(b.get("timeout_s", 8.0))
        self.target = str(p.get("target_name") or b.get("target_name") or "green cube")
        self.prompt_t = str(b.get("prompt") or DEFAULT_PROMPT)
        self.last_err = ""
        self.last_ms = 0.0
        self.last_label = ""

    # ------------------------------------------------------------ HTTP
    def _post(self, payload):
        req = urllib.request.Request(
            self.url, data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"})
        t0 = time.time()
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            data = json.loads(r.read().decode("utf-8"))
        self.last_ms = (time.time() - t0) * 1000.0
        return data

    def _chat(self, img_b64, w, h, target=None):
        prompt = self.prompt_t.format(target=target or self.target, w=w, h=h)
        payload = {
            "model": self.model,
            "temperature": 0.0,
            "max_tokens": 160,
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": "data:image/jpeg;base64," + img_b64}},
                ],
            }],
        }
        data = self._post(payload)
        try:
            return data["choices"][0]["message"]["content"]
        except Exception:                          # noqa: BLE001
            # 有些本地服务把回复放在 response/message 里
            for k in ("response", "message", "text"):
                if isinstance(data.get(k), str):
                    return data[k]
            raise RuntimeError("看不懂服务端返回：%s" % str(data)[:200])

    # ------------------------------------------------------------ 对外
    def detect_box(self, img_rgb, target=None):
        """返回 (box(x1,y1,x2,y2) | None, label, 说明)。img_rgb 是 (h,w,3) uint8。"""
        if not self.enabled:
            return None, "", "brain.enabled=false"
        try:
            import cv2
        except ImportError:
            return None, "", "缺 cv2，无法编码 JPEG"
        h, w = img_rgb.shape[:2]
        ok, enc = cv2.imencode(".jpg", img_rgb[:, :, ::-1],
                               [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return None, "", "JPEG 编码失败"
        b64 = base64.b64encode(enc.tobytes()).decode("ascii")
        try:
            text = self._chat(b64, w, h, target)
        except urllib.error.URLError as e:
            self.last_err = "连不上 %s：%s" % (self.url, e)
            return None, "", self.last_err
        except Exception as e:                     # noqa: BLE001
            self.last_err = "%s: %s" % (type(e).__name__, e)
            return None, "", self.last_err

        obj = _extract_json(text)
        if not obj or not obj.get("found") or not obj.get("bbox_2d"):
            self.last_err = "模型说没找到（回复：%s）" % (text or "")[:120]
            return None, "", self.last_err
        box = _to_pixels(obj["bbox_2d"], w, h)
        box[0] = max(0.0, min(box[0], w - 1))
        box[1] = max(0.0, min(box[1], h - 1))
        box[2] = max(box[0] + 1, min(box[2], w))
        box[3] = max(box[1] + 1, min(box[3], h))
        self.last_label = str(obj.get("label", ""))[:24]
        self.last_err = ""
        return box, self.last_label, "OK %.0fms" % self.last_ms

    def box_mask(self, shape, box, shrink=0.08):
        """把框转成 bool mask（向内收一点，减少背景像素混进来）。"""
        h, w = shape[:2]
        x1, y1, x2, y2 = box
        dx, dy = (x2 - x1) * shrink, (y2 - y1) * shrink
        x1, x2 = int(x1 + dx), int(x2 - dx)
        y1, y2 = int(y1 + dy), int(y2 - dy)
        m = np.zeros((h, w), dtype=bool)
        m[max(0, y1):max(1, y2), max(0, x1):max(1, x2)] = True
        return m


# ---------------------------------------------------------------- 自检
def _check(a):
    from common import load_config, is_dry_run
    cfg = load_config(a.config)
    b = cfg.get("brain") or {}
    print("=" * 62)
    print(" brain 自检")
    print("=" * 62)
    print(" enabled = %s" % b.get("enabled"))
    print(" url     = %s" % b.get("url"))
    print(" model   = %s" % b.get("model"))
    print(" target  = %s" % ((cfg.get("perception") or {}).get("target_name")
                             or b.get("target_name")))
    if not b.get("enabled"):
        print("\n ⚠️ brain.enabled=false：现在是颜色阈值模式（最稳，不受模型影响）")
        print("    要用大脑就把 real_params.yaml 的 brain.enabled 改 true，"
              "并把 perception.target_source 改 brain")

    # 取图：给了 --image 就读文件，否则抓一帧相机
    img = None
    if a.image:
        import cv2
        img = cv2.imread(os.path.expanduser(a.image))
        if img is None:
            print(" ❌ 读不到图片 %s" % a.image)
            return 1
        img = img[:, :, ::-1]      # BGR → RGB
    else:
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import Image as RosImage
        except ImportError:
            print(" ❌ 没有 rclpy：先 source /opt/ros/foxy/setup.bash，"
                  "或用 --image 指定一张图")
            return 2
        c = cfg["camera"]
        prefix = c["prefix"].rstrip("/")
        rclpy.init()
        node = Node("brain_check")
        holder = {"img": None}

        def on_img(m):
            buf = np.frombuffer(m.data, dtype=np.uint8)
            if len(buf) < m.height * m.width * 3:
                return
            im = buf[:m.height * m.width * 3].reshape(m.height, m.width, 3)
            holder["img"] = im.copy() if "bgr" in (m.encoding or "").lower() \
                else im[:, :, ::-1].copy()

        node.create_subscription(RosImage, "%s/%s" % (prefix, c["color_suffix"]),
                                 on_img, 10)
        t0 = time.time()
        while holder["img"] is None and time.time() - t0 < 8:
            rclpy.spin_once(node, timeout_sec=0.2)
        img = holder["img"]
        node.destroy_node()
        rclpy.shutdown()
        if img is None:
            print(" ❌ 8 秒内没收到彩色图（相机没起 / 前缀不对）")
            return 2

    h, w = img.shape[:2]
    print("\n 图像 %dx%d" % (w, h))
    client = BrainClient(cfg)
    client.enabled = True                        # 自检时忽略 enabled
    t0 = time.time()
    box, label, info = client.detect_box(img)
    dt = (time.time() - t0) * 1000
    print(" 结果：%s" % info)
    if box:
        print(" 标签：%s" % label)
        print(" 框  ：[%.0f, %.0f, %.0f, %.0f]  中心 (%.0f, %.0f)"
              % (box[0], box[1], box[2], box[3],
                 (box[0] + box[2]) / 2, (box[1] + box[3]) / 2))
        print(" 耗时：%.0f ms（含网络）" % dt)
        if dt > 1500:
            print(" ⚠️ 单次 >1.5s：抓取时感知会跟不上。降分辨率/换更小模型/改 interval")
    else:
        print(" ❌ 没拿到框。依次查：")
        print("   1) 服务端起了吗：curl -s %s" % cfg.get("brain", {}).get("url"))
        print("   2) 模型名对吗（Ollama: `ollama list`；LM Studio 用 /v1/models 看）")
        print("   3) 提示词里要求 JSON，有些模型会夹解释 —— 本脚本已做容错解析")
        return 1
    print("\n 下一步：把 perception.target_source 改 brain，重起 launch，")
    print("        看 /grasp/target_xyz 有没有数；颜色阈值模式随时可切回来。")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="端到端识别大脑客户端（HTTP）")
    ap.add_argument("--check", action="store_true", help="连通性/效果自检")
    ap.add_argument("--image", default=None, help="自检用的图片路径（不给就抓相机）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--target", default=None, help="要识别什么，如 'green cube'")
    a = ap.parse_args(argv)
    if a.check:
        return _check(a)
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""perception_mode.py —— 一键切换「颜色阈值 / 识别大脑」，并检查配置一致性

为什么要有它
------------
切感知模式其实要同时改两处，忘了任何一处都会得到"看起来配了但没生效"的状态：

    perception.target_source: color | brain     ← 用哪条路
    brain.enabled:            false | true      ← 服务开不开

典型翻车：
  · 只改 target_source=brain、忘了 brain.enabled=true
    → 感知启动时自动退回 color，日志只有一行 warn，抓取照样"能跑"但你以为在用大脑
  · 只改 brain.enabled=true、忘了 target_source=brain
    → 大脑连上了但根本没人调用，白占显存
  · 切完忘了重启 launch（配置只在启动时读一次）

用法
----
  python3 robot_code/arm_grasp/scripts/perception_mode.py            # 只看当前状态
  python3 robot_code/arm_grasp/scripts/perception_mode.py color      # 切颜色阈值
  python3 robot_code/arm_grasp/scripts/perception_mode.py brain      # 切大脑
  python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "red block"
  python3 robot_code/arm_grasp/scripts/perception_mode.py brain --check   # 切完顺手自检
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import socket
import sys
import time
from urllib.parse import urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WS = os.environ.get("ROBOT_WS", os.path.expanduser("~/robot_vla_ws"))
PARAMS = os.path.join(WS, "robot_code", "arm_grasp", "config", "real_params.yaml")


# ---------------------------------------------------------------- 读写
def load():
    import yaml
    with open(PARAMS, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def patch_key(path, key, value, section=None, dry=False):
    """改 `key: value` 这一行，保留行尾注释；section 给定时只在该顶层段内改。"""
    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    out, in_sec, hit = [], (section is None), False
    # 值可能是 "带空格的引号串"，也可能是裸 token，后面再跟行尾注释
    pat = re.compile(r'^(\s*)%s:\s*("(?:[^"]*)"|\'(?:[^\']*)\'|[^\s#]+)(\s*(?:#.*)?)$'
                     % re.escape(key))
    for ln in lines:
        if re.match(r'^[A-Za-z_][A-Za-z0-9_]*:\s*$', ln):
            in_sec = (section is None) or ln.startswith(section + ":")
        m = pat.match(ln) if in_sec else None
        if m and not hit:
            out.append("%s%s: %s%s" % (m.group(1), key, value, m.group(3)))
            hit = True
        else:
            out.append(ln)
    if hit and not dry:
        shutil.copy2(path, path + ".bak.%s" % time.strftime("%Y%m%d-%H%M%S"))
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(out) + "\n")
    return hit


def tcp_ok(url, timeout=1.5):
    try:
        u = urlparse(url)
        host = u.hostname or "127.0.0.1"
        port = u.port or (443 if u.scheme == "https" else 80)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            s.connect((host, port))
            return True, "%s:%d 可连接" % (host, port)
        finally:
            s.close()
    except Exception as e:                         # noqa: BLE001
        return False, "连不上：%s" % e


# ---------------------------------------------------------------- 展示
def show(cfg):
    p = cfg.get("perception") or {}
    b = cfg.get("brain") or {}
    src = str(p.get("target_source", "color")).lower()
    print("=" * 62)
    print(" 当前感知模式")
    print("=" * 62)
    print(" perception.target_source : %s" % src)
    print(" perception.target_color  : %s" % p.get("target_color"))
    print(" perception.target_name   : %s" % p.get("target_name"))
    print(" color_thresholds        : %s"
          % {k: p.get("color_thresholds", {}).get(k) for k in ("green", "red", "blue")})
    print(" brain.enabled           : %s" % b.get("enabled"))
    print(" brain.url               : %s" % b.get("url"))
    print(" brain.model             : %s" % b.get("model"))
    print(" brain.fallback_to_color : %s" % b.get("fallback_to_color"))

    print("\n--- 一致性检查 ---")
    bad = 0
    if src == "brain":
        if not b.get("enabled"):
            print(" ❌ target_source=brain 但 brain.enabled=false → "
                  "启动时会自动退回颜色阈值（等于白配）")
            bad += 1
        else:
            ok, msg = tcp_ok(str(b.get("url", "")))
            print(" %s 大脑服务端 %s" % ("✅" if ok else "⚠️ ", msg))
            if not ok:
                print("    大脑不可用不会让任务挂掉（fallback_to_color=%s），"
                      "但那一刻实际在走颜色阈值" % b.get("fallback_to_color"))
    else:
        if b.get("enabled"):
            print(" ⚠️  target_source=color，但 brain.enabled=true → 大脑会连着却不被调用"
                  "（白占显存）。建议一起关掉")
        else:
            print(" ✅ 颜色阈值模式，大脑关闭（最稳的一条路）")
    print(" ✅ 改了配置要重启 launch 才生效（配置只在启动时读一次）")
    return 0 if bad == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="切换颜色阈值 / 识别大脑")
    ap.add_argument("mode", nargs="?", choices=["color", "brain"], help="要切到哪个模式")
    ap.add_argument("--target", default=None,
                    help='brain 模式下识别什么，如 "green cube" / "red block" / "apple"')
    ap.add_argument("--check", action="store_true", help="切完后跑 brain.py --check")
    ap.add_argument("--config", default=None, help="real_params.yaml 路径（默认自动找）")
    ap.add_argument("--dry-run", action="store_true", help="只打印将要改什么，不写文件")
    a = ap.parse_args(argv)

    global PARAMS
    if a.config:
        PARAMS = os.path.expanduser(a.config)
    if not os.path.isfile(PARAMS):
        print("❌ 找不到 %s" % PARAMS)
        return 2

    if a.mode is None:
        return show(load())

    # ---------- 切模式 ----------
    if a.mode == "color":
        changes = [("target_source", "color", "perception"),
                   ("enabled", "false", "brain")]
    else:
        changes = [("target_source", "brain", "perception"),
                   ("enabled", "true", "brain")]
    print("=" * 62)
    print(" 切换到 %s 模式%s" % (a.mode.upper(), "（演练，不写文件）" if a.dry_run else ""))
    print("=" * 62)
    for key, val, sec in changes:
        hit = patch_key(PARAMS, key, val, section=sec, dry=a.dry_run)
        print(" %s %s.%s → %s" % ("✅" if hit else "❌ 没找到该行", sec, key, val))
    if a.target:
        hit = patch_key(PARAMS, "target_name", '"%s"' % a.target,
                        section="perception", dry=a.dry_run)
        print(" %s perception.target_name → \"%s\"" % ("✅" if hit else "❌", a.target))
    if not a.dry_run:
        print("\n 原文件已备份为 %s.bak.<时间戳>" % PARAMS)

    show(load())
    print("\n 下一步：")
    print("   1) 重启链路：ros2 launch arm_grasp real_grasp.launch.py start_task:=false")
    print("   2) 确认出数：timeout 5 ros2 topic echo /grasp/target_xyz")
    print("   3) 看感知那行日志里是「颜色命中」还是「大脑框命中」，"
          "以此确认真的走了你想用的那条")
    if a.check and a.mode == "brain":
        print("\n 跑 brain 自检…")
        from brain import main as brain_main
        return brain_main(["--check"] + (["--config", PARAMS] if a.config else []))
    return 0


if __name__ == "__main__":
    sys.exit(main())

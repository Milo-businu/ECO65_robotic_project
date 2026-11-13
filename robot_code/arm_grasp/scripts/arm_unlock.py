#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_unlock.py —— 开关「两道锁」+ 设速度，一条命令搞定

为什么要这个工具
----------------
真机上有两道锁，同时满足才会真的下发运动指令：

    profile.mode: real              ← 说明这是真机（不是演练）
    profile.allow_arm_motion: true  ← 二次确认：允许动臂

`git pull` 之后配置文件会回到模板值（`allow_arm_motion: false`），于是
jog.py / 标定扫位 / 抓取任务全都会拒绝执行，报：

    ❌ 本工具会动机械臂，需要两道锁：profile.mode: real + profile.allow_arm_motion: true

手改 YAML 容易改错缩进（YAML 对缩进敏感），所以做成一条命令。

用法
----
  python3 robot_code/arm_grasp/scripts/arm_unlock.py              # 只看当前状态
  python3 robot_code/arm_grasp/scripts/arm_unlock.py on --speed 5 # 开锁 + 速度 5（最慢）
  python3 robot_code/arm_grasp/scripts/arm_unlock.py on --speed 10
  python3 robot_code/arm_grasp/scripts/arm_unlock.py off          # 立刻上锁

安全约定
--------
  · 开锁时会要求你手动确认「工作区清空、急停在手边、速度已设好」
  · 速度被限制在 1~20（`arm.speed_hard_max`），代码里再夹一层
  · 上锁只改 allow_arm_motion，不动 mode（mode 保持 real 不影响任何只读脚本）
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WS = os.environ.get("ROBOT_WS", os.path.expanduser("~/robot_vla_ws"))
PARAMS = os.path.join(WS, "robot_code", "arm_grasp", "config", "real_params.yaml")


def patch(path, key, value, section=None, dry=False):
    """改 `key: value` 这一行（保留行尾注释）；section 给定时只在该顶层段内改。"""
    lines = open(path, encoding="utf-8").read().splitlines()
    out, in_sec, hit = [], (section is None), False
    pat = re.compile(r'^(\s*)%s:\s*([^\s#]+)(\s*(?:#.*)?)$' % re.escape(key))
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
        open(path, "w", encoding="utf-8").write("\n".join(out) + "\n")
    return hit


def show(cfg):
    prof, arm = cfg["profile"], cfg["arm"]
    mode = str(prof.get("mode"))
    allow = bool(prof.get("allow_arm_motion"))
    spd = int(arm.get("speed", 20))
    cap = int(arm.get("speed_hard_max", 20))
    print("=" * 58)
    print(" 当前机械臂状态")
    print("=" * 58)
    print(" profile.mode            : %s%s"
          % (mode, "" if mode == "real" else "   ← 不是 real，臂不会动"))
    print(" profile.allow_arm_motion: %s%s"
          % (allow, "" if allow else "   ← 第二道锁关着，所有动臂命令都会被拒"))
    print(" arm.speed               : %d%%（硬上限 %d%%）" % (spd, cap))
    if mode == "real" and allow:
        print("\n ✅ 两道锁都开了 —— 现在 jog / 标定扫位 / 抓取任务都会真的动")
    else:
        print("\n ⛔ 锁着 —— 只读脚本（preflight / workspace_map / tune_perception）照常能用")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="开关两道锁 + 设速度")
    ap.add_argument("action", nargs="?", choices=["on", "off"], help="on=开锁  off=上锁")
    ap.add_argument("--speed", type=int, default=None, help="速度百分比 1~20（默认不动）")
    ap.add_argument("--config", default=None)
    ap.add_argument("--yes", action="store_true", help="跳过交互确认（脚本化用，慎用）")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)

    global PARAMS
    if a.config:
        PARAMS = os.path.expanduser(a.config)
    if not os.path.isfile(PARAMS):
        print("❌ 找不到 %s" % PARAMS)
        return 2

    import yaml
    cfg = yaml.safe_load(open(PARAMS, encoding="utf-8"))

    if a.action is None:
        return show(cfg)

    # ---------- 上锁 ----------
    if a.action == "off":
        patch(PARAMS, "allow_arm_motion", "false", section="profile", dry=a.dry_run)
        print("✅ 已上锁：allow_arm_motion = false（臂不会再动）")
        if not a.dry_run:
            print("   正在运行的任务不受影响，下一条命令就会被拒。")
        return show(yaml.safe_load(open(PARAMS, encoding="utf-8")))

    # ---------- 开锁 ----------
    spd = a.speed if a.speed is not None else int(cfg["arm"].get("speed", 10))
    spd = max(1, min(int(spd), 20))
    print("=" * 58)
    print(" ⚠️  即将允许机械臂运动")
    print("=" * 58)
    print(" 速度会设成 %d%%（硬上限 20%%）" % spd)
    print(" 动臂前请确认：")
    print("   1. 工作区已清空，人在臂展之外")
    print("   2. 急停按钮在手边（按下会立刻断使能）")
    print("   3. 手上没有线缆会被夹爪/臂带到的位置")
    if not a.yes and not a.dry_run:
        try:
            ans = input("\n 确认无误请输入 yes 回车：").strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = ""
        if ans != "yes":
            print(" 已取消，什么都没改。")
            return 1

    patch(PARAMS, "mode", "real", section="profile", dry=a.dry_run)
    patch(PARAMS, "allow_arm_motion", "true", section="profile", dry=a.dry_run)
    patch(PARAMS, "speed", str(spd), section="arm", dry=a.dry_run)
    if a.dry_run:
        print("（--dry-run：只打印，没写文件）")
        return 0

    print("\n✅ 已开锁：mode=real、allow_arm_motion=true、speed=%d%%" % spd)
    print("   备份：%s.bak.<时间戳>" % os.path.basename(PARAMS))
    print("\n 接下来：")
    print("   只读脚本不用重启：preflight.py / workspace_map.py / tune_perception.py")
    print("   动臂的节点要重启才读到新配置：")
    print("     ros2 launch arm_grasp real_grasp.launch.py start_task:=false")
    print("   第一次手动挪臂（最慢）：")
    print("     python3 robot_code/arm_grasp/scripts/jog.py --speed 5")
    print("\n 用完想立刻锁上：arm_unlock.py off")
    return show(yaml.safe_load(open(PARAMS, encoding="utf-8")))


if __name__ == "__main__":
    sys.exit(main())

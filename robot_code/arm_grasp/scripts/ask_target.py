#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ask_target.py —— 一句话切换抓取目标颜色（运行期生效，不用重启感知）

为什么要有它
------------
之前的流程：想换目标颜色 → 改 real_params.yaml → 重启整个 launch。
一次重启 40 秒，还要重新等相机、等手眼、等 FK 稳定。现场试三种颜色就是三分钟纯等待。

这个脚本走的是另一条路：感知节点在运行时订阅 /perception/target_color，
收到消息就改自己内存里的 cfg["perception"]["target_color"]，下一个周期立刻生效。
所以只要一行：

    ask "抓蓝色的方块"

就把目标从绿色换成蓝色，相机不用重启、机械臂不用回 home。

支持的说法（中英文都行，只要句子里有这个词）
-----------------------------------------------
    绿 / green          → green
    红 / red            → red
    蓝 / blue           → blue
    随便 / 任意 / 都行 / auto → auto（三种掩码里像素最多的那个）

用法
----
    ask "抓蓝色的方块"              # 切换 + 确认看到目标
    ask 红色                        # 一个词也行
    ask "blue object" --save        # 顺手写进 real_params.yaml，下次启动默认蓝色
    ask "随便" --grasp              # 切换 → 确认看到 → 直接接着跑抓取

⚠️ 注意
   · 只切"哪几个像素算目标"，不改阈值。红/蓝两组的阈值还是出厂值，
     现场光照下可能一个像素都不命中 —— 那就得先 tune 调阈值。
   · 不写 --save 的话，感知重启后回到 yaml 里配的颜色。这是刻意的：
     试颜色是临时动作，别一激动把配置文件改乱了。
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import load_config  # noqa: E402

WS = os.environ.get("ROBOT_WS", os.path.expanduser("~/robot_vla_ws"))
PARAMS = os.path.join(WS, "robot_code", "arm_grasp", "config", "real_params.yaml")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 顺序有意义：auto 的词最特殊先判，剩下的都是单字/单词，互不冲突
RULES = [
    ("auto", ["随便", "任意", "哪个都", "都行", "最大的", "auto"]),
    ("blue", ["蓝", "blue"]),
    ("green", ["绿", "green"]),
    ("red", ["红", "red"]),
]
CN = {"green": "绿色", "red": "红色", "blue": "蓝色", "auto": "自动（像素最多）"}
ANSI = dict(g="\033[32m", y="\033[33m", r="\033[31m", b="\033[1m", n="\033[0m")


def say(tag, msg):
    print("%s[%s]%s %s" % (ANSI.get(tag, ""), {"g": "OK", "y": "!!", "r": "XX", "b": ">>"}.get(tag, ".."),
                           ANSI["n"], msg))


def parse_demand(text):
    """一句话 → green/red/blue/auto。找不到就返回 (None, None)。"""
    s = str(text).strip().lower()
    for name, kws in RULES:
        for kw in kws:
            if kw in s:
                return name, kw
    return None, None


# ---------------------------------------------------------------- 写配置
def save_color(color, dry=False):
    """把 perception.target_color 写进 real_params.yaml（保留行尾注释，写前校验）。"""
    import yaml

    with open(PARAMS, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()

    out, in_sec, hit = [], False, False
    pat = re.compile(r'^(\s*)target_color:(\s*)(\S+)(.*)$')
    for ln in lines:
        if re.match(r'^[A-Za-z_]', ln):          # 顶层段开始
            in_sec = ln.startswith("perception:")
        if in_sec and not hit:
            m = pat.match(ln)
            if m:
                # 段间空白与行尾注释原样保留（不做 strip，否则注释会被推到顶格）
                out.append("%s%s:%s%s%s" % (m.group(1), "target_color", m.group(2), color, m.group(4)))
                hit = True
                continue
        out.append(ln)

    if not hit:
        raise RuntimeError("在 %s 的 perception 段里没找到 target_color 这一行" % PARAMS)

    new = "\n".join(out) + "\n"
    cfg = yaml.safe_load(new)                         # ← 写盘前先验证，不通过就抛异常
    got = cfg["perception"]["target_color"]
    if got != color:
        raise RuntimeError("写回后读出来是 %r，不是 %r —— 放弃写入" % (got, color))

    if dry:
        say("b", "dry-run：不写盘")
        return
    bak = "%s.bak.%s" % (PARAMS, time.strftime("%Y%m%d-%H%M%S"))
    if os.path.exists(bak):                      # 同一秒内连改两次，别把备份覆盖成中间态
        bak = "%s.%d" % (bak, os.getpid())
    with open(PARAMS, "r", encoding="utf-8") as f:
        old = f.read()
    with open(bak, "w", encoding="utf-8") as f:
        f.write(old)
    with open(PARAMS, "w", encoding="utf-8") as f:
        f.write(new)
    say("g", "已写进配置：%s（备份 %s）" % (PARAMS, os.path.basename(bak)))


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(
        description="一句话切换抓取目标颜色（运行期生效）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='例：ask "抓蓝色的方块"      ask 红色 --save      ask 随便 --grasp')
    ap.add_argument("demand", nargs="*", help="一句话，如 抓蓝色 / blue object")
    ap.add_argument("--save", action="store_true", help="同时写进 real_params.yaml（下次启动默认）")
    ap.add_argument("--grasp", action="store_true", help="确认看到目标后直接接着跑抓取")
    ap.add_argument("--wait", type=float, default=3.0, help="等目标出现的最长秒数（默认 3）")
    ap.add_argument("--dry-run", action="store_true", help="只解析，不连 ROS")
    a = ap.parse_args()

    if not a.demand:
        say("y", "要说一句话才知道抓什么，例如：ask \"抓蓝色的方块\"")
        say("b", "支持：绿/green  红/red  蓝/blue  随便/auto")
        return 2

    text = " ".join(a.demand)
    color, kw = parse_demand(text)
    if color is None:
        say("r", "没听懂：%r 里没有颜色词" % text)
        say("b", "支持：绿/green  红/red  蓝/blue  随便/任意/auto")
        return 2

    say("b", "需求：%s   →   命中关键词「%s」   →   target_color = %s"
        % (text, kw, CN[color]))

    cfg = load_config()
    old = cfg["perception"].get("target_color", "green")
    if old == color:
        say("y", "本来就是 %s，不重复切（继续往下确认目标）" % CN[color])

    if a.dry_run:
        say("b", "dry-run：只解析，不连 ROS")
        return 0

    try:
        import rclpy
        from rclpy.node import Node  # noqa: F401
        from std_msgs.msg import String, Float64MultiArray
    except Exception as e:  # ROS 环境没 source
        say("r", "导入 rclpy 失败：%s" % e)
        say("b", "先载入环境：renv   （或直接用 alias：ask \"抓蓝色\" 会自动载）")
        return 1

    tgt_topic = cfg["perception"]["target_topic"]
    rclpy.init()
    node = rclpy.create_node("ask_target")
    try:
        pub = node.create_publisher(String, "/perception/target_color", 10)
        box = {"d": None}

        def on_tgt(m):
            box["d"] = list(m.data)

        node.create_subscription(Float64MultiArray, tgt_topic, on_tgt, 10)

        # 连发几次：DDS 发现要时间，第一帧常常掉在发现完成之前
        for _ in range(6):
            pub.publish(String(data=color))
            rclpy.spin_once(node, timeout_sec=0.1)

        try:
            n_sub = pub.get_subscription_count()
        except Exception:
            n_sub = -1
        if n_sub == 0:
            say("y", "没人订阅 %s —— 感知节点没在跑？先开一个终端 link0" % "/perception/target_color")

        # 等一帧新目标：能收到就说明"切换后确实看得见东西"
        t0 = time.time()
        while time.time() - t0 < a.wait and box["d"] is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if a.save:
            save_color(color, dry=False)

        d = box["d"]
        if d is None:
            say("y", "已切到 %s，但 %.1fs 内没看到目标" % (CN[color], a.wait))
            say("b", "常见原因：① 方块不在视野 / 被机械臂挡住")
            say("b", "          ② 这一类颜色的阈值还没在现场调过（红/蓝仍是出厂值 min=100）")
            say("b", "→ 调阈值：tune      看模式：pmode      看话题：rtl")
            return 1

        if len(d) >= 3:
            s = ("目标位姿：x=%+.3f  y=%+.3f  z=%+.3f m" % (d[0], d[1], d[2]))
            if len(d) >= 5:
                s += ("   宽 %.1fmm  高 %.1fmm" % (d[3] * 1000.0, d[4] * 1000.0))
            say("g", s)
        else:
            say("g", "目标数据：%s" % (d,))

        if not a.save:
            say("b", "运行期生效（感知重启会回到 yaml 里的 %s）；想长期保留加 --save" % old)

        if a.grasp:
            say("b", "接着跑抓取（grasp_task.py --real）……")
            import subprocess
            return subprocess.call([sys.executable, os.path.join(SCRIPT_DIR, "grasp_task.py"), "--real"],
                                   cwd=WS)
        say("g", "可以抓了：grasp")
        return 0
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

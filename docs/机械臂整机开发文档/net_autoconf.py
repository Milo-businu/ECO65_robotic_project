#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""net_autoconf.py —— 一条命令把「机械臂 IP / 本机 UDP 上报 IP」配对好

为什么专门写这个
----------------
`/joint_states` 一条数据都没有，是本项目真机上最耗时间的坑：
  rm_driver 用 UDP **主动**把关节角发到它配置里的 `udp_ip`。
  这个 IP 填的不是【真机 PC 自己的 IP】→ 包全发到空气里 → 感知算不了 FK
  → 抓取整条链全废，而且报错信息完全不指向这里（只会说"收不到关节角"）。

手动修要开两个文件、抄两次 IP，还容易只改了 real_params.yaml 忘了改驱动那份。
这个脚本一次改全，并且改完自己 ping 一遍机械臂。

找不到机械臂怎么办
------------------
  ping 不通时本脚本会**在机械臂网段扫 8080 端口**（rm_driver 的控制端口），
  命中的基本就是机械臂，自动改用它。比"猜 192.168.1.18"靠谱：
  换过 IP、被 DHCP 分过、出厂不是 .18 都能找到。

它改哪些文件
------------
  ① robot_code/ros2_rm_robot/rm_driver/config/rm_eco65_config.yaml   ← 驱动真正读的
     （`--symlink-install` 时 install/ 里那份是软链，改 src 即可；
       若 install 里是实体副本，会一起改掉）
  ② robot_code/arm_grasp/config/real_params.yaml 的 arm.udp_ip / arm.arm_ip
     （只是记录与自检用；但两边不一致会让人以后看糊涂）

用法
----
  python3 robot_code/arm_grasp/scripts/net_autoconf.py                 # 自动配对并写入
  python3 robot_code/arm_grasp/scripts/net_autoconf.py --check-only    # 只看不改
  python3 robot_code/arm_grasp/scripts/net_autoconf.py --arm-ip 192.168.1.18
  python3 robot_code/arm_grasp/scripts/net_autoconf.py --udp-ip 192.168.1.10
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import shutil
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WS = os.environ.get("ROBOT_WS", os.path.expanduser("~/robot_vla_ws"))
DRV_CFG = os.path.join(WS, "robot_code", "ros2_rm_robot", "rm_driver",
                       "config", "rm_eco65_config.yaml")
PARAMS = os.path.join(WS, "robot_code", "arm_grasp", "config", "real_params.yaml")

PASS, WARN, FAIL = [], [], []


def ok(m):
    PASS.append(m); print("  ✅ %s" % m)


def warn(m):
    WARN.append(m); print("  ⚠️  %s" % m)


def bad(m):
    FAIL.append(m); print("  ❌ %s" % m)


def local_ips():
    """尽量把本机所有 IPv4 都列出来（hostname -I 最全，socket 兜底）。"""
    ips = []
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True,
                             timeout=3).stdout
        ips += [x for x in out.split() if "." in x]
    except Exception:                              # noqa: BLE001
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.append(info[4][0])
    except Exception:                              # noqa: BLE001
        pass
    seen, out = set(), []
    for ip in ips:
        if ip not in seen and not ip.startswith("127."):
            seen.add(ip)
            out.append(ip)
    return out


def same_subnet(ip, ref, mask=24):
    """按 /24 判断是否同网段（机械臂现场几乎都是 192.168.1.x）。"""
    try:
        a, b = ip.split("."), ref.split(".")
        return a[:3] == b[:3] if mask == 24 else a[:2] == b[:2]
    except Exception:                              # noqa: BLE001
        return False


def ping(ip, timeout=1):
    cmd = ["ping", "-c", "2", "-W", str(timeout), ip]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout * 3 + 2)
        return r.returncode == 0, (r.stdout or "") + (r.stderr or "")
    except Exception as e:                         # noqa: BLE001
        return False, str(e)


def tcp_open(ip, port, timeout=2.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((ip, port))
        return True
    except Exception:                              # noqa: BLE001
        return False
    finally:
        s.close()


def patch_ip(path, key, value, dry=False):
    """把 yaml 里 `key: "..."` 换成新值。返回 True 表示有改动。"""
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    pat = re.compile(r'^(\s*)%s:\s*"?[^"#\s]*"?(\s*(?:#.*)?)$' % re.escape(key))
    hit = False
    out = []
    for ln in lines:
        m = pat.match(ln)
        if m:
            out.append('%s%s: "%s"%s' % (m.group(1), key, value, m.group(2)))
            hit = True
        else:
            out.append(ln)
    if hit and not dry:
        shutil.copy2(path, path + ".bak.%s" % time.strftime("%Y%m%d-%H%M%S"))
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(out) + "\n")
    return hit


# ---------------------------------------------------------------- 网卡 / 扫描
def iface_of_ip(ip):
    """哪个网卡上有这个 IP（用来在提示里给出真实网卡名，而不是写死 eth0）。"""
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show"], capture_output=True,
                             text=True, timeout=3).stdout
        for ln in out.splitlines():
            # 2: enp3s0    inet 192.168.1.26/24 brd ...
            parts = ln.split()
            if len(parts) >= 4 and parts[3].split("/")[0] == ip:
                return parts[1].rstrip(":")
    except Exception:                              # noqa: BLE001
        pass
    return ""


def nmcli_conn_of(dev):
    """网卡对应的 NetworkManager 连接名（nmcli 用的是连接名，不是网卡名）。"""
    if not dev:
        return ""
    try:
        out = subprocess.run(["nmcli", "-t", "-f", "NAME,DEVICE", "con", "show"],
                             capture_output=True, text=True, timeout=3).stdout
        for ln in out.splitlines():
            if ":" in ln:
                name, d = ln.rsplit(":", 1)
                if d == dev:
                    return name
    except Exception:                              # noqa: BLE001
        pass
    return ""


def route_source(ip):
    """问内核：去这个 IP 会走哪个网卡、用哪个源地址。

    为什么必须问内核：一台机器可能同时有两个 192.168.1.x 地址（有线 + USB 网卡），
    此时"挑一个同网段的 IP"是猜的，而 rm_driver 的 udp_ip 必须是**内核实际使用的
    源地址**，否则机械臂把 UDP 回包发到另一个网卡 → /joint_states 永远空。
    """
    try:
        out = subprocess.run(["ip", "route", "get", ip], capture_output=True,
                             text=True, timeout=3).stdout.strip()
        dev = re.search(r"\bdev (\S+)", out)
        src = re.search(r"\bsrc (\S+)", out)
        return (dev.group(1) if dev else ""), (src.group(1) if src else ""), out
    except Exception:                              # noqa: BLE001
        return "", "", ""


def same_subnet_groups(pairs):
    """按 /24 分组，找出"同一网段出现在多个网卡上"的情况。"""
    g = {}
    for dev, ip in pairs:
        key = ".".join(ip.split(".")[:3])
        g.setdefault(key, []).append((dev, ip))
    return {k: v for k, v in g.items() if len({d for d, _ in v}) > 1}


def local_iface_pairs():
    """[(网卡, ipv4), ...]，只列有 IPv4 的（用于在提示里给真实网卡名）。"""
    out = []
    try:
        txt = subprocess.run(["ip", "-o", "-4", "addr", "show"], capture_output=True,
                             text=True, timeout=3).stdout
        for ln in txt.splitlines():
            parts = ln.split()
            if len(parts) >= 4 and parts[2] == "inet":
                out.append((parts[1].rstrip(":"), parts[3].split("/")[0]))
    except Exception:                              # noqa: BLE001
        pass
    return out


def pick_physical(pairs):
    """从 [(网卡, ip)] 里挑一个像"有线网卡"的：排除 lo / docker / veth / br- / virbr。"""
    skip = ("lo", "docker", "veth", "br-", "virbr", "tun", "tap")
    for dev, _ip in pairs:
        if not any(dev.startswith(k) for k in skip):
            return dev
    return ""


def scan_for_arm(prefix, port=8080, timeout=0.35, workers=64):
    """在 prefix.x 网段里扫 8080 端口 —— 那是 rm_driver 的控制端口，命中基本就是机械臂。

    比"猜 IP"靠谱：机械臂换过 IP、被 DHCP 分过、或出厂不是 .18 都能找到。
    """
    from concurrent.futures import ThreadPoolExecutor
    hosts = ["%s.%d" % (prefix, i) for i in range(1, 255)]

    def probe(h):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            return h if s.connect_ex((h, port)) == 0 else None
        except Exception:                          # noqa: BLE001
            return None
        finally:
            s.close()

    with ThreadPoolExecutor(max_workers=workers) as ex:
        return sorted(h for h in ex.map(probe, hosts) if h)


def arp_mac(ip):
    """尽力拿 MAC，便于判断是不是睿尔曼的网口（拿不到就返回空）。"""
    try:
        out = subprocess.run(["ip", "neigh", "show", ip], capture_output=True,
                             text=True, timeout=2).stdout
        m = re.search(r"lladdr ([0-9a-f:]{17})", out)
        return m.group(1) if m else ""
    except Exception:                              # noqa: BLE001
        return ""


def main(argv=None):
    ap = argparse.ArgumentParser(description="机械臂 IP / UDP 上报 IP 自动配对")
    ap.add_argument("--arm-ip", default=None, help="机械臂 IP（默认从 real_params.yaml 读）")
    ap.add_argument("--udp-ip", default=None, help="本机在机械臂网段的 IP（默认自动挑）")
    ap.add_argument("--check-only", action="store_true", help="只检查不写文件")
    ap.add_argument("--no-scan", action="store_true",
                    help="ping 不通时不要扫网段（默认会扫 8080 端口找机械臂）")
    a = ap.parse_args(argv)

    print("=" * 62)
    print(" 机械臂网络自动配对（rm_driver 的 arm_ip / udp_ip）")
    print("=" * 62)

    # ---------- 读当前配置 ----------
    try:
        from common import load_config
        cfg = load_config()
        arm_ip = a.arm_ip or str(cfg["arm"].get("arm_ip", "")).strip()
        udp_cfg = str(cfg["arm"].get("udp_ip", "")).strip()
    except Exception as e:                         # noqa: BLE001
        bad("读不到 real_params.yaml：%s" % e)
        return 1
    print("\n  配置里的机械臂 IP: %s" % (arm_ip or "（空）"))
    print("  配置里的本机 UDP IP: %s" % (udp_cfg or "（空）"))

    if not arm_ip:
        bad("arm.arm_ip 是空的，先用 --arm-ip 192.168.1.18 指定")
        return 1

    # ---------- 选本机在机械臂网段的 IP ----------
    print("\n[1/5] 挑本机在机械臂网段的 IP")
    ips = local_ips()
    print("      本机 IPv4: %s" % (ips or "（一个都没读到）"))
    cand = [ip for ip in ips if same_subnet(ip, arm_ip)]
    udp_trusted = False
    if a.udp_ip:
        udp_ip = a.udp_ip
        udp_trusted = True
        ok("按 --udp-ip 指定：%s" % udp_ip)
    elif cand:
        # ★ 不再"挑一个同网段的"，而是问内核：去机械臂实际走哪个网卡、用哪个源地址
        rdev, rsrc, rline = route_source(arm_ip)
        if rsrc and same_subnet(rsrc, arm_ip):
            udp_ip = rsrc
            udp_trusted = True
            ok("内核去 %s 会走 %s、源地址 %s → 用它当 udp_ip"
               % (arm_ip, rdev or "?", rsrc))
        else:
            udp_ip = sorted(cand)[0]
            udp_trusted = True
            ok("自动选中 %s（与机械臂同网段）" % udp_ip)
        dev = iface_of_ip(udp_ip)
        if dev:
            print("      这块网卡是 %s —— 机械臂的网线要插在它上面" % dev)
        groups = same_subnet_groups(local_iface_pairs())
        if groups:
            for net, items in groups.items():
                warn("%s.0/24 同时出现在多块网卡上：%s"
                     % (net, ", ".join("%s=%s" % (d, i) for d, i in items)))
            print("      ⚠️ 两个网卡同网段会让 UDP 回包走错网卡（表现为 /joint_states 永远空）。")
            print("         建议只保留一块（拔掉另一根线，或把另一块改到别的网段）。")
            if rsrc:
                print("         已按内核路由选 %s；确认机械臂网线插在 %s 上" % (udp_ip, rdev or "?"))
    elif udp_cfg and same_subnet(udp_cfg, arm_ip):
        udp_ip = udp_cfg
        warn("本机网卡上没看到同网段 IP，但配置里的 %s 看着对；先沿用" % udp_cfg)
    else:
        udp_ip = ""
        bad("本机没有 %s 同网段的 IP" % arm_ip)

    prefix = ".".join(udp_ip.split(".")[:3]) if udp_ip else ""

    # ---------- 机械臂连通性：ping 不通就扫 8080 ----------
    print("\n[2/5] 机械臂连通性")
    if not arm_ip:
        bad("arm.arm_ip 是空的：用 --arm-ip 指定，或让本脚本扫（需要本机有同网段 IP）")
    elif ping(arm_ip)[0]:
        ok("ping %s 通" % arm_ip)
    else:
        bad("ping %s 不通" % arm_ip)
        if a.no_scan or not prefix or not udp_trusted:
            print("      本机不在机械臂那个网段（或已 --no-scan），没法扫。先解决 IP，再看三条：")
            print("      ① 网线是否插在机械臂的【网口】（不是示教器口），网口灯闪不闪")
            print("      ② 机械臂控制器上电了吗、急停松开没有")
            print("      ③ 机械臂自己的屏幕上一般会显示它的 IP，看一眼是不是 %s" % arm_ip)
        else:
            print("      不去猜了：在 %s.0/24 里扫 8080 端口（rm_driver 的控制端口）…" % prefix)
            found = [h for h in scan_for_arm(prefix) if h != udp_ip]
            if len(found) == 1:
                mac = arp_mac(found[0])
                ok("找到 %s%s（8080 开放）→ 判定为机械臂，改用它"
                   % (found[0], (" / " + mac) if mac else ""))
                arm_ip = found[0]
            elif found:
                warn("扫到 %d 个开着 8080 的设备：%s" % (len(found), ", ".join(found)))
                print("      用 --arm-ip 指定其中哪个是机械臂（其余可能是路由器/其它服务）")
            else:
                bad("整个 %s.0/24 里没有开 8080 的设备" % prefix)
                print("      这意味着：机械臂没上电 / 网线没插在机械臂网口 / ")
                print("      机械臂和本机不在同一台交换机上 / 它被配到了别的网段")
                print("      先看机械臂自己屏幕上的 IP，再回来 --arm-ip 指定")

    # ---------- 8080 端口 ----------
    print("\n[3/5] 机械臂控制端口 8080")
    if arm_ip and tcp_open(arm_ip, 8080):
        ok("%s:8080 可连接（rm_driver 的 TCP 通道）" % arm_ip)
    elif arm_ip:
        warn("%s:8080 连不上 —— 机械臂没使能/没上电，或被别的程序占着" % arm_ip)

    # ---------- 写入 ----------
    print("\n[4/5] 写入配置")
    if not udp_ip:
        bad("没有可写的 udp_ip，跳过写入")
        if prefix == "":
            print("      先把有线网卡配成机械臂同网段（下面用真实网卡/连接名）：")
            pairs = local_iface_pairs()
            dev = pick_physical(pairs)
            if dev:
                conn = nmcli_conn_of(dev) or dev
                print("        你的有线网卡是 %s，连接名是 '%s'" % (dev, conn))
                print("        sudo nmcli con mod '%s' ipv4.addresses 192.168.1.100/24" % conn)
                print("        sudo nmcli con mod '%s' ipv4.method manual" % conn)
                print("        sudo nmcli con up '%s'" % conn)
            else:
                print("        没识别出有线网卡，先跑 ip -br addr 看看哪个网卡连着机械臂")
                print("        然后：sudo nmcli con mod '<连接名>' ipv4.addresses 192.168.1.100/24")
        return _summary()

    targets = [DRV_CFG, PARAMS]
    # install 里若存在实体副本（不是软链），也要一起改，否则驱动读的还是旧的
    for inst in glob.glob(os.path.join(WS, "install", "rm_driver", "share",
                                       "rm_driver", "config", "rm_eco65_config.yaml")):
        if os.path.realpath(inst) != os.path.realpath(DRV_CFG):
            targets.append(inst)
    for inst in glob.glob(os.path.join(WS, "install", "arm_grasp", "share",
                                       "arm_grasp", "config", "real_params.yaml")):
        if os.path.realpath(inst) != os.path.realpath(PARAMS):
            targets.append(inst)

    for path in targets:
        if not os.path.isfile(path):
            warn("跳过（不存在）：%s" % path)
            continue
        keys = ("arm_ip", "udp_ip")
        hits = []
        for k in keys:
            v = arm_ip if k == "arm_ip" else udp_ip
            r = patch_ip(path, k, v, dry=a.check_only)
            hits.append("%s=%s%s" % (k, v, "" if r else "（没这行）"))
        print("      %s → %s" % (os.path.relpath(path, WS), ", ".join(hits)))

    if a.check_only:
        warn("--check-only：只检查，没有改任何文件")

    # ---------- 结论 ----------
    print()
    if udp_trusted:
        ok("应写入的 udp_ip = %s（来自本机网卡，可信）" % udp_ip)
    else:
        warn("udp_ip=%s 只是沿用配置值，不是从本机网卡读到的；"
             "配好静态 IP 后重跑一次确认" % udp_ip)
    return _summary()


def _summary():
    print("\n" + "=" * 62)
    print(" 汇总：✅ %d / ⚠️ %d / ❌ %d" % (len(PASS), len(WARN), len(FAIL)))
    print("=" * 62)
    if FAIL:
        print("\n先修这些：")
        for m in FAIL:
            print("  ❌ %s" % m)
        return 1
    print("\n下一步（真机实测顺序）：")
    print("  1) 重启 rm_driver 让它重新读配置：")
    print("     ros2 launch arm_grasp real_grasp.launch.py start_task:=false")
    print("  2) 另开终端确认关节角在刷（这是唯一判据）：")
    print("     timeout 3 ros2 topic echo /joint_states")
    print("  3) 有数据 = 通了；还是空 → 回来看 [4/4] 写的是不是你期望的网卡 IP")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/bin/bash
# check_real_robot.sh —— 真机环境盘点（只读，不改任何东西）
#
# 用法（在真机 beavertech 上）：
#   bash check_real_robot.sh 2>&1 | tee ~/real_robot_check.txt
# 然后把 ~/real_robot_check.txt 的内容贴给我，我来分析并给下一步指令。
#
# 设计原则：每一项都自带 ✅/❌ 判定，不需要你理解含义也能照抄反馈。

echo "=============================================="
echo " 真机环境盘点  $(date '+%Y-%m-%d %H:%M:%S')"
echo "=============================================="

ok()   { echo "  ✅ $1"; }
bad()  { echo "  ❌ $1"; }
info() { echo "     $1"; }

# ---------- 1. 操作系统 ----------
echo
echo "[1] 操作系统"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "     $PRETTY_NAME"
    case "$VERSION_ID" in
        20.04) ok "Ubuntu 20.04（本项目要求的版本）" ;;
        22.04) bad "Ubuntu 22.04 —— 与 ROS2 Foxy 不匹配，需要换系统或换 ROS 版本" ;;
        *)     bad "Ubuntu $VERSION_ID —— 非预期版本" ;;
    esac
else
    bad "读不到 /etc/os-release"
fi
info "内核: $(uname -r)"

# ---------- 2. ROS2 ----------
echo
echo "[2] ROS2"
if [ -d /opt/ros/foxy ]; then
    ok "/opt/ros/foxy 存在"
else
    bad "/opt/ros/foxy 不存在"
fi
if [ -n "$ROS_DISTRO" ]; then
    ok "当前 ROS_DISTRO=$ROS_DISTRO"
else
    bad "ROS 未 source（需先 source /opt/ros/foxy/setup.bash）"
fi
if command -v ros2 >/dev/null 2>&1; then
    ok "ros2 命令可用"
else
    bad "ros2 命令不可用"
fi

# ---------- 3. 机械臂（ECO65，以太网 + JSON） ----------
echo
echo "[3] 机械臂 ECO65"
info "请确认机械臂已上电并接入网线"
echo "     本机 IP：$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^$' | head -3 | tr '\n' ' ')"
echo "     ---- 机械臂默认网段通常是 192.168.1.x / 192.168.10.x ----"
if command -v ip >/dev/null 2>&1; then
    info "网络接口："
    ip -o link show 2>/dev/null | awk -F': ' '{print "       "$2}' | grep -v lo | head -5
fi
info "请手动确认：能否 ping 通机械臂 IP？（ping <机械臂IP>）"
if ls /opt/ros/foxy/share 2>/dev/null | grep -qi 'rm_'; then
    ok "已安装 rm_* 相关 ROS 包"
else
    bad "未发现 rm_* ROS 包（rm_driver / rm_control 等）"
fi

# ---------- 4. 夹爪（轮趣 MS42DC，USB-CDC 串口） ----------
echo
echo "[4] 夹爪 MS42DC（关键：vid 1a86 / pid 55d4）"
if command -v lsusb >/dev/null 2>&1; then
    if lsusb | grep -qi '1a86:55d4'; then
        ok "USB 上找到 WCH CH9102/CH343（1a86:55d4）"
        lsusb | grep -i '1a86:55d4' | sed 's/^/       /'
    else
        bad "USB 上没有 1a86:55d4（夹爪未接 / 未上电 / 线坏了）"
        info "当前所有 USB 设备："
        lsusb | sed 's/^/       /' | head -15
    fi
else
    bad "lsusb 不可用"
fi
if [ -e /dev/motor_serial ]; then
    ok "/dev/motor_serial udev 软链存在"
    info "实际指向: $(readlink -f /dev/motor_serial)"
else
    bad "/dev/motor_serial 不存在（需配 udev 规则，或用 /dev/ttyUSB0 等实际设备）"
    info "当前 tty 设备："
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/       /' || info "       无"
fi

# ---------- 5. 相机（RealSense D435i） ----------
echo
echo "[5] 相机 RealSense D435i"
if lsusb 2>/dev/null | grep -qi '8086'; then
    ok "USB 上找到 Intel 设备（8086）"
    lsusb | grep -i '8086' | sed 's/^/       /'
else
    bad "USB 上没有 Intel 设备（相机未接 / 用了 USB2 线 / 供电不足）"
fi
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    ok "librealsense 已安装"
    info "固件版本（重要！决定 SDK 版本）："
    rs-enumerate-devices 2>/dev/null | grep -iE 'firmware|name|serial' | sed 's/^/       /' | head -8
else
    bad "librealsense 未安装（rs-enumerate-devices 不可用）"
    info "装了之后第一时间跑：rs-enumerate-devices | grep -i firmware"
fi
read -r -t 1 _ 2>/dev/null || true

# ---------- 6. 算力（GPU） ----------
echo
echo "[6] 算力"
if command -v nvidia-smi >/dev/null 2>&1; then
    ok "NVIDIA 驱动可用"
    nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader 2>/dev/null | sed 's/^/       /'
else
    bad "nvidia-smi 不可用（驱动未装？）"
fi

# ---------- 7. 磁盘 ----------
echo
echo "[7] 磁盘"
df -h / /home 2>/dev/null | sed 's/^/     /'
AVAIL=$(df -h /home 2>/dev/null | awk 'NR==2{print $4}' || echo "?")
info "/home 可用: $AVAIL（需 ≥40GB 才够装 ROS+VLA 权重）"

# ---------- 8. 保护名单（绝不触碰的工作空间） ----------
echo
echo "[8] 已有工作空间（保护名单，绝不覆盖）"
for ws in ~/ros_tutorial_ws ~/ros2_workspace ~/ros2_ws ~/robot_vla_ws; do
    if [ -d "$ws" ]; then
        ok "$ws 存在（保护对象）"
    else
        info "$ws 不存在"
    fi
done

# ---------- 9. 网络可达性（决定能否 git clone） ----------
echo
echo "[9] 网络"
if timeout 6 git ls-remote https://github.com/IntelRealSense/librealsense.git HEAD >/dev/null 2>&1; then
    ok "GitHub 可达"
else
    bad "GitHub 不可达（需走离线方案：Mac 下好再拷过来）"
fi
if timeout 6 git ls-remote git@gitee.com:Milo-businu/cloud2_eco65_robot.git HEAD >/dev/null 2>&1; then
    ok "Gitee 可达（SSH 密钥已登记）"
else
    bad "Gitee 不可达（需先登记本机 SSH 公钥）"
fi

echo
echo "=============================================="
echo " 盘点结束。把上面全部输出贴给 AI 即可。"
echo "=============================================="

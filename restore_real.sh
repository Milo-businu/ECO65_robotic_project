#!/bin/bash
# =====================================================================
# restore_real.sh —— 把云服务器的仓库搬到真机后，跑这一个脚本就完成"软件侧"准备
#
# 用法（在真机 beavertech 上，仓库已经 clone 到 ~/robot_vla_ws）：
#   cd ~/robot_vla_ws
#   bash restore_real.sh
#
# 它做什么（全部幂等，可重复跑）：
#   ① 检查 Ubuntu 20.04 / ROS2 Foxy
#   ② 装 Python 依赖（yaml / numpy / opencv / pyserial）
#   ③ 配 udev：把夹爪 1a86:55d4 固定成 /dev/motor_serial
#   ④ 先补编 rm_ros_interfaces / rm_driver / rm_control，再编 arm_grasp
#      （新 clone 的工作空间里这三个 C++ 包没编过时，直接编 arm_grasp
#       会报 "Failed to find .../package.sh"，见第 ④ 步注释）
#   ⑤ 跑 preflight.py：把配置 / FK / IK / 夹爪协议 / 状态机全验一遍
#
# 它【不做】什么（这些必须人来，脚本代劳不了）：
#   ✗ 拨码开关 2 号打到 ON
#   ✗ 手眼标定（25 号清单）
#   ✗ 夹爪开口标定（calibrate_gripper.py）
#   ✗ 把 profile.mode 改成 real（两道锁，防止误动硬件）
#   ✗ 把 install/setup.bash 写进 ~/.bashrc（项目铁律：绝不污染其它工作空间）
# =====================================================================
set -e

WS="${WS:-$HOME/robot_vla_ws}"
PKG="$WS/robot_code/arm_grasp"

echo "============================================================"
echo " ECO65 真机移植 · 软件侧一键准备"
echo " 工作空间: $WS"
echo "============================================================"

# ---------- ① 环境检查 ----------
echo
echo "[1/5] 环境检查"
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "     系统: $PRETTY_NAME"
    case "$VERSION_ID" in
        20.04) echo "     ✅ Ubuntu 20.04" ;;
        *)     echo "     ⚠️  非 20.04，ROS2 Foxy 可能装不上，请留意" ;;
    esac
fi
if [ ! -d /opt/ros/foxy ]; then
    echo "     ❌ /opt/ros/foxy 不存在。先按 README装 ROS2 Foxy。"
    exit 1
fi
echo "     ✅ /opt/ros/foxy 存在"
if [ ! -f "$PKG/config/real_params.yaml" ]; then
    echo "     ❌ 找不到 $PKG/config/real_params.yaml"
    echo "        确认仓库已完整 clone（git clone 到 ~/robot_vla_ws）"
    exit 1
fi
echo "     ✅ 找到 real_params.yaml"
if grep -q "robot_vla_ws/install/setup.bash" "$HOME/.bashrc" 2>/dev/null; then
    echo "     ⚠️  警告：~/.bashrc 里 source 了工作空间 setup.bash，会污染其它 ROS 空间！"
    echo "        按项目铁律请手动删掉那一行，改用内联 source。"
fi

# ---------- ② Python 依赖 ----------
echo
echo "[2/5] 安装 Python 依赖"
sudo apt update -qq
sudo apt install -y python3-yaml python3-numpy python3-opencv python3-pip
python3 -m pip install --user pyserial 2>/dev/null || sudo apt install -y python3-serial
echo "     ✅ 依赖就绪"

# ---------- ③ udev 固定夹爪串口 ----------
echo
echo "[3/5] 配置夹爪 udev 规则（把 1a86:55d4 固定成 /dev/motor_serial）"
RULE=/etc/udev/rules.d/99-ms42dc-gripper.rules
if [ -f "$RULE" ] && grep -q "motor_serial" "$RULE"; then
    echo "     ✅ 规则已存在，跳过"
else
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="motor_serial", GROUP="dialout", MODE="0660"' \
        | sudo tee "$RULE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "     ✅ 已写入 $RULE"
fi
if ! id -nG "$USER" | grep -qw dialout; then
    sudo usermod -aG dialout "$USER"
    echo "     ⚠️  已把 $USER 加入 dialout 组 —— 需要【注销重新登录】才生效"
fi
echo
echo "     ⚠️  硬件三步（脚本做不了，现在就检查）："
echo "        1. 夹爪供电 10~25V 接接口一 XH2.54 4Pin"
echo "        2. USB-C 接电脑（不支持热插拔，插好再上电）"
echo "        3. ★ 拨码开关 2 号打到 ON（1 号保持 OFF）—— 不打 ON 串口收不到数据"
if [ -e /dev/motor_serial ]; then
    echo "     ✅ /dev/motor_serial 已存在 → $(readlink -f /dev/motor_serial)"
else
    echo "     ⚠️  /dev/motor_serial 还没出现（重插 USB / 重登后再看）"
    echo "        临时可用真实设备名：ls /dev/ttyUSB* /dev/ttyACM*"
fi

# ---------- ④ 编译 ----------
# ⚠️ 2026-11-03 真机实测踩坑：
#   新 clone 的工作空间里 ros2_rm_robot 的 C++ 包还没编过时，
#   `colcon build --packages-select arm_grasp` 会报
#   "Failed to find .../install/rm_driver/share/rm_driver/package.sh"。
#   原因：colcon-ros 对 ament_python 包会在构建期检查 exec_depend 包的
#   环境钩子。两处修复：
#     ① package.xml 已把工作空间内 C++ 驱动包从 exec_depend 移除（它们是
#        【运行期】先决条件，不是构建期依赖，见文件内注释）
#     ② 本脚本显式补编整机运行真正需要的 3 个 C++ 包
echo
echo "[4/5] 编译（先补依赖包，再编 arm_grasp）"
source /opt/ros/foxy/setup.bash
cd "$WS"

# 整机运行必须有的三个 C++ 包：消息接口 → 机械臂驱动 → action 桥接
DEPS="rm_ros_interfaces rm_driver rm_control"
MISSING=""
for p in $DEPS; do
    if [ ! -f "$WS/install/$p/share/$p/package.sh" ]; then
        MISSING="$MISSING $p"
    fi
done
if [ -n "$MISSING" ]; then
    echo "     检测到尚未编译的依赖包:$MISSING"
    echo "     先补编它们（C++ 包，约 1~3 分钟）…"
    if ! colcon build --packages-select $MISSING --symlink-install; then
        echo
        echo "     ❌ 依赖包编译失败。别急，抓取包本身是纯 Python，可以不编译就跑："
        echo "        python3 $PKG/scripts/preflight.py                  # 离线自检"
        echo "        python3 $PKG/scripts/grasp_task.py                 # dry-run 演练"
        echo "        ros2 launch $PKG/launch/real_grasp.launch.py      # 用绝对路径起整机"
        echo "        依赖包编译问题按 README排查（缺 rosidl / libapi_cpp.so 等）。"
        echo "        修好后重跑本脚本即可（每一步都可重复跑）。"
        exit 1
    fi
    echo "     ✅ 依赖包编译完成"
else
    echo "     ✅ 依赖包都已编译，跳过"
fi

if ! colcon build --packages-select arm_grasp --symlink-install; then
    echo "     ⚠️  arm_grasp 编译失败，但它是纯 Python 包 —— 不编译也能跑："
    echo "        python3 $PKG/scripts/preflight.py"
    echo "        ros2 launch $PKG/launch/real_grasp.launch.py"
    echo "        （只是不能写成「ros2 launch arm_grasp ...」这种按包名调用的形式）"
else
    echo "     ✅ arm_grasp 编译完成（可用 ros2 launch arm_grasp ...）"
fi

# ---------- ⑤ 离线自检 ----------
echo
echo "[5/5] 跑 preflight.py（不接硬件，把代码链路全验一遍）"
source /opt/ros/foxy/setup.bash
python3 "$PKG/scripts/preflight.py" || true
# 注意：preflight 里有"待办"警告不算失败，脚本不因此中断

echo
echo "============================================================"
echo " 软件侧准备完成 ✅"
echo "============================================================"
echo
echo "接下来的顺序（详见 README）："
echo "  1) python3 $PKG/scripts/calibrate_gripper.py --demo   # 先看懂标定怎么玩"
echo "     python3 $PKG/scripts/calibrate_gripper.py --port /dev/motor_serial"
echo "  2) 按 25 号清单做手眼标定 → 把 4x4 矩阵填进 config/real_params.yaml"
echo "  3) 把 real_params.yaml 里 profile.mode 改 real、allow_arm_motion 改 true"
echo "  4) 开 3 个终端分别确认：相机 / joint_states / 夹爪 / 感知 都有数"
echo "  5) source /opt/ros/foxy/setup.bash && source $WS/install/local_setup.bash"
echo "     ros2 launch arm_grasp real_grasp.launch.py"
echo
echo " ⚠️ 第一次真机务必：清空工作区、急停按钮在手边、speed ≤ 20。"
echo "============================================================"

#!/bin/bash
# =====================================================================
# start_link.sh —— 一条命令起整机链路（自动处理好 source，不用每个终端记两行）
#
# 为什么要它：
#   新开一个终端不 source 工作空间，`ros2 launch arm_grasp ...` 会报
#       Package 'arm_grasp' not found
#   而项目铁律又不允许把 source 写进 ~/.bashrc（会污染其它工作空间）。
#   所以把它封进这个脚本：source → 判断有没有编译过 → 有包名用包名、
#   没有就用绝对路径（纯 Python 包，不编译也能跑）。
#
# 用法：
#   bash ~/robot_vla_ws/start_link.sh                 # 只起链路，不自动抓（默认）
#   bash ~/robot_vla_ws/start_link.sh start_task:=true
#   bash ~/robot_vla_ws/start_link.sh start_web:=true # 顺便起夹爪网页控制
#   bash ~/robot_vla_ws/start_link.sh --show-args     # 只看有哪些参数
# =====================================================================
set -e

WS="${WS:-$HOME/robot_vla_ws}"
PKG_LAUNCH="$WS/robot_code/arm_grasp/launch/real_grasp.launch.py"

if [ ! -f "$PKG_LAUNCH" ]; then
    echo "❌ 找不到 $PKG_LAUNCH"
    echo "   确认仓库已在 $WS（ls ~/robot_vla_ws/robot_code/arm_grasp）"
    exit 1
fi

# ---- source（两行都在这里做掉）----
if [ -f /opt/ros/foxy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/foxy/setup.bash
else
    echo "❌ /opt/ros/foxy 不存在：先装 ROS2 Foxy（26 号手册）"
    exit 1
fi
if [ -f "$WS/install/local_setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS/install/local_setup.bash"
fi

echo "[start_link] 工作空间: $WS"
if ros2 pkg prefix arm_grasp >/dev/null 2>&1; then
    echo "[start_link] 用包名启动（已编译）"
    exec ros2 launch arm_grasp real_grasp.launch.py "$@"
else
    echo "[start_link] 包未编译 → 用绝对路径启动（纯 Python，不影响功能）"
    exec ros2 launch "$PKG_LAUNCH" "$@"
fi

#!/bin/bash
# =====================================================================
# setup_system.sh —— 在任意一台 Ubuntu 20.04 上重建 ROS2 Foxy + Gazebo + 编译项目
#
# 用法：  bash recovery/setup_system.sh
# 依赖：  本仓库的 robot_code/（已含 ros2_rm_robot 等全部源码 + 阻尼补丁）和 models/
# 设计：
#   - 代码来自本仓库，无需再联网 clone 睿尔曼官方包
#   - 隔离约定：绝不把 ~/robot_vla_ws/install/setup.bash 写进全局 .bashrc
#     （会 overlay 污染其它 ROS 工作空间）；工作空间用 alias sb 按需 source
#   - 幂等：重跑安全
# =====================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$HOME/robot_vla_ws"
SRC="$WS/src"

echo "[1/6] 配置 ROS2 软件源（腾讯云内网镜像，速度快）"
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.cloud.tencent.com/ros2/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "[2/6] 安装 ROS2 Foxy 桌面版 + 编译工具（约 10-20 分钟）"
sudo apt update
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep

echo "[3/6] 安装 MoveIt2"
sudo apt install -y ros-foxy-moveit

echo "[4/6] 安装仿真与控制依赖（这几个是踩坑踩出来的）"
sudo apt install -y ros-foxy-xacro \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-ros2-control \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers \
  ros-foxy-joint-state-publisher \
  ros-foxy-tf2-tools \
  gazebo11 libgazebo11-dev xvfb python3-pip

echo "[5/6] 放入项目源码（来自本仓库 robot_code/ + models/，含阻尼补丁）"
mkdir -p "$SRC" "$WS/models"
# 增量覆盖，已存在的文件不会被破坏；排除 .git / 缓存
rsync -a --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
  "$REPO_DIR/robot_code/" "$SRC/"
rsync -a "$REPO_DIR/models/" "$WS/models/"
# 睿尔曼驱动库（若存在）
if [ -f "$SRC/ros2_rm_robot/rm_driver/lib/lib_install.sh" ]; then
  sudo bash "$SRC/ros2_rm_robot/rm_driver/lib/lib_install.sh"
fi

echo "[6/6] 编译（--symlink-install，改代码不用重编）"
grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc \
  || echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source /opt/ros/foxy/setup.bash
cd "$WS"
if [ -d "$SRC/ros2_rm_robot/rm_ros_interfaces" ]; then
  colcon build --packages-select rm_ros_interfaces --symlink-install
  source ./install/setup.bash
fi
colcon build --symlink-install

echo "=== P0 系统 / ROS 环境就绪 ==="
echo "下一步：bash recovery/setup_vla.sh   （装 VLA 推理环境）"

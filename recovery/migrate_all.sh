#!/bin/bash
# ============================================================================
# migrate_all.sh —— ECO65 机械臂项目一键移植脚本
# 
# 适用目标：Ubuntu 20.04 + NVIDIA GPU（驱动 ≥ 450）
# 用法：bash env/migrate_all.sh
# 
# 执行顺序：
#   1. 系统依赖 + ROS2 Foxy + Gazebo
#   2. 项目代码克隆 + 编译
#   3. VLA 推理环境（OpenPI + PyTorch）
#   4. 环境变量写入
#   5. 验证
# ============================================================================
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[$(date +%H:%M:%S)]${NC} $*"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] WARN${NC} $*"; }
err()  { echo -e "${RED}[$(date +%H:%M:%S)] ERROR${NC} $*"; }

# ========================== 预检 ==========================
log "=== 预检：系统版本与 GPU ==="
if ! grep -q '20.04' /etc/os-release 2>/dev/null; then
    warn "非 Ubuntu 20.04，脚本可能不兼容，继续执行..."
fi

if ! command -v nvidia-smi &>/dev/null; then
    err "未检测到 NVIDIA 驱动，请先安装驱动（≥450）"
    exit 1
fi
log "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo '未知')"

# ========================== 第1步：系统依赖 ==========================
log ""
log "=========================================="
log "  第1步：安装系统依赖 + ROS2 Foxy + Gazebo"
log "=========================================="

log "[1.1] 配置 ROS2 软件源（腾讯云内网镜像）"
sudo apt update -qq
sudo apt install -y -qq curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.cloud.tencent.com/ros2/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

log "[1.2] 安装 ROS2 Foxy 桌面版 + 编译工具（约 10-20 分钟）"
sudo apt update -qq
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep

log "[1.3] 安装 MoveIt2"
sudo apt install -y ros-foxy-moveit

log "[1.4] 安装仿真与控制依赖"
sudo apt install -y \
  ros-foxy-xacro \
  ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-ros2-control \
  ros-foxy-ros2-control \
  ros-foxy-ros2-controllers \
  ros-foxy-joint-state-publisher \
  ros-foxy-tf2-tools

log "[1.5] 安装 Gazebo 11 + 辅助工具"
sudo apt install -y gazebo11 libgazebo11-dev xvfb python3-pip

log "[1.6] 安装 Python 依赖（matplotlib + numpy）"
pip3 install --upgrade pip -q
pip3 install matplotlib numpy -q

log "第1步完成 ✓"

# ========================== 第2步：项目克隆与编译 ==========================
log ""
log "=========================================="
log "  第2步：克隆项目代码 + 编译"
log "=========================================="

PROJECT_DIR="$HOME/robot_vla_ws"

if [ ! -d "$PROJECT_DIR/src" ]; then
    log "[2.1] 克隆项目仓库"
    mkdir -p "$PROJECT_DIR/src"
    cd "$PROJECT_DIR/src"
    
    # 克隆主项目（从 Gitee，国内速度快）
    git clone -b main https://gitee.com/Milo-businu/eco65_robot.git temp_repo
    # 把主项目内容移到工作空间根目录
    cd temp_repo
    # 移动 src 下的自定义包
    for pkg in arm_control arm_perception arm_handeye arm_calibration arm_grasp; do
        if [ -d "src/$pkg" ]; then
            cp -r "src/$pkg" "$PROJECT_DIR/src/"
        fi
    done
    # 移动 env、models、docs
    cp -r env "$PROJECT_DIR/" 2>/dev/null || true
    cp -r models "$PROJECT_DIR/" 2>/dev/null || true
    cp -r docs "$PROJECT_DIR/" 2>/dev/null || true
    cp .gitignore "$PROJECT_DIR/" 2>/dev/null || true
    cd "$PROJECT_DIR"
    rm -rf "$PROJECT_DIR/src/temp_repo"
    
    # 初始化 Git
    cd "$PROJECT_DIR"
    git init
    git add -A
    git commit -m "初始移植"
    
    log "[2.2] 克隆睿尔曼官方 ROS2 驱动（foxy 分支）"
    cd "$PROJECT_DIR/src"
    if [ ! -d ros2_rm_robot ]; then
        git clone -b foxy https://github.com/RealManRobot/ros2_rm_robot.git
    fi
    
    log "[2.3] 安装睿尔曼驱动库"
    if [ -f "$PROJECT_DIR/src/ros2_rm_robot/rm_driver/lib/lib_install.sh" ]; then
        cd "$PROJECT_DIR/src/ros2_rm_robot/rm_driver/lib"
        sudo bash lib_install.sh
    fi
else
    log "[2.1] 项目目录已存在，跳过克隆"
fi

log "[2.4] 编译项目"
source /opt/ros/foxy/setup.bash
cd "$PROJECT_DIR"

# 先编译接口包
if [ -d "src/ros2_rm_robot/rm_ros_interfaces" ]; then
    colcon build --packages-select rm_ros_interfaces --symlink-install
    source ./install/setup.bash
fi

# 全量编译
colcon build --symlink-install

log "[2.5] 修复功能包 console_scripts 入口（scripts 移入包内成为子包）"
cd "$PROJECT_DIR/src"
for p in arm_control arm_perception arm_handeye arm_calibration; do
    if [ -d "$p/scripts" ] && [ ! -d "$p/$p/scripts" ]; then
        mkdir -p "$p/$p"
        mv "$p/scripts" "$p/$p/scripts"
        log "  已将 $p/scripts 移入 $p/$p/scripts"
    fi
done

log "[2.6] 补充腕部深度相机到 URDF（无头服务器需 Xvfb 才能启用渲染）"
URDF_FILE="$PROJECT_DIR/src/ros2_rm_robot/rm_description/urdf/rm_eco65_gazebo.urdf.xacro"
if [ -f "$URDF_FILE" ] && ! grep -q 'wrist_depth_camera' "$URDF_FILE"; then
    cat >> "$URDF_FILE" <<'URDF_EOF'

  <!-- 腕部深度相机（Eye-in-Hand）：固定在 Link6 上 -->
  <link name="camera_link">
    <visual>
      <geometry><box size="0.04 0.09 0.03"/></geometry>
      <material name="cam_blue"><color rgba="0.1 0.2 0.8 1"/></material>
    </visual>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="1e-4" iyy="2e-4" izz="2e-4" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="Link6"/>
    <child link="camera_link"/>
    <origin xyz="0.06 0 0.02" rpy="0 0 1.5708"/>
  </joint>
  <gazebo reference="camera_link">
    <sensor type="depth" name="wrist_depth_camera">
      <update_rate>15</update_rate>
      <camera>
        <horizontal_fov>1.8</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>8</far></clip>
      </camera>
      <plugin name="wrist_camera_plugin" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
URDF_EOF
    log "  已补充腕部深度相机定义到 URDF"
fi

# 重新编译使 console_scripts 生效
cd "$PROJECT_DIR"
source /opt/ros/foxy/setup.bash
colcon build --symlink-install

log "第2步完成 ✓"

# ========================== 第3步：VLA 推理环境 ==========================
log ""
log "=========================================="
log "  第3步：安装 VLA 推理环境（OpenPI + PyTorch）"
log "=========================================="

log "[3.1] 安装 miniconda（与 ROS Python 3.8 完全隔离）"
if [ ! -d ~/miniconda3 ]; then
    wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
    bash /tmp/miniconda.sh -b -p ~/miniconda3
fi
source ~/miniconda3/etc/profile.d/conda.sh
unset PYTHONPATH
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main 2>/dev/null || true
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r    2>/dev/null || true

log "[3.2] 创建 openpi 环境（Python 3.11）"
if ! conda env list | grep -q '^openpi'; then
    conda create -y -n openpi python=3.11
fi
conda activate openpi

log "[3.3] 安装 PyTorch（CUDA 11.8）"
pip install --upgrade pip -q
pip install --index-url https://download.pytorch.org/whl/cu118 torch torchvision

log "[3.4] 安装 VLA 服务依赖"
pip install fastapi uvicorn pillow numpy

log "[3.5] 验证 GPU"
python -c "import torch; print('CUDA 可用:', torch.cuda.is_available()); print('显卡:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A')"

log "第3步完成 ✓"

# ========================== 第4步：环境变量 ==========================
log ""
log "=========================================="
log "  第4步：写入环境变量"
log "=========================================="

if ! grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    log "已添加 ROS2 环境变量"
fi

if ! grep -q "source $PROJECT_DIR/install/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "source $PROJECT_DIR/install/setup.bash" >> ~/.bashrc
    log "已添加工作空间环境变量"
fi

if ! grep -q "alias sb=" ~/.bashrc 2>/dev/null; then
    echo "alias sb='source $PROJECT_DIR/install/setup.bash'" >> ~/.bashrc
    log "已添加快捷命令 sb"
fi

log "第4步完成 ✓"

# ========================== 第5步：验证 ==========================
log ""
log "=========================================="
log "  第5步：验证"
log "=========================================="

source /opt/ros/foxy/setup.bash
source "$PROJECT_DIR/install/setup.bash" 2>/dev/null || true

log "[5.1] ROS2 版本"
ros2 --version 2>/dev/null || warn "ros2 命令不可用"

log "[5.2] 工作空间包列表"
ros2 pkg list 2>/dev/null | grep -E 'arm_|rm_' || warn "未找到项目包"

log "[5.3] Gazebo"
gzserver --version 2>/dev/null || warn "gzserver 不可用"

log "[5.4] URDF 生成测试"
cd "$PROJECT_DIR"
if [ -f src/ros2_rm_robot/rm_gazebo/config/gazebo_eco65_6fb_description.urdf.xacro ]; then
    source /opt/ros/foxy/setup.bash
    source install/setup.bash 2>/dev/null || true
    ros2 run xacro xacro src/ros2_rm_robot/rm_gazebo/config/gazebo_eco65_6fb_description.urdf.xacro \
        link6_type:=Link6_6fb > /tmp/eco65_6fb.urdf 2>/dev/null
    if [ -s /tmp/eco65_6fb.urdf ]; then
        log "URDF 生成成功 → /tmp/eco65_6fb.urdf"
    else
        warn "URDF 生成失败"
    fi
fi

log "[5.5] 无头服务器相机验证（需 Xvfb 虚拟显示启用 Gazebo 渲染）"
if ! pgrep -f 'Xvfb :99' >/dev/null; then
    setsid Xvfb :99 -screen 0 1280x720x24 > /tmp/xvfb.log 2>&1 &
    sleep 2
fi
if command -v ros2 >/dev/null && pgrep -f 'Xvfb :99' >/dev/null; then
    export DISPLAY=:99
    setsid bash -c 'ros2 launch rm_gazebo gazebo_eco65_6fb_demo.launch.py > /tmp/gazebo_verify.log 2>&1' &
    sleep 30
    if timeout 10 ros2 topic list 2>/dev/null | grep -q 'wrist_depth_camera/image_raw'; then
        log "相机话题 /wrist_depth_camera/image_raw 已发布 ✓"
    else
        warn "相机话题未出现，请检查 Gazebo 渲染引擎日志"
    fi
    pkill -f 'gazebo_eco65_6fb_demo' 2>/dev/null || true
fi

log ""
log "=========================================="
log "  移植完成！"
log "=========================================="
log ""
log "后续操作："
log "  1. 新终端生效：source ~/.bashrc"
log "  2. 启动仿真：ros2 launch rm_gazebo gazebo_eco65_6fb_demo.launch.py"
log "  3. 运行工作空间扫描：python3 src/arm_control/scripts/workspace_scan.py"
log "  4. VLA 环境：conda activate openpi"
log ""

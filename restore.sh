#!/bin/bash
# =====================================================================
# restore.sh —— 在新 Ubuntu 20.04 + NVIDIA 服务器上"一口气"恢复 ECO65 整机环境
#
# 用法：
#   git clone git@gitee.com:Milo-businu/cloud2_eco65_robot.git
#   cd cloud2_eco65_robot
#   bash restore.sh
#
# 设计三原则：最快 / 最不容易出错 / 最安全
#   - 代码来自本仓库 robot_code/，不依赖外网 clone 睿尔曼包
#   - 隔离：绝不把工作空间 install/setup.bash 写进全局 .bashrc（写进 ~/.bash_aliases）
#   - 幂等：每大步可单独跑；失败会停在当前步，修好后重跑即可
#   - 安全：权重(12G)默认不自动下载（国内易失败），单独确认后再下
#
# 注意：本脚本会 sudo apt，请用有 sudo 权限的账户运行；需要联网
#       （ROS apt 源 + OpenPI/LeRobot 的 GitHub clone + 权重下载）。
# =====================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR"

echo "============================================================"
echo "  ECO65 整机恢复脚本（cloud2_eco65_robot）"
echo "============================================================"
echo "[预检] 系统 / GPU / 磁盘"
if ! grep -q '20.04' /etc/os-release 2>/dev/null; then
  echo "警告：非 Ubuntu 20.04，兼容性未知，继续…"
fi
command -v nvidia-smi >/dev/null || { echo "错误：未检测到 NVIDIA 驱动，请先装驱动(>=450)"; exit 1; }
echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null || echo 未知)"
FREE_GB=$(df -BG ~ | awk 'NR==2{print $4}' | tr -d G)
if [ "${FREE_GB:-0}" -lt 40 ]; then
  echo "警告：家目录剩余 ${FREE_GB}GB < 40GB，权重+conda 可能不够，先 df -h 看看"
fi

echo
echo ">>>> [步骤 1/5] 系统 + ROS2 Foxy + 编译项目（约 20-40 分钟）"
bash recovery/setup_system.sh

echo
echo ">>>> [步骤 2/5] VLA 推理环境（OpenPI + LeRobot 钉死，约 10-30 分钟）"
bash recovery/setup_vla.sh

echo
echo ">>>> [步骤 3/5] 写入别名（隔离方式：追加到 ~/.bash_aliases）"
if [ -f recovery/bashrc_additions ]; then
  if ! grep -q "ECO65 机械臂项目" ~/.bash_aliases 2>/dev/null; then
    cat recovery/bashrc_additions >> ~/.bash_aliases
    echo "已追加别名到 ~/.bash_aliases（新终端自动生效）"
  else
    echo "别名已存在，跳过"
  fi
fi

echo
echo ">>>> [步骤 4/5] 下载 VLA 权重（π0.5-DROID，约 12GB）"
read -r -p "现在就下载权重吗？(y/N，国内可能很慢/不通，建议先 N，之后单独跑 recovery/get_weights.sh) " ANS
if [ "$ANS" = "y" ] || [ "$ANS" = "Y" ]; then
  bash recovery/get_weights.sh
else
  echo "跳过。之后手动：bash recovery/get_weights.sh（或 Mac 下好再 scp 到 ~/openpi/checkpoints/）"
fi

echo
echo ">>>> [步骤 5/5] 验收清单（请新开终端逐项执行）"
echo "------------------------------------------------------------"
echo "  source ~/.bash_aliases"
echo "  sb"
echo "  urdf            # 生成 /tmp/eco65_6fb.urdf（每次重编译后先跑）"
echo "  sim             # 起仿真，出现 Gazebo 窗口即成功"
echo "  fk              # 运动学自检（期望与黄金指纹一致）"
echo "  ik              # 逆解自检"
echo "  bash recovery/js_drift.sh 300 60   # 关节错位回归（Δj 全程 ~0.0000）"
echo "  # VLA 推理自检（需权重）："
echo "  conda activate openpi"
echo "  cd ~/openpi && MALLOC_CONF=background_thread:false ARROW_DEFAULT_MEMORY_POOL=system python -u test_infer.py"
echo "------------------------------------------------------------"

echo
echo "============================================================"
echo "  恢复完成！详细排错见 docs/机械臂整机开发文档/18_新云服务器完整恢复手册_小白版.md"
echo "============================================================"

#!/bin/bash
# =====================================================================
# setup_vla.sh —— 在 Ubuntu 20.04 + NVIDIA 显卡机器上重建 VLA 推理环境
#
# 用法：  bash recovery/setup_vla.sh
# 核心原则：与 ROS2 Foxy(Python3.8) 完全隔离 —— 独立 conda 环境 openpi(Python3.11)
# 关键钉死（来自 requirements/vla_requirements.txt / openpi_pinned_versions.txt）：
#   OpenPI @ commit 215abfb  +  LeRobot @ rev 0cf8648  +  三钉 numpy/pyarrow/datasets
# 绝不执行 conda init（不写 .bashrc，避免污染 ROS 终端）
#
# ⚠️ OpenPI / LeRobot 需从 GitHub clone；国内若不通，见本脚本末尾"离线回退"。
# =====================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIP=~/miniconda3/envs/openpi/bin/pip
PY=~/miniconda3/envs/openpi/bin/python
OPENPI_COMMIT=215abfb217dbac7d5f1273282331b9b1866c0479
LEROBO_REV=0cf864870cf29f4738d3ade893e6fd13fbd7cdb5

echo "[1/5] 安装 miniconda（若已存在则跳过）"
if [ ! -d ~/miniconda3 ]; then
  wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh
  bash /tmp/miniconda.sh -b -p ~/miniconda3
fi
source ~/miniconda3/etc/profile.d/conda.sh
unset PYTHONPATH   # ROS 会话会设置 PYTHONPATH，清掉避免污染 VLA 环境
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main 2>/dev/null || true
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r    2>/dev/null || true

echo "[2/5] 创建独立环境 openpi（Python 3.11）"
if ! conda env list | grep -q '^openpi'; then
  conda create -y -n openpi python=3.11
fi
conda activate openpi

echo "[3/5] 安装 OpenPI 本体 + 钉死 LeRobot + 三钉回滚"
if [ ! -d ~/openpi/.git ]; then
  git clone https://github.com/Physical-Intelligence/openpi.git ~/openpi
fi
cd ~/openpi
git checkout -q "$OPENPI_COMMIT" 2>/dev/null || true
git submodule update --init --recursive 2>/dev/null || true

# 装 openpi 本体及其依赖（用 conda 环境的 pip，而非 .venv）
$PIP install --upgrade pip -q
$PIP install -e . -q || $PIP install -e . --no-build-isolation -q

# 钉死 LeRobot（旧版，必须这个 rev，否则 import 报错）
$PIP install "git+https://github.com/huggingface/lerobot.git@$LEROBO_REV" -q

# 三钉复查：装 lerobot 会把 numpy 顶到 2.x，必须回滚（红字 conflict 忽略）
$PIP install "numpy==1.26.4" "pyarrow==17.0.0" "datasets==2.21.0" -q

echo "[4/5] 安装 VLA 服务依赖 + 放入自定义文件"
$PIP install fastapi uvicorn pillow -q
cp -f "$REPO_DIR/recovery/vla_server.py"  ~/openpi/vla_server.py
cp -f "$REPO_DIR/recovery/test_infer.py" ~/openpi/test_infer.py

echo "[5/5] 验证"
$PY -c "import torch; print('CUDA 可用:', torch.cuda.is_available()); print('显卡:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A')"
$PY -c "import openpi, lerobot; print('openpi + lerobot import OK')"
$PY -c "import numpy, pyarrow, datasets; assert numpy.__version__=='1.26.4' and pyarrow.__version__=='17.0.0' and datasets.__version__=='2.21.0', '三钉未生效'; print('三钉 OK', numpy.__version__, pyarrow.__version__, datasets.__version__)"

echo "=== VLA 环境就绪 ==="
echo "推理自测（需权重）：conda activate openpi && cd ~/openpi && MALLOC_CONF=background_thread:false ARROW_DEFAULT_MEMORY_POOL=system python -u test_infer.py"

# ---------------------------------------------------------------------
# 离线回退（GitHub 被墙时）：从还能用的旧服务器直接拷 OpenPI 源码（去掉 .git/.venv/checkpoints）
#   在旧服务器上：  tar czf /tmp/openpi_src.tgz --exclude=.git --exclude=.venv --exclude=checkpoints --exclude='*.pyc' -C ~ openpi
#   在旧服务器上：  tar czf /tmp/conda_openpi.tgz -C ~/miniconda3/envs openpi
#   在新服务器上：  scp 旧服务器:/tmp/openpi_src.tgz ~ && mkdir -p ~/openpi && tar xzf ~/openpi_src.tgz -C ~
#                  scp 旧服务器:/tmp/conda_openpi.tgz ~/miniconda3/envs && tar xzf ~/miniconda3/envs/conda_openpi.tgz -C ~/miniconda3/envs
#   然后重跑本脚本第 4/5 步（把自定义文件拷进去 + 三钉复查）即可。
# ---------------------------------------------------------------------

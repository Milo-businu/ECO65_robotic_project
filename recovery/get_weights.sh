#!/usr/bin/env bash
# get_weights.sh —— 下载 π0.5-DROID 权重到 ~/openpi/checkpoints/
#
# 用法（新服务器上）：
#   bash ~/get_weights.sh
#
# 说明：openpi 的权重存放在 Google Cloud Storage 公开桶 gs://openpi-assets。
#       国内云服务器访问可能很慢或直接不通，脚本失败就换路线 B（见 README 第 10 步）。
set -e

PY="${PY:-$HOME/miniconda3/envs/openpi/bin/python}"
CKPT_DIR="${CKPT_DIR:-$HOME/openpi/checkpoints}"
GS="$(dirname "$PY")/gsutil"

if [ ! -x "$PY" ]; then
  echo "找不到 Python：$PY —— 先做完 README 第 9 步（装 conda openpi 环境）"
  exit 1
fi

mkdir -p "$CKPT_DIR"

if [ ! -x "$GS" ]; then
  echo "安装 gsutil ..."
  "$PY" -m pip install -q gsutil
fi

echo "开始下载（几 GB，耐心等）..."
"$GS" -m cp -r gs://openpi-assets/checkpoints/pi05_droid "$CKPT_DIR"/

echo "下载完成，内容如下："
ls -la "$CKPT_DIR/pi05_droid" | head -20
du -sh "$CKPT_DIR/pi05_droid" 2>/dev/null || true

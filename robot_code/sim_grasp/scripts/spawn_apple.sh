#!/bin/bash
# spawn_apple.sh —— 在已生成的绿色方块旁生成红色苹果🍎（方案A 语义通道测试用）
#
# 用法:
#   bash ~/robot_vla_ws/robot_code/sim_grasp/scripts/spawn_apple.sh          # 默认偏移 13cm
#   bash ~/robot_vla_ws/robot_code/sim_grasp/scripts/spawn_apple.sh 0.10     # 自定义偏移(米)
#
# 原理: 先查 target_box 的真实位姿（gz model -p），再沿 +y 方向偏移 OFFSET 生成苹果。
#       —— 与方块生成在哪、用哪条启动流程都无关，保证“不远也不近”且两者同进腕部相机视野。
#       幂等：若 red_apple 已存在会先删除再生成。
set -u
source /opt/ros/foxy/setup.bash 2>/dev/null
source ~/robot_vla_ws/install/setup.bash 2>/dev/null

OFFSET="${1:-0.13}"                                  # 与方块距离（米），默认 13cm
APPLE_SDF=~/robot_vla_ws/models/red_apple/model.sdf

if [ ! -f "$APPLE_SDF" ]; then
  echo "[spawn_apple] 找不到 $APPLE_SDF —— 红色苹果模型没建好？"
  exit 1
fi

# 查方块位姿（gz model -p 输出单行 6 个数: x y z roll pitch yaw）
POSE=$(gz model -m target_box -p 2>/dev/null | tail -1)
if [ -z "$POSE" ]; then
  echo "[spawn_apple] 没找到 target_box —— 请先生成绿色方块，再跑本脚本"
  exit 1
fi
X=$(echo "$POSE" | awk '{print $1}')
Y=$(echo "$POSE" | awk '{print $2}')
AX=$(python3 -c "print(round(float('$X'),4))")
AY=$(python3 -c "print(round(float('$Y')+float('$OFFSET'),4))")
echo "[spawn_apple] 方块在 (x=$X, y=$Y)；苹果将生成在 (x=$AX, y=$AY)，距方块 ${OFFSET}m"

# 幂等：已存在先删
gz model -m red_apple -d >/dev/null 2>&1 || true
sleep 1

ros2 run gazebo_ros spawn_entity.py -entity red_apple \
     -file "$APPLE_SDF" -x "$AX" -y "$AY" -z 0.0 \
     > /tmp/spawn_apple.log 2>&1
sleep 3

echo "[spawn_apple] 苹果生成结果："
gz model -m red_apple -p 2>/dev/null | tail -1
echo "[spawn_apple] 详细日志见 /tmp/spawn_apple.log"

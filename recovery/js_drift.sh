#!/bin/bash
# js_drift.sh —— /joint_states 位置漂移量化（ROS2 Foxy）
# 用法: bash js_drift.sh [总时长秒=300] [采样间隔秒=60]
# 注意: 位置输出顺序为 j2,j3,j1,j4,j5,j6（睿尔曼硬件接口注册序，非异常）
# 用法示例: 先 killgaz -> sim -> 发两点式轨迹到位，然后:
#   bash ~/js_drift.sh 300 60

DUR=${1:-300}; STEP=${2:-60}
OUT=/tmp/js_drift
rm -rf "$OUT"; mkdir -p "$OUT"

# 环境保险：脚本内自行 source（只在子进程生效，不污染当前终端）
command -v ros2 >/dev/null 2>&1 || {
  source /opt/ros/foxy/setup.bash
  source ~/robot_vla_ws/install/setup.bash
}

echo ">> 采样 ${DUR}s，每 ${STEP}s 一张快照。请确保已发完两点式轨迹、处于到位保持状态。"
END=$((SECONDS+DUR))
while [ $SECONDS -lt $END ]; do
  timeout 2 ros2 topic echo /joint_states > "$OUT/snap_$(printf '%04d' $SECONDS).txt" 2>/dev/null
  sleep "$STEP"
done

echo
echo "秒    |   j2    |   j3    |   j1    |   j4    |   j5    |   j6    |  v_j2"
echo "------+---------+---------+---------+---------+---------+---------+----------"
first_j2=""
for f in "$OUT"/snap_*.txt; do
  [ -s "$f" ] || continue
  ts=$(basename "$f" .txt | sed 's/snap_//')
  j2=$(grep -a -A8 '^position' "$f" | sed -n '2p'   | awk '{print $2}')
  j3=$(grep -a -A8 '^position' "$f" | sed -n '3p'   | awk '{print $2}')
  j1=$(grep -a -A8 '^position' "$f" | sed -n '4p'   | awk '{print $2}')
  j4=$(grep -a -A8 '^position' "$f" | sed -n '5p'   | awk '{print $2}')
  j5=$(grep -a -A8 '^position' "$f" | sed -n '6p'   | awk '{print $2}')
  j6=$(grep -a -A8 '^position' "$f" | sed -n '7p'   | awk '{print $2}')
  v2=$(grep -a -A2 '^velocity' "$f" | sed -n '2p'   | awk '{print $2}')
  [ -z "$first_j2" ] && first_j2="$j2"
  d=$(awk -v a="$j2" -v b="$first_j2" 'BEGIN{printf "%+.4f", a-b}')
  printf "%-6s|%9s|%9s|%9s|%9s|%9s|%9s|%9s  (Δj2=%s)\n" "$ts" "$j2" "$j3" "$j1" "$j4" "$j5" "$j6" "$v2" "$d"
done
echo
echo "判读: Δj2 随时间持续增大 = 物理在漂移; 全程 |Δj2|<0.005 = 已稳。"

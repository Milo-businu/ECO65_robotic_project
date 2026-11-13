#!/bin/bash
# cleanup.sh —— 彻底清场，保证只有一个 gzserver / robot_state_publisher
#
# ⚠️ 巨坑记录：Linux 进程名(comm)被内核截断到 15 字符，
#    robot_state_publisher 实际叫 "robot_state_pub"，
#    所以 pkill -9 -x robot_state_publisher 从来就没生效过！
#    必须按【完整命令行路径】匹配。
#    后果：旧实例一直存活，用 latched QoS 抢发旧 robot_description，
#    导致 Gazebo 拿到的是旧 URDF（改 velocity/effort 全都不生效）。

for p in gzserver gzclient joint_state_publisher rviz2 ruby; do
    pkill -9 -x "$p" 2>/dev/null
done

# 按完整可执行路径杀（[/] 括号技巧避免匹配到本脚本自身）
pkill -9 -f "[/]robot_state_publisher/robot_state_publisher" 2>/dev/null
pkill -9 -f "[s]pawner.py"       2>/dev/null
pkill -9 -f "[c]amera_sim_min"   2>/dev/null
pkill -9 -f "[r]os2 launch"      2>/dev/null
pkill -9 -f "[g]azebo_ros"       2>/dev/null
sleep 3

echo "剩余 gzserver:              $(pgrep -x gzserver | wc -l)"
echo "剩余 robot_state_publisher: $(ps -eo args= | grep -c '[/]robot_state_publisher/robot_state_publisher')"
echo "剩余 ros2 launch:           $(ps -eo args= | grep -c '[r]os2 launch')"
echo "--- 明细 ---"
pgrep -a -x gzserver
ps -eo pid,etime,args= | grep '[/]robot_state_publisher/robot_state_publisher'
echo "--- 清理完成 ---"

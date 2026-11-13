#!/bin/bash
source /opt/ros/foxy/setup.bash
source ~/robot_vla_ws/install/setup.bash
export DISPLAY=:99; export LIBGL_ALWAYS_SOFTWARE=1
export GZ_WORLD=/tmp/grasp.world
export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugins:$GAZEBO_PLUGIN_PATH
pgrep -f '[X]vfb' >/dev/null || { Xvfb :99 -screen 0 1280x1024x24 >/dev/null 2>&1 & sleep 2; }
bash /tmp/cleanup.sh
cd /tmp
setsid nohup ros2 launch ~/robot_vla_ws/src/arm_perception/launch/camera_sim_min.launch.py \
     use_gazebo_gui:=false > /tmp/eff.log 2>&1 < /dev/null &
sleep 50
{
  echo "=== 硬件接口 ==="
  ros2 control list_hardware_interfaces 2>&1
  echo "=== 控制器 ==="
  ros2 control list_controllers 2>&1
  echo "=== 插件日志里 finger 的 Command ==="
  grep -a -A6 "Loading joint: finger_left_joint" /tmp/eff.log
} > /tmp/eff_hw.txt 2>&1
echo DONE > /tmp/eff.flag

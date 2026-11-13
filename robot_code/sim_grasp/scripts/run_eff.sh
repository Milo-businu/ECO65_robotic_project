#!/bin/bash
# run_eff.sh —— effort(力矩)驱动版抓取闭环
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/foxy/setup.bash
source ~/robot_vla_ws/install/setup.bash
# 现场用本机 xacro 生成 IK 用的 URDF：
# 不要直接用别的机器拷来的 config/eco65_6fb.urdf —— 里面 mesh/配置路径写的是那个机器的用户名
xacro ~/robot_vla_ws/src/ros2_rm_robot/rm_description/urdf/rm_eco65.urdf link6_type:=Link6_6fb > /tmp/eco65_6fb.urdf 2>/dev/null
if [ -s /tmp/eco65_6fb.urdf ]; then
    export ECO65_URDF=/tmp/eco65_6fb.urdf
else
    export ECO65_URDF="$ROOT/config/eco65_6fb.urdf"   # 兜底：用随包那份
fi
source ~/robot_vla_ws/install/setup.bash
# ---------- 显示设置（默认【开画面】）----------
# 有真实显示器（真机桌面）：直接用真屏 + 硬件渲染（快）
# 没有（云服务器 / 纯后台）：起 Xvfb 虚拟屏 :99 + 软件渲染（兜底，会慢）
GUI="${GZ_GUI:-true}"
if [ -n "$DISPLAY" ]; then
    echo "[run_eff] 使用现有显示器 $DISPLAY（硬件渲染）"
else
    echo "[run_eff] 未检测到显示器，改用虚拟屏 :99（软件渲染，会明显变慢）"
    export LIBGL_ALWAYS_SOFTWARE=1
fi
export GZ_WORLD="$ROOT/config/grasp.world"
export GAZEBO_PLUGIN_PATH=$HOME/gazebo_plugins:$GAZEBO_PLUGIN_PATH

if [ -z "$DISPLAY" ]; then
    pgrep -f '[X]vfb' >/dev/null || { Xvfb :99 -screen 0 1280x1024x24 >/dev/null 2>&1 & sleep 2; }
    export DISPLAY=:99
fi
bash "$ROOT/scripts/cleanup.sh"

cd /tmp
setsid nohup ros2 launch ~/robot_vla_ws/src/arm_perception/launch/camera_sim_min.launch.py \
     use_gazebo_gui:=$GUI > /tmp/sim_eff.log 2>&1 < /dev/null &
# 轮询等待控制器就绪（避免启动竞态）
for i in $(seq 1 40); do
  if ros2 control list_controllers 2>/dev/null | grep -q 'rm_group_controller.*active'; then
    echo "控制器就绪，等待 $((i*3)) 秒"; break
  fi
  sleep 3
done
sleep 5

{
  echo "=== 指令接口 ==="
  ros2 control list_hardware_interfaces 2>&1 | sed -n "/command interfaces/,/state interfaces/p"
  echo "=== controller ==="
  ros2 control list_controllers 2>&1
} > /tmp/eff_hw2.txt 2>&1

ros2 run gazebo_ros spawn_entity.py -entity target_box \
     -file ~/robot_vla_ws/models/target_box/model.sdf \
     -x 0.40 -y 0.0 -z 0.0 > /tmp/spawn_eff.log 2>&1
sleep 4
gz model -m target_box -p > /tmp/box_eff_before.txt 2>&1

python3 -u "$ROOT/scripts/eff_test.py" > /tmp/eff_run.log 2>&1

sleep 2
gz model -m target_box -p > /tmp/box_eff_after.txt 2>&1
echo DONE > /tmp/eff_done.flag

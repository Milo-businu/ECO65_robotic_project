#!/bin/bash
# apply_patches.sh —— 一键恢复「硬件对齐仿真抓取闭环」的全部改动
#
# 为什么需要这个脚本：
#   .gitignore 第 1 行忽略了整个 src/ros2_rm_robot/，
#   所以「手指 effort 接口」和「effort 控制器」这两处最关键的改动
#   git 永远不会保存。它们以补丁形式存在 sim_grasp/patches/ 下，靠本脚本回填。
#
# 用法：bash sim_grasp/scripts/apply_patches.sh
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "$ROOT/.." && pwd)"
SRC="$WS/src/ros2_rm_robot"

echo "仓库根: $WS"
echo "补丁源: $ROOT/patches"
echo

# ---------- 1. 回填 ros2_rm_robot 里被 gitignore 的两处关键改动 ----------
echo "=== 1) 回填手指 effort 接口 ==="
if [ -d "$SRC/rm_gazebo/config" ]; then
    cp "$ROOT/patches/gazebo_eco65_description.urdf.xacro" "$SRC/rm_gazebo/config/"
    echo "  ✅ rm_gazebo/config/gazebo_eco65_description.urdf.xacro"
else
    echo "  ❌ 找不到 $SRC/rm_gazebo/config —— ros2_rm_robot 未安装，请先恢复该包"
fi

echo "=== 2) 回填 effort 控制器 ==="
if [ -d "$SRC/rm_moveit2_config/rm_eco65_config/config" ]; then
    cp "$ROOT/patches/ros2_controllers.yaml" "$SRC/rm_moveit2_config/rm_eco65_config/config/"
    echo "  ✅ rm_eco65_config/config/ros2_controllers.yaml"
else
    echo "  ❌ 找不到 rm_eco65_config/config —— 请先恢复该包"
fi

# ---------- 2. 恢复并编译 gazebo_grasp_fix 插件 ----------
echo "=== 3) 恢复 gazebo_grasp_fix 插件 ==="
if [ ! -d "$HOME/gazebo-pkgs" ]; then
    echo "  本地无 gazebo-pkgs，正在 clone..."
    git clone https://github.com/JenniferBuehler/gazebo-pkgs.git "$HOME/gazebo-pkgs"
fi

cd "$HOME/gazebo-pkgs"
if git apply --check "$ROOT/patches/gazebo_grasp_fix.patch" 2>/dev/null; then
    git apply "$ROOT/patches/gazebo_grasp_fix.patch"
    echo "  ✅ 插件补丁已应用"
else
    echo "  ⚠️  补丁无法直接应用（可能已应用过）。尝试反查..."
    if git apply --reverse --check "$ROOT/patches/gazebo_grasp_fix.patch" 2>/dev/null; then
        echo "  ✅ 补丁已处于应用状态，跳过"
    else
        echo "  ❌ 补丁状态异常，请手动检查 $HOME/gazebo-pkgs"
    fi
fi

echo "=== 4) 生成 protobuf 消息并编译插件 ==="
cd "$HOME/gazebo-pkgs/gazebo_grasp_plugin"
if [ ! -f msgs/grasp_event.pb.cc ]; then
    protoc --cpp_out=msgs -I msgs msgs/grasp_event.proto
    echo "  ✅ 已生成 grasp_event.pb.cc"
fi
mkdir -p "$HOME/gazebo_plugins"
bash "$ROOT/scripts/build_plugin.sh" >/dev/null 2>&1 \
    && echo "  ✅ 插件已编译 → $HOME/gazebo_plugins/libgazebo_grasp_fix.so" \
    || echo "  ❌ 插件编译失败，手动跑: bash $ROOT/scripts/build_plugin.sh"

# ---------- 3. 回填夹爪几何 / 惯量 / 插件参数 ----------
echo "=== 5) 夹爪与惯量参数（在 xacro 里，git 已跟踪） ==="
X="$WS/src/arm_perception/urdf/eco65_camera.urdf.xacro"
if [ -f "$X" ]; then
    grep -q 'izz="1.0e-3"' "$X" && echo "  ✅ 手指等效惯量 1.0e-3（含电机折算惯量）" \
        || echo "  ❌ 惯量未设置！应为 ixx=iyy=6.0e-4, izz=1.0e-3"
    grep -q 'palm_link>Link6' "$X" && echo "  ✅ palm_link=Link6（URDF→SDF 会合并 gripper_base）" \
        || echo "  ❌ palm_link 应为 Link6"
    grep -q 'gazebo_grasp_fix' "$X" && echo "  ✅ grasp_fix 插件已挂载" \
        || echo "  ❌ 缺少 grasp_fix 插件块"
else
    echo "  ❌ 找不到 $X"
fi

echo
echo "=== 恢复完成，接下来 ==="
echo "  bash $ROOT/scripts/run_eff.sh"

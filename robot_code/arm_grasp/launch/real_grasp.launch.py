# -*- coding: utf-8 -*-
"""整机一键启动 —— 机械臂 + 夹爪 + 相机 + 感知 + 抓取任务总控

一条命令把这条链全部拉起来：

  rm_driver ──(话题)──> 机械臂            /joint_states, /rm_driver/movej_cmd
  realsense2_camera ──> 彩色/对齐深度      /camera/d435i/...
  gripper_node ──────> MS42DC 串口        /gripper/*
  perception_node ───> 目标 3D 位姿       /grasp/target_xyz
  grasp_task ────────> 抓取状态机         （默认不自动启动，见下）

⚠️ 安全设计：start_task 默认 false
  只起"链路"，不自动抓。因为真机一上来就动臂很危险。
  先用下面三条确认链路通，再单独跑总控：
    ros2 topic hz /camera/d435i/color/image_raw     # 相机 ≈30Hz
    ros2 topic hz /joint_states                     # 关节角在刷
    ros2 topic echo /gripper/state                  # 夹爪状态
    ros2 topic echo /grasp/target_xyz               # 感知出数（需手眼已标定）
  确认后再：
    ros2 launch arm_grasp real_grasp.launch.py start_task:=true
  或单独：
    ros2 run arm_grasp grasp_task --config <yaml> --real

夹爪网页控制：
    ros2 launch arm_grasp real_grasp.launch.py start_web:=true
    浏览器打开 http://127.0.0.1:8777

用法：
  source /opt/ros/foxy/setup.bash
  source ~/robot_vla_ws/install/local_setup.bash       # 别写进 ~/.bashrc！
  ros2 launch arm_grasp real_grasp.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _pkg_path(*parts):
    """优先用源码目录那份；包还没 colcon build 时也不会抛异常。"""
    src = os.path.join(os.path.expanduser("~/robot_vla_ws/robot_code/arm_grasp"),
                       *parts)
    if os.path.isfile(src):
        return src
    try:
        return os.path.join(get_package_share_directory("arm_grasp"), *parts)
    except Exception:                              # noqa: BLE001
        return None


def _script_path(name):
    """优先用源码目录那份（改了立刻生效，不用重新 colcon build）。"""
    return _pkg_path("scripts", name + ".py")


def _share_or_none(pkg, *parts):
    """查其它包（rm_driver 等）的 install/share 路径；没编译就返回 None。"""
    try:
        p = os.path.join(get_package_share_directory(pkg), *parts)
        return p if os.path.isfile(p) else None
    except Exception:                              # noqa: BLE001
        return None


def _resolve_params(context):
    p = LaunchConfiguration("params_file").perform(context)
    if not p:
        p = _pkg_path("config", "real_params.yaml")
        if not p:
            raise RuntimeError("找不到 real_params.yaml，用 params_file:=<路径> 显式指定")
    return p


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _setup(context, *args, **kwargs):
    import yaml
    path = _resolve_params(context)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    mode = str(cfg["profile"].get("mode", "dry-run"))
    acts = [
        LogInfo(msg="[real_grasp] 配置 %s  mode=%s" % (path, mode)),
        LogInfo(msg=("[real_grasp] ⚠️ 当前是 dry-run 演练模式，不会动硬件。"
                     if mode != "real" else
                     "[real_grasp] ⚠️ REAL 模式：确认急停按钮在手边、工作区已清空！")),
    ]

    # ---------------- 相机 ----------------
    if _flag(context, "start_camera"):
        cam_launch = _pkg_path("launch", "d435i_eco65.launch.py")
        if cam_launch:
            acts.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cam_launch),
                launch_arguments={
                    "params_file": path,
                    "enable_depth_filters": LaunchConfiguration(
                        "enable_depth_filters").perform(context),
                }.items()))
        else:
            acts.append(LogInfo(msg="[real_grasp] ⚠️ 找不到 d435i_eco65.launch.py，跳过相机"))

    # ---------------- 机械臂驱动 ----------------
    # rm_driver / rm_control 必须先编译（它们的 launch 要从 install/share 读 config）。
    # 没编译时【明确告警并跳过】，而不是让整个 launch 抛异常崩掉 ——
    # 这样即使 C++ 包暂时编不过，也能先起相机/夹爪/感知验证话题链路。
    if _flag(context, "start_arm"):
        drv = _share_or_none("rm_driver", "launch", "rm_eco65_driver.launch.py")
        if drv:
            acts.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(drv)))
        else:
            acts.append(LogInfo(msg=(
                "[real_grasp] ⚠️ rm_driver 未编译 → 跳过机械臂驱动。\n"
                "  补编：cd ~/robot_vla_ws && colcon build --packages-select "
                "rm_ros_interfaces rm_driver rm_control --symlink-install")))
        if str(cfg["arm"].get("backend", "movej")) == "action":
            ctl = _share_or_none("rm_control", "launch", "rm_eco65_control.launch.py")
            if ctl:
                acts.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ctl)))
            else:
                acts.append(LogInfo(msg="[real_grasp] ⚠️ rm_control 未编译 → action 后端不可用"))

    # ---------------- 自研节点（用 python3 直接跑，不必先编译成可执行文件）----------------
    common_args = ["--config", path]
    acts.append(ExecuteProcess(
        cmd=["python3", _script_path("gripper_node")] + common_args,
        name="gripper_node", output="screen"))
    acts.append(ExecuteProcess(
        cmd=["python3", _script_path("perception_node")] + common_args,
        name="perception_node", output="screen"))

    if _flag(context, "start_web"):
        web_args = list(common_args) + [
            "--host", LaunchConfiguration("web_host").perform(context),
            "--listen-port", LaunchConfiguration("web_port").perform(context),
        ]
        acts.append(ExecuteProcess(
            cmd=["python3", _script_path("gripper_web")] + web_args,
            name="gripper_web", output="screen"))

    if _flag(context, "start_task"):
        task_args = list(common_args)
        if mode == "real":
            task_args.append("--real")
        acts.append(ExecuteProcess(
            cmd=["python3", _script_path("grasp_task")] + task_args,
            name="grasp_task", output="screen"))
    else:
        acts.append(LogInfo(msg=(
            "[real_grasp] start_task=false：只起链路，不自动抓取。\n"
            "  确认 4 个话题都有数之后再单独跑：\n"
            "    ros2 run arm_grasp grasp_task --config " + path +
            (" --real" if mode == "real" else ""))))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="",
                              description="real_params.yaml 路径（留空自动找）"),
        DeclareLaunchArgument("start_camera", default_value="true",
                              description="是否启动 D435i 相机"),
        DeclareLaunchArgument("start_arm", default_value="true",
                              description="是否启动 rm_driver（机械臂）"),
        DeclareLaunchArgument("start_task", default_value="false",
                              description="是否自动跑抓取任务（真机建议先 false）"),
        DeclareLaunchArgument("enable_depth_filters", default_value="false",
                              description="深度空域/时域滤波（透传给相机 launch，默认关闭）"),
        DeclareLaunchArgument("start_web", default_value="false",
                              description="是否启动夹爪网页控制"),
        DeclareLaunchArgument("web_host", default_value="127.0.0.1",
                              description="网页监听地址；远程访问用 0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8777",
                              description="网页监听端口"),
        OpaqueFunction(function=_setup),
    ])

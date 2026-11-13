# -*- coding: utf-8 -*-
"""D435i 腕装（Eye-in-Hand）启动文件 —— 从 real_params.yaml 读配置，不再硬编码

和通用 realsense 启动模板的两点不同：
  ① 序列号 / 话题前缀 / 手眼结果全部从 config/real_params.yaml 读，
     改配置即可，不用改 launch 文件
  ② 相机话题前缀由 camera.prefix【自动反推】camera_namespace，
     保证 launch 与感知节点永远不会对不上（这是真机实测踩过的最容易误判的坑）

前置：必须先装好 realsense2_camera（17 号文档第 4 节）。
启动：
  ros2 launch arm_grasp d435i_eco65.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_cfg(context, params_file):
    import yaml
    p = params_file
    if not p:
        # 优先用源码目录那份（改了立刻生效，且包没 colcon build 也能用）
        src = os.path.expanduser("~/robot_vla_ws/robot_code/arm_grasp/config/real_params.yaml")
        p = src
        if not os.path.isfile(p):
            try:
                p = os.path.join(get_package_share_directory("arm_grasp"),
                                 "config", "real_params.yaml")
            except Exception:                      # noqa: BLE001
                raise RuntimeError("找不到 real_params.yaml，用 params_file:=<路径> 显式指定")
    with open(p, "r", encoding="utf-8") as f:
        return yaml.safe_load(f), p


def _split_prefix(prefix):
    """'/camera/d435i' → namespace='camera', name='d435i'；'/d435i' → '', 'd435i'"""
    s = prefix.strip("/")
    if s.startswith("camera/"):
        return "camera", s.split("/", 1)[1].split("/")[0]
    return "", s.split("/")[0]


def _detect_profile_style(rs_launch_path):
    """判断装的是哪一代 realsense-ros，返回 (style, 证据)。

    为什么必须探测：不同大版本的参数名完全不同，写错不会报错，只会被**静默忽略**，
    然后相机按默认分辨率/帧率跑 —— 现象是"能出图但深度对不上/帧率奇怪"，很难查。

      style="v4"  depth_module.depth_profile + rgb_camera.color_profile
      style="v3"  depth_module.profile       + rgb_camera.profile
      style="v2"  depth_width/height/fps     + color_width/height/fps
    """
    try:
        txt = open(rs_launch_path, "r", encoding="utf-8", errors="ignore").read()
    except Exception:                              # noqa: BLE001
        return "", ""
    if "rgb_camera.color_profile" in txt:
        return "v4", "rs_launch 里有 rgb_camera.color_profile"
    if "depth_module.profile" in txt:
        return "v3", "rs_launch 里有 depth_module.profile"
    if "depth_width" in txt or "color_width" in txt:
        return "v2", "rs_launch 里有 color_width/depth_width"
    return "", "认不出参数风格（按 v3 试）"


def _profile_args(style, cam_serial, cam, ns, name):
    """按探测到的风格组装 launch 参数。"""
    w = int(cam.get("width", 640))
    h = int(cam.get("height", 480))
    fps = int(cam.get("fps", 30))
    base = {
        "camera_namespace": ns,
        "camera_name": name,
        "serial_no": cam_serial,
        "align_depth.enable": "true",     # 像素→3D 必需
        "pointcloud.enable": "false",     # 用不到，省 USB3 带宽与 CPU
        "enable_gyro": "false",
        "enable_accel": "false",
        "enable_sync": "true",
    }
    prof = "%dx%dx%d" % (w, h, fps)
    if style == "v4":
        base["depth_module.depth_profile"] = prof
        base["rgb_camera.color_profile"] = prof
    elif style == "v2":
        base.update({"depth_width": str(w), "depth_height": str(h),
                     "depth_fps": str(fps), "color_width": str(w),
                     "color_height": str(h), "color_fps": str(fps)})
    else:                                          # v3 / 未知
        base["depth_module.profile"] = prof
        base["rgb_camera.profile"] = prof
    return base


def _launch_setup(context, *args, **kwargs):
    cfg, path = _load_cfg(context, LaunchConfiguration("params_file").perform(context))
    cam = cfg["camera"]
    ns, name = _split_prefix(cam.get("prefix", "/camera/d435i"))
    serial = str(cam.get("serial_no") or "")
    h = cfg["handeye"]

    actions = [
        LogInfo(msg="[d435i] 配置来自 %s" % path),
        LogInfo(msg="[d435i] camera_namespace='%s' camera_name='%s' → 话题 /%s%s/..."
                    % (ns, name, (ns + "/") if ns else "", name)),
    ]

    rs_launch = None
    try:
        rs_launch = os.path.join(get_package_share_directory("realsense2_camera"),
                                 "launch", "rs_launch.py")
    except Exception:                              # noqa: BLE001
        pass
    if rs_launch and os.path.isfile(rs_launch):
        style, why = _detect_profile_style(rs_launch)
        la = _profile_args(style, serial, cam, ns, name)
        actions.append(LogInfo(msg="[d435i] 参数风格=%s（%s）→ 用 %s"
                                   % (style or "未知", why,
                                      {"v4": "depth_profile/color_profile",
                                       "v2": "width/height/fps",
                                       "v3": "profile"}.get(style, "profile"))))
        # 深度时域/空域滤波：对静态桌面目标能显著压低飞点噪声。
        # ⚠️ 参数名随 realsense-ros 版本变化，默认【关闭】；现场先用
        #    ros2 param list | grep -i filter
        # 核对，再 enable_depth_filters:=true 试。不认就保持 false，不影响主链路。
        if LaunchConfiguration("enable_depth_filters").perform(context).lower() \
                in ("true", "1", "yes"):
            la["spatial_filter.enable"] = "true"
            la["temporal_filter.enable"] = "true"
            actions.append(LogInfo(msg="[d435i] 已启用深度空域/时域滤波"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments=la.items(),
        ))
    else:
        actions.append(LogInfo(msg=(
            "[d435i] ⚠️ 找不到 realsense2_camera 的 rs_launch.py（包没装或没编译）→ "
            "跳过相机节点。按 17 号文档第 4 节安装 realsense-ros 后再来。")))

    tf_xyz = h.get("tf_xyz")
    tf_q = h.get("tf_quat_xyzw")
    if tf_xyz and tf_q:
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="d435i_handeye_tf",
            arguments=[str(v) for v in list(tf_xyz) + list(tf_q)] +
                      ["flange", "d435i_link"],
            output="screen"))
    else:
        actions.append(LogInfo(msg=(
            "[d435i] ⚠️ 未配置 handeye.tf_xyz / tf_quat_xyzw → 【不发】flange→d435i_link。\n"
            "         抓取计算不需要这条 TF（感知直接用 FK·X 算），但 RViz 里看不到相机、\n"
            "         tf2_echo 也会查不到。标完定把 handeye_calib.py 输出的「推荐接法」\n"
            "         那 7 个数填进 real_params.yaml 就有了。")))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value="",
                              description="real_params.yaml 路径（留空=自动找）"),
        DeclareLaunchArgument("enable_depth_filters", default_value="false",
                              description="是否启用深度空域/时域滤波（默认关闭；"
                                          "版本参数名可能不同，先 ros2 param list 核对）"),
        OpaqueFunction(function=_launch_setup),
    ])

"""camera_sim_min.launch.py —— 相机版仿真精简版（去掉 MoveIt 依赖 panda 的问题）"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

sys.path.insert(0, os.path.join(get_package_share_directory("rm_gazebo"), "launch"))
from gz_demo_common import generate_gz_demo_launch

CAMERA_XACRO = os.path.expanduser(
    "~/robot_vla_ws/src/arm_perception/urdf/eco65_camera.urdf.xacro")

def generate_launch_description():
    return generate_gz_demo_launch(
        urdf_filename=CAMERA_XACRO,
        robot_name_in_model="rm_eco65_description",
        controller_names=["joint_state_broadcaster", "rm_group_controller",
                      "gripper_controller"],
    )

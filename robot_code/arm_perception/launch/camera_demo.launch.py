"""camera_demo.launch.py —— 带腕部相机的 ECO65 仿真（复用官方 gz_demo_common 模板）"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 借用官方的通用 Gazebo 启动模板（gzserver/spawn/控制器 全套照抄）
sys.path.insert(0, os.path.join(get_package_share_directory('rm_gazebo'), 'launch'))
from gz_demo_common import generate_gz_demo_launch

# 我们的相机版模型（绝对路径，os.path.join 遇绝对路径会直接采用）
CAMERA_XACRO = os.path.expanduser('~/robot_vla_ws/src/arm_perception/urdf/eco65_camera.urdf.xacro')

def generate_launch_description():
    # MoveIt 部分照抄官方（规划功能不受影响）
    rm_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rm_eco65_config'),
                         'launch', 'gazebo_moveit_demo.launch.py')
        )
    )
    return generate_gz_demo_launch(
        urdf_filename=CAMERA_XACRO,          # ← 唯一的改动：换成相机版模型
        robot_name_in_model='rm_eco65_description',
        controller_names=['joint_state_broadcaster', 'rm_group_controller'],
        post_spawn_actions=[rm_gazebo_moveit],
    )

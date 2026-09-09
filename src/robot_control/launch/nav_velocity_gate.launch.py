"""使用方法：ros2 launch robot_control nav_velocity_gate.launch.py 连接 Nav2 平滑速度和安全链。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_control'), 'config', 'control.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Nav2 速度门控使用的控制参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Nav2 速度门控节点的 ROS 日志级别'),
        Node(
            package='robot_control', executable='nav_controller_node',
            parameters=[LaunchConfiguration('config_file')],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

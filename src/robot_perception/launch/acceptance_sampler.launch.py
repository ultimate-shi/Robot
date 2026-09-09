"""使用方法：ros2 launch robot_perception acceptance_sampler.launch.py 保存双目感知验收样本。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_perception'),
        'config', 'acceptance_sampler.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='验收采样节点使用的语义感知参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='验收采样节点的 ROS 日志级别'),
        Node(
            package='robot_perception', executable='acceptance_sampler',
            parameters=[LaunchConfiguration('config_file')],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

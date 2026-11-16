"""使用方法：ros2 launch robot_control obstacle_avoidance.launch.py 过滤最终底盘速度。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_control'),
        'config', 'obstacle_avoidance.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='最终避障过滤器使用的控制参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'input_topic', default_value='/cmd_vel_nav',
            description='最终避障过滤器接收的上游速度指令话题'),
        DeclareLaunchArgument(
            'output_topic', default_value='/cmd_vel_safe',
            description='最终避障过滤器发布的安全速度指令话题'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='最终避障过滤节点的 ROS 日志级别'),
        Node(
            package='robot_control', executable='obstacle_avoidance',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('/cmd_vel_raw', LaunchConfiguration('input_topic')),
                ('/cmd_vel', LaunchConfiguration('output_topic')),
            ],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

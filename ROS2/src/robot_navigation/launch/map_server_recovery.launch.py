"""作用：独立启动地图生命周期自动恢复；使用方法：ros2 launch robot_navigation map_server_recovery.launch.py。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_navigation'),
                          'config', 'map_server_recovery.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config,
                              description='地图服务自动恢复的服务名和重试参数 YAML'),
        DeclareLaunchArgument('log_level', default_value='warn',
                              description='地图服务自动恢复节点日志级别'),
        Node(package='robot_navigation', executable='map_server_recovery',
             name='map_server_recovery', parameters=[LaunchConfiguration('config_file')],
             arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
             output='screen'),
    ])

"""使用方法：ros2 launch robot_navigation mapping_snapshot.launch.py 管理在线地图快照。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'mapping_snapshot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='地图快照管理器使用的建图参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='临时地图快照的保存目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='长期地图快照的根保存目录'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='地图快照管理节点的 ROS 日志级别'),
        Node(
            package='robot_navigation', executable='mapping_snapshot_manager',
            parameters=[LaunchConfiguration('config_file'), {
                'preview_directory': LaunchConfiguration('preview_directory'),
                'save_directory': LaunchConfiguration('save_directory'),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

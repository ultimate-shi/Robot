"""使用方法：ros2 launch robot_navigation rtabmap_mapping.launch.py 启动 RTAB-Map 在线建图核心。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'rtabmap_mapping.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='RTAB-Map 在线建图参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='RTAB-Map 在线建图节点的 ROS 日志级别'),
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('left/image_rect', '/stereo/left/image_rect'),
                ('right/image_rect', '/stereo/right/image_rect'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
                ('odom', '/odom'),
                ('map', '/map'),
                ('mapData', '/rtabmap/mapData'),
                ('cloud_map', '/mapping/cloud_map'),
            ],
            arguments=[
                '--delete_db_on_start', '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

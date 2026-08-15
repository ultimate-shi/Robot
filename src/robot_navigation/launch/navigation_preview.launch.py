"""使用方法：ros2 launch robot_navigation navigation_preview.launch.py 回放地图快照和虚拟底盘。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def source(package, filename):
    return PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename))


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    perception_share = get_package_share_directory('robot_perception')
    preview_config = os.path.join(nav_share, 'config', 'navigation_preview.yaml')
    terrain_config = os.path.join(perception_share, 'config', 'terrain_perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_file', default_value='/tmp/robot_preview/current.yaml',
            description='导航预演使用的二维占据栅格 YAML 快照路径'),
        DeclareLaunchArgument(
            'ply_file', default_value='/tmp/robot_preview/current.ply',
            description='导航预演使用的三维环境 PLY 快照路径'),
        IncludeLaunchDescription(source('robot_navigation', 'nav2.launch.py'),
                                 launch_arguments={'map_yaml_file': LaunchConfiguration('map_yaml_file')}.items()),
        IncludeLaunchDescription(source('robot_control', 'control.launch.py'),
                                 launch_arguments={'chassis_cmd_topic': '/cmd_vel_safe'}.items()),
        IncludeLaunchDescription(source('robot_control', 'safety.launch.py')),
        Node(package='robot_navigation', executable='publish_ply',
             parameters=[{'ply_file': LaunchConfiguration('ply_file'),
                          'allow_fallback': False}], output='screen'),
        Node(package='robot_perception', executable='snapshot_local_observer',
             parameters=[preview_config], output='screen'),
        Node(package='robot_perception', executable='virtual_ultrasonic',
             parameters=[terrain_config, {'cache_static_source': True,
                                          'source_timeout': 0.0}], output='screen'),
        Node(package='robot_navigation', executable='goal_manager',
             parameters=[preview_config], output='screen'),
    ])

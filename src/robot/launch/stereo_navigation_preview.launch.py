"""使用方法：兼容入口；转发已保存双目地图的虚拟导航预演。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    target = os.path.join(get_package_share_directory(
        'robot_navigation'), 'launch', 'navigation_preview.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_file', default_value='/tmp/robot_preview/current.yaml',
            description='导航预演使用的二维占据栅格 YAML 快照路径'),
        DeclareLaunchArgument(
            'ply_file', default_value='/tmp/robot_preview/current.ply',
            description='导航预演使用的三维环境 PLY 快照路径'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target),
            launch_arguments={
                'map_yaml_file': LaunchConfiguration('map_yaml_file'),
                'ply_file': LaunchConfiguration('ply_file'),
            }.items()),
    ])

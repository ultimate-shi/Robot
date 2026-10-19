"""使用方法：旧虚拟入口；内部仅转发到 robot_main，建议直接使用 robot_main。."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='/workspace/maps/studyroom/studyroom.yaml',
            description='兼容参数：虚拟模式二维地图 YAML 路径'),
        DeclareLaunchArgument(
            'ply_file', default_value='/workspace/maps/studyroom/studyroom.ply',
            description='兼容参数：虚拟模式三维环境 PLY 路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='兼容参数：整车日志级别'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_main'),
                'launch', 'robot.launch.py')),
            launch_arguments={
                'mode': 'virtual',
                'map_yaml_file': LaunchConfiguration('map_yaml_file'),
                'ply_file': LaunchConfiguration('ply_file'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
    ])

"""使用方法：ros2 launch robot_navigation robot.launch.py 启动工作区地图的数字孪生导航."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include(package, filename, arguments=None):
    """在独立作用域引用功能 launch，避免参数泄漏到兄弟功能。"""
    share = get_package_share_directory(package)
    return GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', filename)),
            launch_arguments=(arguments or {}).items()),
    ])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='/workspace/maps/studyroom/studyroom.yaml',
            description='数字孪生使用的工作区二维占据栅格 YAML 文件路径'),
        DeclareLaunchArgument(
            'ply_file',
            default_value='/workspace/maps/studyroom/studyroom.ply',
            description='数字孪生使用的工作区三维环境 PLY 文件路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='数字孪生组合入口内各 ROS 节点的日志级别'),
        include('robot_navigation', 'nav2.launch.py', {
            'map_yaml_file': LaunchConfiguration('map_yaml_file'),
            'log_level': LaunchConfiguration('log_level')}),
        include('robot_control', 'control.launch.py', {
            'chassis_cmd_topic': '/cmd_vel_safe',
            'log_level': LaunchConfiguration('log_level')}),
        include('robot_control', 'safety.launch.py', {
            'log_level': LaunchConfiguration('log_level')}),
        include('robot_perception', 'virtual_sensors.launch.py', {
            'log_level': LaunchConfiguration('log_level')}),
        include('robot_navigation', 'ply_map.launch.py', {
            'ply_file': LaunchConfiguration('ply_file'),
            'log_level': LaunchConfiguration('log_level')}),
    ])

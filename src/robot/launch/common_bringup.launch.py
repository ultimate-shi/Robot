"""使用方法：兼容入口；组合 robot_navigation/nav2 与 robot_control/control，不再承载实现。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include(package, filename):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename)))


def generate_launch_description():
    return LaunchDescription([
        include('robot_navigation', 'nav2.launch.py'),
        include('robot_control', 'control.launch.py'),
    ])

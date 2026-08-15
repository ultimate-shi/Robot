"""使用方法：兼容入口；ros2 launch robot robot.launch.py 转发到 robot_navigation。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    target = os.path.join(get_package_share_directory(
        'robot_navigation'), 'launch', 'robot.launch.py')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(target))])

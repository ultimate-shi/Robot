"""使用方法：ros2 launch robot_navigation robot.launch.py 启动静态数字孪生导航与安全链。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def include(package, filename, arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
        launch_arguments=(arguments or {}).items())


def generate_launch_description():
    return LaunchDescription([
        include('robot_navigation', 'nav2.launch.py'),
        include('robot_control', 'control.launch.py', {'chassis_cmd_topic': '/cmd_vel_safe'}),
        include('robot_control', 'safety.launch.py'),
        include('robot_perception', 'virtual_sensors.launch.py'),
        Node(package='robot_navigation', executable='publish_ply',
             parameters=[{'ply_file': os.path.join(
                 get_package_share_directory('robot_navigation'),
                 'map', 'studyroom.ply')}], output='screen'),
    ])

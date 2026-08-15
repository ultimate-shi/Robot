"""使用方法：ros2 launch robot_perception virtual_sensors.launch.py 启动数字孪生传感器链。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        'robot_perception'), 'config', 'terrain_perception.yaml')
    return LaunchDescription([
        Node(package='robot_perception', executable='pointcloud_obstacle_filter',
             parameters=[config], output='screen'),
        Node(package='robot_perception', executable='terrain_analyzer',
             parameters=[config], output='screen'),
        Node(package='robot_perception', executable='virtual_ultrasonic',
             parameters=[config], output='screen'),
        Node(package='robot_perception', executable='range_to_scan',
             parameters=[config], output='screen'),
        Node(package='robot_perception', executable='virtual_imu',
             parameters=[config], output='screen'),
    ])

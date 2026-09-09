"""使用方法：ros2 launch robot_perception virtual_sensors.launch.py 组合数字孪生传感器链。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        'robot_perception'), 'config', 'terrain_perception.yaml')
    share = get_package_share_directory('robot_perception')
    log_level = LaunchConfiguration('log_level')

    def include(filename, start_argument, arguments=None):
        """在独立作用域引用虚拟传感器，避免参数泄漏。"""
        launch_arguments = {
            'config_file': config,
            'log_level': log_level,
            **(arguments or {}),
        }
        return GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    share, 'launch', filename)),
                launch_arguments=launch_arguments.items(),
                condition=IfCondition(LaunchConfiguration(start_argument))),
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='数字孪生传感器链各节点的 ROS 日志级别'),
        DeclareLaunchArgument(
            'start_pointcloud_obstacle', default_value='true',
            description='是否启动环境点云障碍提取'),
        DeclareLaunchArgument(
            'start_terrain_analysis', default_value='true',
            description='是否启动地形分析'),
        DeclareLaunchArgument(
            'start_virtual_ultrasonic', default_value='true',
            description='是否启动虚拟超声波'),
        DeclareLaunchArgument(
            'start_range_to_scan', default_value='true',
            description='是否启动超声波距离到 LaserScan 转换'),
        DeclareLaunchArgument(
            'start_virtual_imu', default_value='true',
            description='是否启动数字孪生 IMU'),
        include(
            'pointcloud_obstacle.launch.py', 'start_pointcloud_obstacle'),
        include('terrain_analysis.launch.py', 'start_terrain_analysis'),
        include('virtual_ultrasonic.launch.py', 'start_virtual_ultrasonic'),
        include('range_to_scan.launch.py', 'start_range_to_scan'),
        include('virtual_imu.launch.py', 'start_virtual_imu'),
    ])

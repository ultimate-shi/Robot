"""使用方法：ros2 launch robot_perception imu.launch.py 启动真实 GY95T 与无磁姿态滤波。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_perception'), 'config', 'imu.yaml')
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument(
            'device', default_value='/dev/gy95t',
            description='GY95T USB 转串口稳定设备路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='GY95T 驱动和姿态滤波节点的 ROS 日志级别'),
        Node(
            package='robot_perception', executable='gy95t_driver',
            parameters=[config, {'device': LaunchConfiguration('device')}],
            arguments=['--ros-args', '--log-level', log_level], output='screen'),
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick', parameters=[config],
            remappings=[
                ('imu/data_raw', '/sensors/imu/data_raw'),
                ('imu/data', '/sensors/imu/data'),
            ],
            arguments=['--ros-args', '--log-level', log_level], output='screen'),
    ])

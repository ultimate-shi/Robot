"""使用方法：ros2 launch robot_perception imu.launch.py 启动 GY95T 与无磁滤波；厂家姿态需显式开启。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument(
            'vendor_orientation_enabled', default_value='false',
            description='是否在 /sensors/imu/vendor_data 声明厂家磁航向有效；需先验证校准、安装轴和地图参考'),
        Node(
            package='robot_perception', executable='gy95t_driver',
            parameters=[config, {
                'device': LaunchConfiguration('device'),
                'vendor_orientation_enabled': ParameterValue(
                    LaunchConfiguration('vendor_orientation_enabled'), value_type=bool),
            }],
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

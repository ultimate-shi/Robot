"""使用方法：ros2 launch robot_perception virtual_ultrasonic.launch.py 从环境数据模拟超声波。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_perception'),
        'config', 'terrain_perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='虚拟超声波参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'cache_static_source', default_value='false',
            description='是否缓存一次静态环境输入并持续用于超声波模拟'),
        DeclareLaunchArgument(
            'source_timeout', default_value='2.0',
            description='动态环境输入的超时时间，0 表示不检查超时，单位为秒'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='虚拟超声波节点的 ROS 日志级别'),
        Node(
            package='robot_perception', executable='virtual_ultrasonic',
            parameters=[LaunchConfiguration('config_file'), {
                'cache_static_source': ParameterValue(
                    LaunchConfiguration('cache_static_source'),
                    value_type=bool),
                'source_timeout': ParameterValue(
                    LaunchConfiguration('source_timeout'), value_type=float),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

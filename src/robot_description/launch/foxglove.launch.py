"""使用方法：ros2 launch robot_description foxglove.launch.py 启动唯一的 Foxglove Bridge。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='8765',
            description='Foxglove Bridge 监听的 WebSocket 端口'),
        DeclareLaunchArgument(
            'address', default_value='0.0.0.0',
            description='Foxglove Bridge 监听的网络地址'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Foxglove Bridge 的 ROS 日志级别'),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': ParameterValue(
                    LaunchConfiguration('port'), value_type=int),
                'address': LaunchConfiguration('address'),
                'asset_uri_allowlist': ['package://robot_description/.*'],
                'allow_file_transfer': True,
                'send_buffer_limit': 10000000,
                'max_packet_messages': 100,
                'client_timeout_ms': 300000,
                'keep_alive_interval_ms': 5000,
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

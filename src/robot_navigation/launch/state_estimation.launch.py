"""使用方法：ros2 launch robot_navigation state_estimation.launch.py 融合里程计并发布统一 /odom。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'state_estimation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='robot_localization EKF 参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'output_topic', default_value='/odom',
            description='融合后统一里程计输出话题'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='二维 EKF 状态估计节点的 ROS 日志级别'),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node',
            parameters=[LaunchConfiguration('config_file')],
            remappings=[
                ('odometry/filtered', LaunchConfiguration('output_topic')),
            ],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

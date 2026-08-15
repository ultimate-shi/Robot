"""使用方法：ros2 launch robot_description description.launch.py 发布 robot_description 与 TF。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(share, 'urdf', 'robot.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    description = {'robot_description': ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)}
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟 /clock'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='机器人状态与 TF 发布节点的 ROS 日志级别'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[description, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
        ),
    ])

"""使用方法：ros2 launch robot_control safety.launch.py 启动 Nav2 速度门控和最终安全过滤。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        'robot_control'), 'config', 'control.yaml')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic', default_value='/cmd_vel_nav',
            description='安全过滤器接收的上游速度指令话题'),
        DeclareLaunchArgument(
            'output_topic', default_value='/cmd_vel_safe',
            description='安全过滤后发布的底盘速度指令话题'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='速度门控与最终安全过滤节点的 ROS 日志级别'),
        Node(package='robot_control', executable='nav_controller_node',
             parameters=[config], arguments=ros_args, output='screen'),
        Node(package='robot_control', executable='obstacle_avoidance',
             parameters=[config],
             remappings=[('/cmd_vel_raw', input_topic), ('/cmd_vel', output_topic)],
             arguments=ros_args, output='screen'),
    ])

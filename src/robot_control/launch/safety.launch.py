"""使用方法：ros2 launch robot_control safety.launch.py 组合速度门控和最终安全过滤。"""

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


def scoped_include(launch_source, **kwargs):
    """隔离子 launch 参数，避免通用参数名污染兄弟功能。"""
    return GroupAction(actions=[
        IncludeLaunchDescription(launch_source, **kwargs),
    ])


def generate_launch_description():
    share = get_package_share_directory('robot_control')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    log_level = LaunchConfiguration('log_level')
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
        DeclareLaunchArgument(
            'start_velocity_gate', default_value='true',
            description='是否启动 Nav2 速度门控节点'),
        DeclareLaunchArgument(
            'start_obstacle_avoidance', default_value='true',
            description='是否启动最终障碍速度过滤节点'),
        scoped_include(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'nav_velocity_gate.launch.py')),
            launch_arguments={
                'log_level': log_level,
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'start_velocity_gate'))),
        scoped_include(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'obstacle_avoidance.launch.py')),
            launch_arguments={
                'input_topic': input_topic,
                'output_topic': output_topic,
                'log_level': log_level,
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'start_obstacle_avoidance'))),
    ])

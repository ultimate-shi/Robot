"""使用方法：ros2 launch robot_control control.launch.py 组合模型、控制器和底盘控制链。"""

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


def source(package, filename):
    """返回指定功能包内独立 launch 的描述源。"""
    return PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename))


def scoped_include(launch_source, **kwargs):
    """隔离子 launch 参数，避免通用参数名污染兄弟功能。"""
    return GroupAction(actions=[
        IncludeLaunchDescription(launch_source, **kwargs),
    ])


def generate_launch_description():
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument(
            'chassis_cmd_topic', default_value='/cmd_vel_safe',
            description='底盘控制器订阅的速度指令话题'),
        DeclareLaunchArgument(
            'publish_odometry', default_value='true',
            description='底盘控制器是否自行发布 /odom 和对应 TF'),
        DeclareLaunchArgument(
            'start_description', default_value='true',
            description='是否同时启动机器人模型和 TF 发布'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='底盘控制组合入口内各 ROS 节点的日志级别'),
        scoped_include(
            source('robot_description', 'description.launch.py'),
            launch_arguments={
                'use_sim_time': 'false',
                'log_level': log_level,
            }.items(),
            condition=IfCondition(
                LaunchConfiguration('start_description'))),
        scoped_include(
            source('robot_control', 'controllers.launch.py'),
            launch_arguments={'log_level': log_level}.items()),
        scoped_include(
            source('robot_control', 'chassis_control.launch.py'),
            launch_arguments={
                'chassis_cmd_topic': LaunchConfiguration(
                    'chassis_cmd_topic'),
                'publish_odometry': LaunchConfiguration('publish_odometry'),
                'log_level': log_level,
            }.items()),
    ])

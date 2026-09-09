"""使用方法：ros2 launch robot_perception stereo_perception.launch.py 组合双目障碍和语义感知。"""

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
    share = get_package_share_directory('robot_perception')
    semantic = os.path.join(share, 'config', 'semantic_perception.yaml')
    inference_url = LaunchConfiguration('inference_url')
    log_level = LaunchConfiguration('log_level')
    return LaunchDescription([
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='语义感知节点调用的本地推理服务基础 URL'),
        DeclareLaunchArgument(
            'detection_mode', default_value='on_demand',
            description='YOLO 检测模式：on_demand 按需识别，continuous 按限频持续识别'),
        DeclareLaunchArgument(
            'start_pointcloud_filter', default_value='true',
            description='是否启动实时双目障碍点云过滤；组合入口已有同类节点时关闭'),
        DeclareLaunchArgument(
            'start_semantic_detection', default_value='true',
            description='是否启动语义目标识别与定位节点'),
        DeclareLaunchArgument(
            'start_acceptance_sampler', default_value='true',
            description='是否启动感知验收样本保存节点'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='双目障碍与语义感知节点的 ROS 日志级别'),
        scoped_include(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'stereo_pointcloud_filter.launch.py')),
            launch_arguments={'log_level': log_level}.items(),
            condition=IfCondition(LaunchConfiguration(
                'start_pointcloud_filter'))),
        scoped_include(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'semantic_detection.launch.py')),
            launch_arguments={
                'config_file': semantic,
                'inference_url': inference_url,
                'detection_mode': LaunchConfiguration('detection_mode'),
                'log_level': log_level,
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'start_semantic_detection'))),
        scoped_include(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'acceptance_sampler.launch.py')),
            launch_arguments={
                'config_file': semantic,
                'log_level': log_level,
            }.items(),
            condition=IfCondition(LaunchConfiguration(
                'start_acceptance_sampler'))),
    ])

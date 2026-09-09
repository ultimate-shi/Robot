"""使用方法：ros2 launch robot_navigation stereo_mapping.launch.py 组合真实双目在线建图链。"""

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
from launch.substitutions import LaunchConfiguration, PythonExpression


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
    start_localization = LaunchConfiguration('start_localization')
    use_imu = LaunchConfiguration('use_imu')
    use_wheel_odometry = LaunchConfiguration('use_wheel_odometry')
    localization_with_imu = IfCondition(PythonExpression([
        '"', start_localization, '" == "true" and "',
        use_imu, '" == "true"',
    ]))
    localization_with_wheel = IfCondition(PythonExpression([
        '"', start_localization, '" == "true" and "',
        use_wheel_odometry, '" == "true"',
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'navigation_processing_enabled', default_value='true',
            description='建图时是否生成视差、深度和完整双目点云'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='是否启动真实 GY95T 并用于视觉里程计和 EKF'),
        DeclareLaunchArgument(
            'wait_imu_to_init', default_value='false',
            description='是否强制视觉里程计收到 IMU 姿态后才启动'),
        DeclareLaunchArgument(
            'imu_device', default_value='/dev/gy95t',
            description='GY95T USB 转串口稳定设备路径'),
        DeclareLaunchArgument(
            'use_wheel_odometry', default_value='false',
            description='是否启动并融合真实四轮反馈里程计'),
        DeclareLaunchArgument(
            'require_head_feedback', default_value='false',
            description='建图时是否必须收到头部实际零位反馈'),
        DeclareLaunchArgument(
            'obstacle_pointcloud_topic',
            default_value='/mapping/stereo_obstacle_points',
            description='过滤后的实时双目障碍点云输出话题'),
        DeclareLaunchArgument(
            'start_description', default_value='true',
            description='是否启动机器人模型和 robot_state_publisher'),
        DeclareLaunchArgument(
            'start_joint_state_publisher', default_value='true',
            description='是否为没有实车反馈的模型发布默认关节状态'),
        DeclareLaunchArgument(
            'start_foxglove_bridge', default_value='true',
            description='是否启动本入口唯一的 Foxglove Bridge'),
        DeclareLaunchArgument(
            'start_localization', default_value='true',
            description='是否启动 IMU、视觉里程计、EKF 和可选轮式里程计'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='false',
            description='是否显式发布深度预览 compressed 图像'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='临时地图快照的保存目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='长期地图快照的根保存目录'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='Foxglove Bridge 监听的 WebSocket 端口'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='双目建图组合入口内各 ROS 节点的日志级别'),
        scoped_include(
            source('robot_description', 'description.launch.py'),
            launch_arguments={'log_level': log_level}.items(),
            condition=IfCondition(
                LaunchConfiguration('start_description'))),
        scoped_include(
            source('robot_description', 'joint_states.launch.py'),
            launch_arguments={'log_level': log_level}.items(),
            condition=IfCondition(
                LaunchConfiguration('start_joint_state_publisher'))),
        scoped_include(
            source('robot_control', 'head_mapping_lock.launch.py'),
            launch_arguments={
                'require_feedback': LaunchConfiguration(
                    'require_head_feedback'),
                'log_level': log_level,
            }.items()),
        scoped_include(
            source('robot_perception', 'stereo_camera.launch.py'),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'navigation_processing_enabled': LaunchConfiguration(
                    'navigation_processing_enabled'),
                'publish_compressed': LaunchConfiguration(
                    'publish_compressed'),
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
                'start_foxglove_bridge': 'false',
                'log_level': log_level,
            }.items()),
        scoped_include(
            source('robot_perception', 'imu.launch.py'),
            launch_arguments={
                'device': LaunchConfiguration('imu_device'),
                'log_level': log_level,
            }.items(),
            condition=localization_with_imu),
        scoped_include(
            source('robot_navigation', 'stereo_odometry.launch.py'),
            launch_arguments={
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'log_level': log_level,
            }.items(),
            condition=IfCondition(start_localization)),
        scoped_include(
            source('robot_navigation', 'state_estimation.launch.py'),
            launch_arguments={'log_level': log_level}.items(),
            condition=IfCondition(start_localization)),
        scoped_include(
            source('robot_control', 'wheel_odometry.launch.py'),
            launch_arguments={'log_level': log_level}.items(),
            condition=localization_with_wheel),
        scoped_include(
            source('robot_perception', 'stereo_pointcloud_filter.launch.py'),
            launch_arguments={
                'output_topic': LaunchConfiguration(
                    'obstacle_pointcloud_topic'),
                'log_level': log_level,
            }.items()),
        scoped_include(
            source('robot_navigation', 'rtabmap_mapping.launch.py'),
            launch_arguments={'log_level': log_level}.items()),
        scoped_include(
            source('robot_navigation', 'mapping_snapshot.launch.py'),
            launch_arguments={
                'preview_directory': LaunchConfiguration('preview_directory'),
                'save_directory': LaunchConfiguration('save_directory'),
                'log_level': log_level,
            }.items()),
        scoped_include(
            source('robot_description', 'foxglove.launch.py'),
            launch_arguments={
                'port': LaunchConfiguration('foxglove_port'),
                'log_level': log_level,
            }.items(),
            condition=IfCondition(
                LaunchConfiguration('start_foxglove_bridge'))),
    ])

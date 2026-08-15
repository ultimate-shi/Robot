"""使用方法：ros2 launch robot_navigation stereo_robot.launch.py 启动真实双目与静态 Nav2 链。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def source(package, filename):
    return PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='完整在线双目机器人各 ROS 节点的日志级别'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        IncludeLaunchDescription(source('robot_description', 'description.launch.py'),
                                 launch_arguments={
                                     'log_level': LaunchConfiguration('log_level')}.items()),
        IncludeLaunchDescription(source('robot_navigation', 'nav2.launch.py'),
                                 launch_arguments={
                                     'log_level': LaunchConfiguration('log_level')}.items()),
        IncludeLaunchDescription(source('robot_control', 'control.launch.py'),
                                 launch_arguments={
                                     'chassis_cmd_topic': '/cmd_vel_safe',
                                     'start_description': 'false',
                                     'log_level': LaunchConfiguration('log_level')}.items()),
        IncludeLaunchDescription(source('robot_control', 'safety.launch.py'),
                                 launch_arguments={
                                     'log_level': LaunchConfiguration('log_level')}.items()),
        IncludeLaunchDescription(source('robot_perception', 'stereo_camera.launch.py'),
                                 launch_arguments={
                                     'video_device': LaunchConfiguration('video_device'),
                                     'apply_auto_camera_controls': LaunchConfiguration(
                                         'apply_auto_camera_controls'),
                                     'log_level': LaunchConfiguration('log_level')}.items()),
        IncludeLaunchDescription(source('robot_perception', 'stereo_perception.launch.py'),
                                 launch_arguments={
                                     'log_level': LaunchConfiguration('log_level')}.items()),
    ])

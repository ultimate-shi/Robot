"""使用方法：兼容入口；转发双目建图、路径预演和本地大脑，始终不驱动底盘。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    target = os.path.join(get_package_share_directory(
        'robot_brain'), 'launch', 'stereo_brain.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='机器人网页与 ROS Bridge 监听的 HTTP 端口'),
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='本地视觉语言模型推理服务的基础 URL'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'http_port': LaunchConfiguration('http_port'),
                'inference_url': LaunchConfiguration('inference_url'),
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
            }.items()),
    ])

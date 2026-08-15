"""使用方法：ros2 launch robot_brain brain.launch.py 启动 HTTP 网页、Qwen客户端和 ROS Bridge。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        'robot_brain'), 'config', 'brain.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='机器人网页与 ROS Bridge 监听的 HTTP 端口'),
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='本地视觉语言模型推理服务的基础 URL'),
        Node(package='robot_brain', executable='brain_web', name='brain_ros_bridge',
             parameters=[config, {
                 'http_port': LaunchConfiguration('http_port'),
                 'inference_url': LaunchConfiguration('inference_url'),
                 'motion_enabled': False,
             }], output='screen'),
    ])

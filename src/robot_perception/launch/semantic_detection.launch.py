"""使用方法：ros2 launch robot_perception semantic_detection.launch.py 启动语义目标识别与定位。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_perception'),
        'config', 'semantic_perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='语义识别和目标定位参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='语义识别节点调用的本地推理服务基础 URL'),
        DeclareLaunchArgument(
            'detection_mode', default_value='on_demand',
            description='YOLO 检测模式，可选 on_demand 或 continuous'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='语义识别与目标定位节点的 ROS 日志级别'),
        Node(
            package='robot_perception', executable='semantic_perception',
            parameters=[LaunchConfiguration('config_file'), {
                'inference_url': LaunchConfiguration('inference_url'),
                'detection_mode': LaunchConfiguration('detection_mode'),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

"""作用：从可配置彩色/对齐深度进行语义定位；用法：semantic_detection.launch.py，整车按相机来源覆盖话题。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_perception'),
        'config', 'semantic_detection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/stereo/right/image_rect', description='已校正彩色图像话题'),
        DeclareLaunchArgument('depth_topic', default_value='/stereo/depth/right/image', description='已对齐彩色图的深度话题'),
        DeclareLaunchArgument('camera_info_topic', default_value='/stereo/right/camera_info', description='彩色校正图的内参话题'),
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
            'start_segmentation', default_value='true',
            description='是否持续运行 SegFormer 场景分割'),
        DeclareLaunchArgument(
            'enable_semantic_navigation', default_value='true',
            description='是否把分割与深度融合为 Nav2 独立语义点云'),
        DeclareLaunchArgument(
            'segmentation_rate', default_value='5.0',
            description='SegFormer 正常目标频率，过载时自动降至配置中的 1Hz'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='语义识别与目标定位节点的 ROS 日志级别'),
        Node(
            package='robot_perception', executable='semantic_perception',
            parameters=[LaunchConfiguration('config_file'), {
                'inference_url': LaunchConfiguration('inference_url'),
                'image_topic': LaunchConfiguration('image_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'detection_mode': LaunchConfiguration('detection_mode'),
                'start_segmentation': LaunchConfiguration('start_segmentation'),
                'enable_semantic_navigation': LaunchConfiguration(
                    'enable_semantic_navigation'),
                'segmentation_rate': LaunchConfiguration('segmentation_rate'),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

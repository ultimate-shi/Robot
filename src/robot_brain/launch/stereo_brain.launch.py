"""使用方法：ros2 launch robot_brain stereo_brain.launch.py 启动双目建图、规划预演和本地大脑。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    brain_share = get_package_share_directory('robot_brain')
    nav_share = get_package_share_directory('robot_navigation')
    perception_share = get_package_share_directory('robot_perception')
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'http_port', default_value='8080',
            description='机器人网页与 ROS Bridge 监听的 HTTP 端口'),
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='本地 YOLO 与纯文本 Qwen 推理网关的基础 URL'),
        DeclareLaunchArgument(
            'detection_mode', default_value='continuous',
            description='网页 YOLO 检测模式，默认 continuous 持续识别'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='本地大脑整套 ROS 节点及网页服务的日志级别'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_share, 'launch', 'stereo_mapping.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'navigation_processing_enabled': 'true',
                'publish_compressed': 'true',
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                perception_share, 'launch', 'stereo_perception.launch.py')),
            launch_arguments={
                'inference_url': LaunchConfiguration('inference_url'),
                'detection_mode': LaunchConfiguration('detection_mode'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_share, 'launch', 'mission_preview.launch.py')),
            launch_arguments={
                'idle_detection_mode': LaunchConfiguration('detection_mode'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                brain_share, 'launch', 'brain.launch.py')),
            launch_arguments={
                'http_port': LaunchConfiguration('http_port'),
                'inference_url': LaunchConfiguration('inference_url'),
                'log_level': LaunchConfiguration('log_level'),
            }.items()),
    ])

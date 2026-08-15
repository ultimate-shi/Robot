"""使用方法：兼容入口；ros2 launch robot stereo_mapping.launch.py 转发到 robot_navigation。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    target = os.path.join(get_package_share_directory(
        'robot_navigation'), 'launch', 'stereo_mapping.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='临时导航预演地图快照的保存目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='长期地图快照的根保存目录'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'preview_directory': LaunchConfiguration('preview_directory'),
                'save_directory': LaunchConfiguration('save_directory'),
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
            }.items()),
    ])

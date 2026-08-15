"""使用方法：兼容入口；ros2 launch robot stereo_camera.launch.py 转发到 robot_perception。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    target = os.path.join(get_package_share_directory(
        'robot_perception'), 'launch', 'stereo_camera.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'calibration_mode', default_value='false',
            description='是否进入双目标定模式并跳过校正、深度等处理'),
        DeclareLaunchArgument(
            'splitter_backend', default_value='cpp',
            description='双目拼接图拆分后端，可选 cpp 或 python'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(target),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'calibration_mode': LaunchConfiguration('calibration_mode'),
                'splitter_backend': LaunchConfiguration('splitter_backend'),
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
            }.items()),
    ])

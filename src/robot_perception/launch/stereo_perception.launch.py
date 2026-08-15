"""使用方法：ros2 launch robot_perception stereo_perception.launch.py 启动双目障碍和语义感知。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_perception')
    pointcloud = os.path.join(share, 'config', 'stereo_pointcloud.yaml')
    semantic = os.path.join(share, 'config', 'semantic_perception.yaml')
    inference_url = LaunchConfiguration('inference_url')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]
    return LaunchDescription([
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='语义感知节点调用的本地推理服务基础 URL'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='双目障碍与语义感知节点的 ROS 日志级别'),
        Node(package='robot_perception', executable='stereo_pointcloud_filter',
             parameters=[pointcloud, {'use_sim_time': False}],
             arguments=ros_args, output='screen'),
        Node(package='robot_perception', executable='semantic_perception',
             parameters=[semantic, {'inference_url': inference_url}],
             arguments=ros_args, output='screen'),
        Node(package='robot_perception', executable='acceptance_sampler',
             parameters=[semantic], arguments=ros_args, output='screen'),
    ])

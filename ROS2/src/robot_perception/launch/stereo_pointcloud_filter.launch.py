"""使用方法：ros2 launch robot_perception stereo_pointcloud_filter.launch.py 过滤双目障碍点云。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_perception'),
        'config', 'stereo_pointcloud_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='双目障碍点云过滤参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'input_topic', default_value='/stereo/points2',
            description='待过滤的完整双目 PointCloud2 输入话题'),
        DeclareLaunchArgument(
            'output_topic', default_value='/nav/stereo_obstacle_points',
            description='过滤后的障碍 PointCloud2 输出话题'),
        DeclareLaunchArgument(
            'node_name', default_value='stereo_pointcloud_filter',
            description='点云过滤节点名称，用于并行输出不同用途点云时区分实例'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟 /clock'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='双目障碍点云过滤节点的 ROS 日志级别'),
        Node(
            package='robot_perception', executable='stereo_pointcloud_filter',
            name=LaunchConfiguration('node_name'),
            parameters=[LaunchConfiguration('config_file'), {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

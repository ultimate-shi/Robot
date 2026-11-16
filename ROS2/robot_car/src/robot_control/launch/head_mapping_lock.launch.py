"""使用方法：ros2 launch robot_control head_mapping_lock.launch.py 在建图期间保持头部归中。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_control'),
        'config', 'head_mapping_lock.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='头部建图归中节点使用的控制参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'require_feedback', default_value='false',
            description='是否必须收到头部实际角度反馈后才允许建图'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='头部建图归中节点的 ROS 日志级别'),
        Node(
            package='robot_control', executable='head_mapping_lock_node',
            name='head_mapping_lock',
            parameters=[LaunchConfiguration('config_file'), {
                'require_feedback': ParameterValue(
                    LaunchConfiguration('require_feedback'), value_type=bool),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

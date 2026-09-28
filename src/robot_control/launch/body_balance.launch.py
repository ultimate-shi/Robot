"""使用方法：ros2 launch robot_control body_balance.launch.py 启动车身平衡节点."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_control'),
        'config', 'body_balance.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=config,
            description='姿态平衡、限幅和过流保护参数 YAML 路径'),
        DeclareLaunchArgument(
            'wheelbase', default_value='0.312',
            description='前后轮轴距，单位米'),
        DeclareLaunchArgument(
            'track', default_value='0.280',
            description='左右轮距，单位米'),
        DeclareLaunchArgument(
            'leg_length', default_value='0.060',
            description='主动腿长度，单位米'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='车身平衡节点日志级别'),
        Node(
            package='robot_control', executable='body_balance_node',
            name='body_balance', parameters=[LaunchConfiguration('config_file'), {
                'wheelbase': ParameterValue(
                    LaunchConfiguration('wheelbase'), value_type=float),
                'track': ParameterValue(
                    LaunchConfiguration('track'), value_type=float),
                'leg_length': ParameterValue(
                    LaunchConfiguration('leg_length'), value_type=float),
            }],
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
            output='screen'),
    ])

"""使用方法：ros2 launch robot_control wheel_odometry.launch.py 发布四轮反馈里程计."""

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
        'config', 'wheel_odometry.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='四轮里程计使用的底盘控制参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'enabled', default_value='true',
            description='是否启用四轮反馈里程计计算和发布'),
        DeclareLaunchArgument(
            'wheelbase', default_value='0.312',
            description='前后轮轴距，单位米'),
        DeclareLaunchArgument(
            'track', default_value='0.280',
            description='左右轮距，单位米'),
        DeclareLaunchArgument(
            'wheel_radius', default_value='0.055',
            description='车轮有效滚动半径，单位米'),
        DeclareLaunchArgument(
            'leg_length', default_value='0.060',
            description='主动腿长度，单位米'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='四轮反馈里程计节点的 ROS 日志级别'),
        Node(
            package='robot_control', executable='four_wheel_odometry_node',
            name='four_wheel_odometry',
            parameters=[LaunchConfiguration('config_file'), {
                'enabled': ParameterValue(
                    LaunchConfiguration('enabled'), value_type=bool),
                'wheelbase': ParameterValue(
                    LaunchConfiguration('wheelbase'), value_type=float),
                'track': ParameterValue(
                    LaunchConfiguration('track'), value_type=float),
                'wheel_radius': ParameterValue(
                    LaunchConfiguration('wheel_radius'), value_type=float),
                'leg_length': ParameterValue(
                    LaunchConfiguration('leg_length'), value_type=float),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

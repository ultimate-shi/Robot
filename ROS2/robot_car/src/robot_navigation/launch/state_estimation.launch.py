"""使用方法：ros2 launch robot_navigation state_estimation.launch.py 融合里程计；无三维传感源时用 two_d_mode_override:=true。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'state_estimation.yaml')
    covariance_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'odometry_covariance_guard.yaml')

    def start_ekf(context):
        """独立入口尊重参数文件；统一整车入口可显式覆盖二维开关。"""
        override = LaunchConfiguration('two_d_mode_override').perform(context).lower()
        parameters = [LaunchConfiguration('config_file').perform(context)]
        if override in ('true', 'false'):
            parameters.append({'two_d_mode': override == 'true'})
        elif override != 'auto':
            raise ValueError('two_d_mode_override 只允许 auto、true 或 false')
        return [Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', parameters=parameters,
            remappings=[
                ('odometry/filtered', LaunchConfiguration('output_topic')),
            ],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level').perform(context),
            ],
            output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'covariance_config_file', default_value=covariance_config,
            description='视觉里程计协方差下限参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='robot_localization EKF 参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'two_d_mode_override', default_value='auto',
            description='auto 尊重参数文件；true/false 覆盖二维 EKF 开关，虚拟模式设 true'),
        DeclareLaunchArgument(
            'output_topic', default_value='/odom',
            description='融合后统一里程计输出话题'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='二维 EKF 状态估计节点的 ROS 日志级别'),
        Node(
            package='robot_navigation',
            executable='odometry_covariance_guard_node',
            name='odometry_covariance_guard',
            parameters=[LaunchConfiguration('covariance_config_file')],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
        OpaqueFunction(function=start_ekf),
    ])

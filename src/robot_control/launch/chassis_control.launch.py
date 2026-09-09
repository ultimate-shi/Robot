"""使用方法：ros2 launch robot_control chassis_control.launch.py 启动底盘反馈和运动控制节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_control'), 'config', 'control.yaml')
    log_level = LaunchConfiguration('log_level')

    def staggered(index, node):
        """在当前作用域解析间隔，再创建不会依赖子作用域的定时器。"""
        def create_timer(context):
            interval = float(LaunchConfiguration(
                'node_start_interval').perform(context))
            return [TimerAction(period=interval * index, actions=[node])]

        return OpaqueFunction(function=create_timer)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='底盘反馈和运动控制参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'chassis_cmd_topic', default_value='/cmd_vel_safe',
            description='底盘控制器订阅的速度指令话题'),
        DeclareLaunchArgument(
            'publish_odometry', default_value='true',
            description='底盘控制器是否自行发布 /odom 和对应 TF'),
        DeclareLaunchArgument(
            'node_start_interval', default_value='0.0',
            description='底盘反馈和控制节点的错峰启动间隔，单位为秒'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='底盘反馈和运动控制节点的 ROS 日志级别'),
        staggered(0, Node(
            package='robot_control', executable='chassis_feedback_node',
            arguments=['--ros-args', '--log-level', log_level],
            output='screen')),
        staggered(1, Node(
            package='robot_control', executable='chassis_controller_node',
            name='chassis_controller',
            parameters=[LaunchConfiguration('config_file'), {
                'publish_odometry': ParameterValue(
                    LaunchConfiguration('publish_odometry'), value_type=bool),
            }],
            remappings=[
                ('/cmd_vel', LaunchConfiguration('chassis_cmd_topic')),
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen')),
    ])

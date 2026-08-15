"""使用方法：ros2 launch robot_control control.launch.py 启动模拟 ros2_control 与四轮底盘。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_share = get_package_share_directory('robot_control')
    description_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(description_share, 'urdf', 'robot.xacro')
    manager_config = os.path.join(control_share, 'config', 'controller_manager.yaml')
    controller_config = os.path.join(control_share, 'config', 'controllers.yaml')
    control_config = os.path.join(control_share, 'config', 'control.yaml')
    chassis_cmd_topic = LaunchConfiguration('chassis_cmd_topic')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]
    description = {'robot_description': ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)}
    controller_names = [
        'joint_state_broadcaster', 'steering_controller', 'wheel_controller',
        'lap_fr_position_controller', 'lap_fl_position_controller',
        'lap_rr_position_controller', 'lap_rl_position_controller',
        'shin_fr_position_controller', 'shin_fl_position_controller',
        'shin_rr_position_controller', 'shin_rl_position_controller',
    ]
    actions = [
        DeclareLaunchArgument(
            'chassis_cmd_topic', default_value='/cmd_vel_safe',
            description='底盘控制器订阅的速度指令话题'),
        DeclareLaunchArgument(
            'start_description', default_value='true',
            description='是否同时启动机器人模型和 TF 发布'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='底盘控制与控制器管理节点的 ROS 日志级别'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_share, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('start_description')),
        ),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[description, manager_config, {'use_sim_time': False}],
             arguments=ros_args, output='screen'),
        Node(package='robot_control', executable='chassis_feedback_node',
             arguments=ros_args, output='screen'),
        Node(package='robot_control', executable='chassis_controller_node', name='chassis_controller',
             parameters=[control_config], remappings=[('/cmd_vel', chassis_cmd_topic)],
             arguments=ros_args, output='screen'),
    ]
    for index, name in enumerate(controller_names):
        actions.append(TimerAction(period=2.0 + 1.2 * index, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=[name, '--param-file', controller_config,
                            '--ros-args', '--log-level', log_level], output='screen')]))
    return LaunchDescription(actions)

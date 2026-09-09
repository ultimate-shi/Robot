"""使用方法：ros2 launch robot_control controllers.launch.py 启动 ros2_control 和全部关节控制器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_share = get_package_share_directory('robot_control')
    description_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(description_share, 'urdf', 'robot.xacro')
    default_controller_config = os.path.join(
        control_share, 'config', 'controllers.yaml')
    log_level = LaunchConfiguration('log_level')
    robot_description = {'robot_description': ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)}
    controller_names = [
        'joint_state_broadcaster', 'steering_controller', 'wheel_controller',
        'head_controller',
        'lap_fr_position_controller', 'lap_fl_position_controller',
        'lap_rr_position_controller', 'lap_rl_position_controller',
        'shin_fr_position_controller', 'shin_fl_position_controller',
        'shin_rr_position_controller', 'shin_rl_position_controller',
    ]
    actions = [
        DeclareLaunchArgument(
            'manager_config_file', default_value=default_controller_config,
            description='controller_manager 参数 YAML 文件路径；默认与控制器共用同名配置'),
        DeclareLaunchArgument(
            'controller_config_file', default_value=default_controller_config,
            description='各关节控制器参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'spawn_start_delay', default_value='2.0',
            description='第一个控制器生成器的启动延迟，单位为秒'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='ros2_control 和控制器生成器的 ROS 日志级别'),
        Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[
                robot_description,
                LaunchConfiguration('manager_config_file'),
                {'use_sim_time': False},
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen'),
    ]
    def create_deferred_spawners(context):
        """提前解析定时器和退出回调参数，保证被复杂入口延时引用时仍可用。"""
        controller_config = LaunchConfiguration(
            'controller_config_file').perform(context)
        resolved_log_level = log_level.perform(context)
        spawn_start_delay = float(LaunchConfiguration(
            'spawn_start_delay').perform(context))
        spawners = [
            Node(
                package='controller_manager', executable='spawner',
                arguments=[
                    name, '--param-file', controller_config,
                    '--ros-args', '--log-level', resolved_log_level,
                ],
                output='screen')
            for name in controller_names
        ]
        deferred_actions = [
            RegisterEventHandler(OnProcessExit(
                target_action=current,
                on_exit=[following],
            ))
            for current, following in zip(spawners, spawners[1:])
        ]
        deferred_actions.append(TimerAction(
            period=spawn_start_delay,
            actions=[spawners[0]],
        ))
        return deferred_actions

    actions.append(OpaqueFunction(function=create_deferred_spawners))
    return LaunchDescription(actions)

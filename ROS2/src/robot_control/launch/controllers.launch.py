"""作用：编排 ros2_control 和关节控制器并处理安全退出；使用方法：ros2 launch robot_control controllers.launch.py。"""

import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def bootstrap_exit_actions(event, context, name, next_spawner):
    """使用方法：控制器生成器退出时调用；关闭中不再启动下一个节点或重复关闭 launch。"""
    if context.is_shutdown or event.returncode in (-signal.SIGINT, -signal.SIGTERM):
        return []
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(
            reason=f'{name} 控制器未能激活，整车启动已停止'))]
    return [next_spawner] if next_spawner is not None else []


def generate_launch_description():
    control_share = get_package_share_directory('robot_control')
    description_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(description_share, 'urdf', 'robot.xacro')
    default_controller_config = os.path.join(
        control_share, 'config', 'controllers.yaml')
    default_servo_calibration = os.path.join(
        get_package_share_directory('robot_hardware'),
        'config', 'servo_calibration.yaml')
    default_wheel_calibration = os.path.join(
        get_package_share_directory('robot_hardware'),
        'config', 'wheel_calibration.yaml')
    log_level = LaunchConfiguration('log_level')
    hardware_arguments = [
        'device_candidates', 'serial_timeout_ms', 'inter_command_delay_ms',
        'discovery_retry_count', 'discovery_retry_delay_ms', 'encoder_query_delay_ms',
        'command_timeout_ms', 'servo_keepalive_period_ms',
        'servo_command_timeout_ms', 'wheel_diameter', 'sonar_lpf_alpha',
        'sonar_min_valid_mm', 'sonar_max_valid_mm']
    robot_description = {'robot_description': ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware_plugin:=', LaunchConfiguration('hardware_plugin'),
            ' servo_calibration_file:=',
            LaunchConfiguration('servo_calibration_file'),
            ' wheel_calibration_file:=',
            LaunchConfiguration('wheel_calibration_file'),
        ] + [item for name in hardware_arguments
             for item in (' ' + name + ':=', LaunchConfiguration(name))]),
        value_type=str)}
    controller_names = [
        'joint_state_broadcaster', 'steering_controller', 'wheel_controller',
        'head_controller',
        'lap_fr_position_controller', 'lap_fl_position_controller',
        'lap_rr_position_controller', 'lap_rl_position_controller',
    ]
    actions = [
        DeclareLaunchArgument(
            'hardware_plugin', default_value='mock_components/GenericSystem',
            description='ros2_control SystemInterface 插件名称'),
        DeclareLaunchArgument(
            'servo_calibration_file', default_value=default_servo_calibration,
            description='实机六路舵机标定参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'wheel_calibration_file', default_value=default_wheel_calibration,
            description='实机四轮轴映射、方向和 D 模式曲线 YAML 文件路径'),
        DeclareLaunchArgument(
            'device_candidates', default_value='/dev/ttyACM0,/dev/ttyACM1,/dev/ttyACM2',
            description='候选串口设备，逗号分隔；插件按身份响应匹配 DMC0/DMC1/SE2'),
        DeclareLaunchArgument(
            'serial_timeout_ms', default_value='500',
            description='单次串口事务超时，单位毫秒'),
        DeclareLaunchArgument(
            'inter_command_delay_ms', default_value='3',
            description='同一串口相邻协议指令间隔，单位毫秒'),
        DeclareLaunchArgument(
            'discovery_retry_count', default_value='20',
            description='控制板身份发现的最大轮数'),
        DeclareLaunchArgument(
            'discovery_retry_delay_ms', default_value='700',
            description='控制板重枚举后再次发现前的等待时间，单位毫秒'),
        DeclareLaunchArgument(
            'encoder_query_delay_ms', default_value='20',
            description='DMC1 相邻编码器参数查询间隔，单位毫秒'),
        DeclareLaunchArgument(
            'command_timeout_ms', default_value='500',
            description='ROS 轮速命令心跳超时，单位毫秒'),
        DeclareLaunchArgument(
            'servo_keepalive_period_ms', default_value='40',
            description='SE2 舵机 U 指令独立刷新周期，单位毫秒'),
        DeclareLaunchArgument(
            'servo_command_timeout_ms', default_value='400',
            description='舵机 PWM 缓存超时后改发中性值，单位毫秒'),
        DeclareLaunchArgument(
            'wheel_diameter', default_value='0.110',
            description='硬件协议换算使用的车轮直径，单位米'),
        DeclareLaunchArgument(
            'sonar_lpf_alpha', default_value='0.2',
            description='超声波低通滤波新样本权重'),
        DeclareLaunchArgument(
            'sonar_min_valid_mm', default_value='30',
            description='SE2 超声波最小有效距离，单位毫米'),
        DeclareLaunchArgument(
            'sonar_max_valid_mm', default_value='2500',
            description='SE2 超声波最大有效距离，单位毫米'),
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
            'controller_service_timeout', default_value='8.0',
            description='单次控制器管理服务响应超时，单位为秒'),
        DeclareLaunchArgument(
            'controller_max_attempts', default_value='12',
            description='每个控制器的最大状态检查和重试次数'),
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
        """提前解析定时器和退出回调参数，保证被复杂入口延时引用时仍可用."""
        controller_config = LaunchConfiguration(
            'controller_config_file').perform(context)
        resolved_log_level = log_level.perform(context)
        spawn_start_delay = float(LaunchConfiguration(
            'spawn_start_delay').perform(context))
        service_timeout = LaunchConfiguration(
            'controller_service_timeout').perform(context)
        max_attempts = LaunchConfiguration(
            'controller_max_attempts').perform(context)
        spawners = [
            Node(
                package='robot_control', executable='controller_bootstrap',
                arguments=[
                    name, '--param-file', controller_config,
                    '--service-timeout', service_timeout,
                    '--max-attempts', max_attempts,
                    '--ros-args', '--log-level', resolved_log_level,
                ],
                output='screen')
            for name in controller_names
        ]
        deferred_actions = []
        for index, current in enumerate(spawners):
            following = spawners[index + 1] if index + 1 < len(spawners) else None

            def after_exit(event, context, name=controller_names[index],
                           next_spawner=following):
                return bootstrap_exit_actions(event, context, name, next_spawner)

            deferred_actions.append(RegisterEventHandler(OnProcessExit(
                target_action=current, on_exit=after_exit)))
        deferred_actions.append(TimerAction(
            period=spawn_start_delay,
            actions=[spawners[0]],
        ))
        return deferred_actions

    actions.append(OpaqueFunction(function=create_deferred_spawners))
    return LaunchDescription(actions)

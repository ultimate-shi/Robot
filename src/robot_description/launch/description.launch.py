"""使用方法：ros2 launch robot_description description.launch.py 发布 robot_description 与 TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(share, 'urdf', 'robot.xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    hardware_plugin = LaunchConfiguration('hardware_plugin')
    servo_calibration_file = LaunchConfiguration('servo_calibration_file')
    hardware_arguments = [
        'device_candidates', 'serial_timeout_ms', 'inter_command_delay_ms',
        'discovery_retry_count', 'discovery_retry_delay_ms', 'encoder_query_delay_ms',
        'command_timeout_ms', 'wheel_diameter', 'sonar_lpf_alpha',
        'sonar_min_valid_mm', 'sonar_max_valid_mm']
    description = {'robot_description': ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware_plugin:=', hardware_plugin,
            ' servo_calibration_file:=', servo_calibration_file,
        ] + [item for name in hardware_arguments
             for item in (' ' + name + ':=', LaunchConfiguration(name))]),
        value_type=str)}
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_plugin', default_value='mock_components/GenericSystem',
            description='ros2_control 硬件插件；实机由 robot_main 传入串口插件'),
        DeclareLaunchArgument(
            'servo_calibration_file', default_value='',
            description='实机舵机标定 YAML；虚拟模式可留空'),
        DeclareLaunchArgument(
            'device_candidates', default_value='/dev/ttyACM0,/dev/ttyACM1,/dev/ttyACM2',
            description='候选串口设备，逗号分隔；插件按 $info 响应识别控制板'),
        DeclareLaunchArgument(
            'serial_timeout_ms', default_value='500',
            description='单次串口事务超时，单位毫秒'),
        DeclareLaunchArgument(
            'inter_command_delay_ms', default_value='3',
            description='同一串口相邻协议指令的最小间隔，单位毫秒'),
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
            'wheel_diameter', default_value='0.110',
            description='实机车轮直径，单位米'),
        DeclareLaunchArgument(
            'sonar_lpf_alpha', default_value='0.2',
            description='超声波一阶低通滤波新样本权重'),
        DeclareLaunchArgument(
            'sonar_min_valid_mm', default_value='30',
            description='SE2 超声波最小有效距离，单位毫米'),
        DeclareLaunchArgument(
            'sonar_max_valid_mm', default_value='2500',
            description='SE2 超声波最大有效距离，单位毫米'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟 /clock'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='机器人状态与 TF 发布节点的 ROS 日志级别'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[description, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            output='screen',
        ),
    ])

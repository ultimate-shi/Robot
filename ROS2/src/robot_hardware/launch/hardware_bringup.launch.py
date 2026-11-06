"""使用方法：通过 robot_hardware/hardware_bringup.launch.py 单独启动真实底盘."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


def include(package, filename, arguments):
    """在独立作用域包含子 launch，避免参数泄漏."""
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items())])


def generate_launch_description():
    hardware_share = get_package_share_directory('robot_hardware')
    calibration = os.path.join(hardware_share, 'config', 'servo_calibration.yaml')
    with open(os.path.join(
            hardware_share, 'config', 'hardware_bringup.yaml'),
            encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)['hardware']
    controllers = os.path.join(
        get_package_share_directory('robot_control'),
        'config', 'controllers.yaml')
    hardware_arguments = {
        name: LaunchConfiguration(name) for name in (
            'device_candidates', 'serial_timeout_ms',
            'inter_command_delay_ms', 'discovery_retry_count',
            'discovery_retry_delay_ms', 'encoder_query_delay_ms', 'command_timeout_ms',
            'servo_keepalive_period_ms', 'servo_command_timeout_ms',
                'leg_io_rate', 'wheel_io_rate', 'servo_feedback_rate', 'feedback_timeout_ms', 'servo_watchdog_ms', 'telemetry_rate',
            'wheel_diameter', 'sonar_lpf_alpha',
            'sonar_min_valid_mm', 'sonar_max_valid_mm')}
    return LaunchDescription([
        DeclareLaunchArgument(
            'servo_calibration_file', default_value=calibration,
            description='六路舵机零位、方向和 PID 标定 YAML 路径'),
        DeclareLaunchArgument(
            'wheel_calibration_file',
            default_value=os.path.join(hardware_share, 'config', 'wheel_calibration.yaml'),
            description='四轮轴映射、方向和 D 模式曲线 YAML 路径'),
        DeclareLaunchArgument(
            'controller_config_file', default_value=controllers,
            description='底盘 ros2_control 控制器配置 YAML 路径'),
        DeclareLaunchArgument(
            'device_candidates',
            default_value=','.join(defaults['device_candidates']),
            description='候选串口设备，逗号分隔并按身份自动匹配'),
        DeclareLaunchArgument(
            'serial_timeout_ms', default_value=str(defaults['serial_timeout_ms']),
            description='单次串口事务超时，单位毫秒'),
        DeclareLaunchArgument(
            'inter_command_delay_ms',
            default_value=str(defaults['inter_command_delay_ms']),
            description='同一串口相邻协议指令间隔，单位毫秒'),
        DeclareLaunchArgument(
            'discovery_retry_count',
            default_value=str(defaults['discovery_retry_count']),
            description='控制板身份发现的最大轮数'),
        DeclareLaunchArgument(
            'discovery_retry_delay_ms',
            default_value=str(defaults['discovery_retry_delay_ms']),
            description='控制板重枚举后再次发现前的等待时间，单位毫秒'),
        DeclareLaunchArgument(
            'encoder_query_delay_ms',
            default_value=str(defaults['encoder_query_delay_ms']),
            description='DMC1 相邻编码器参数查询间隔，单位毫秒'),
        DeclareLaunchArgument(
            'command_timeout_ms',
            default_value=str(defaults['ros_command_timeout_ms']),
            description='ROS 轮速命令心跳超时，单位毫秒'),
        DeclareLaunchArgument(
            'servo_watchdog_ms', default_value=str(defaults['servo_watchdog_ms']),
            description='SE2 固件现有 U 刷新看门狗期限，单位毫秒；仅用于监测，不修改固件'),
        DeclareLaunchArgument(
            'telemetry_rate', default_value=str(defaults['telemetry_rate']),
            description='硬件状态与诊断独立发布频率，单位 Hz；不占控制管理器读写周期'),
        DeclareLaunchArgument(
            'leg_io_rate', default_value=str(defaults['leg_io_rate']),
            description='DMC0 腿板独立读写频率，单位 Hz；20 Hz 需先验收'),
        DeclareLaunchArgument(
            'wheel_io_rate', default_value=str(defaults['wheel_io_rate']),
            description='DMC1 轮板独立读写频率，单位 Hz'),
        DeclareLaunchArgument(
            'servo_feedback_rate', default_value=str(defaults['servo_feedback_rate']),
            description='SE2 舵机与超声波反馈查询频率，单位 Hz'),
        DeclareLaunchArgument(
            'feedback_timeout_ms', default_value=str(defaults['feedback_timeout_ms']),
            description='控制板真实反馈失效期限，单位毫秒'),
        DeclareLaunchArgument(
            'servo_keepalive_period_ms',
            default_value=str(defaults['servo_keepalive_period_ms']),
            description='SE2 舵机 U 指令独立刷新周期，单位毫秒'),
        DeclareLaunchArgument(
            'servo_command_timeout_ms',
            default_value=str(defaults['servo_command_timeout_ms']),
            description='舵机 PWM 缓存超时后改发中性值，单位毫秒'),
        DeclareLaunchArgument(
            'wheel_diameter', default_value=str(defaults['wheel_diameter']),
            description='硬件协议换算使用的车轮直径，单位米'),
        DeclareLaunchArgument(
            'sonar_lpf_alpha', default_value=str(defaults['sonar_lpf_alpha']),
            description='超声波低通滤波新样本权重'),
        DeclareLaunchArgument(
            'sonar_min_valid_mm', default_value=str(defaults['sonar_min_valid_mm']),
            description='SE2 超声波最小有效距离，单位毫米；更小值按振铃或无效回波处理'),
        DeclareLaunchArgument(
            'sonar_max_valid_mm', default_value=str(defaults['sonar_max_valid_mm']),
            description='SE2 超声波最大有效距离，单位毫米；更大值按无效回波处理'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='硬件管理器和控制器日志级别'),
        include('robot_description', 'description.launch.py', {
            'hardware_plugin': 'robot_hardware/RobotSerialSystem',
            'servo_calibration_file': LaunchConfiguration(
                'servo_calibration_file'),
            'wheel_calibration_file': LaunchConfiguration(
                'wheel_calibration_file'),
            'log_level': LaunchConfiguration('log_level'),
            **hardware_arguments,
        }),
        include('robot_control', 'controllers.launch.py', {
            'hardware_plugin': 'robot_hardware/RobotSerialSystem',
            'servo_calibration_file': LaunchConfiguration(
                'servo_calibration_file'),
            'wheel_calibration_file': LaunchConfiguration(
                'wheel_calibration_file'),
            'manager_config_file': LaunchConfiguration(
                'controller_config_file'),
            'controller_config_file': LaunchConfiguration(
                'controller_config_file'),
            'log_level': LaunchConfiguration('log_level'),
            **hardware_arguments,
        }),
    ])

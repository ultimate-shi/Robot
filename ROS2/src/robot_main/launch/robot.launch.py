"""作用：编排实机或仿真整车与网页手动/导航速度仲裁；使用方法：ros2 launch robot_main robot.launch.py mode:=real|virtual。"""

import os
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml

from robot_main.map_validation import validate_map_yaml
from robot_navigation.mapping.database import validate_bundle


def source(package, filename):
    return PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename))


def include(package, filename, arguments=None):
    """隔离子 launch 参数，避免同名配置泄漏到其他功能."""
    return GroupAction(actions=[IncludeLaunchDescription(
        source(package, filename),
        launch_arguments=(arguments or {}).items())])


def delayed(delay, action):
    return TimerAction(period=float(delay), actions=[action])


def generate_launch_description():
    main_share = get_package_share_directory('robot_main')
    control_share = get_package_share_directory('robot_control')
    hardware_share = get_package_share_directory('robot_hardware')
    nav_share = get_package_share_directory('robot_navigation')
    perception_share = get_package_share_directory('robot_perception')
    with open(os.path.join(main_share, 'config', 'robot.yaml'), encoding='utf-8') as stream:
        defaults = yaml.safe_load(stream)['defaults']
    with open(os.path.join(
            hardware_share, 'config', 'hardware_bringup.yaml'),
            encoding='utf-8') as stream:
        hardware_defaults = yaml.safe_load(stream)['hardware']

    declarations = [
        DeclareLaunchArgument(
            'mode', default_value='virtual',
            description='运行模式，只允许 real 或 virtual'),
        DeclareLaunchArgument(
            'map_yaml_file', default_value='',
            description='已有地图 YAML；留空时实机启动在线 RTAB-Map 建图与导航'),
        DeclareLaunchArgument(
            'database_path', default_value='',
            description='已有地图配套数据库；留空自动使用地图 YAML 同名 .db，实机重定位必需'),
        DeclareLaunchArgument(
            'localization_config_file', default_value=os.path.join(nav_share, 'config', 'map_localization.yaml'),
            description='已有地图视觉定位与匹配确认阈值 YAML'),
        DeclareLaunchArgument(
            'ply_file', default_value=str(defaults['virtual_ply']),
            description='虚拟模式环境三维 PLY 文件路径'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='实机双目相机设备路径'),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=os.path.join(
                perception_share, 'config', 'stereo_camera.yaml'),
            description='双目相机参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'imu_device', default_value='/dev/gy95t',
            description='实机 GY95T 串口设备路径'),
        DeclareLaunchArgument(
            'use_imu', default_value=str(defaults['use_imu']).lower(),
            description='实机模式是否启动并融合 GY95T'),
        DeclareLaunchArgument(
            'wait_imu_to_init', default_value='false',
            description='视觉里程计是否等待 IMU 后再初始化'),
        DeclareLaunchArgument(
            'require_head_feedback', default_value='true',
            description='在线建图前是否必须确认头部实际归中'),
        DeclareLaunchArgument(
            'use_wheel_odometry',
            default_value=str(defaults['use_wheel_odometry']).lower(),
            description='是否发布并融合四轮反馈里程计'),
        DeclareLaunchArgument(
            'initial_pose_enabled', default_value='false',
            description='实机已有地图是否使用明确提供的启动位置估计；仍需视觉确认，默认不假定当前在原点'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='启动地图 X，单位米；实机启用 initial_pose_enabled 时作为初始估计，虚拟模式用于固定变换'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='启动地图 Y，单位米；实机启用 initial_pose_enabled 时作为初始估计，虚拟模式用于固定变换'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='启动地图朝向，单位弧度；实机启用 initial_pose_enabled 时作为初始估计'),
        DeclareLaunchArgument(
            'initial_yaw_source', default_value='manual',
            description='已有地图的初始航向来源：manual 使用 initial_yaw；magnetic 仅允许实机且须提供 magnetic_east_yaw_in_map'),
        DeclareLaunchArgument(
            'magnetic_east_yaw_in_map', default_value='',
            description='磁 ENU 东轴在 map 坐标中的偏航角，单位弧度；选择 magnetic 时必须显式设置'),
        DeclareLaunchArgument(
            'controller_manager_config_file',
            default_value=os.path.join(control_share, 'config', 'controllers.yaml'),
            description='controller_manager 参数 YAML 路径'),
        DeclareLaunchArgument(
            'controller_config_file',
            default_value=os.path.join(control_share, 'config', 'controllers.yaml'),
            description='统一 ros2_control 控制器参数 YAML 路径'),
        DeclareLaunchArgument(
            'controller_service_timeout',
            default_value=str(defaults['controller_service_timeout']),
            description='单次控制器管理服务响应超时，单位为秒'),
        DeclareLaunchArgument(
            'controller_max_attempts',
            default_value=str(defaults['controller_max_attempts']),
            description='每个关节控制器启动时的最大状态检查和重试次数'),
        DeclareLaunchArgument(
            'chassis_control_config_file',
            default_value=os.path.join(control_share, 'config', 'chassis_control.yaml'),
            description='底盘运动学与轮速超时保护参数 YAML'),
        DeclareLaunchArgument(
            'web_console_config_file',
            default_value=os.path.join(
                get_package_share_directory('robot_web'),
                'config', 'web_console.yaml'),
            description='网页控制台的速度仲裁、租约与 ROS 数据源参数 YAML'),
        DeclareLaunchArgument(
            'servo_calibration_file',
            default_value=os.path.join(
                hardware_share, 'config', 'servo_calibration.yaml'),
            description='实机六路舵机标定 YAML 路径'),
        DeclareLaunchArgument(
            'wheel_calibration_file',
            default_value=os.path.join(
                hardware_share, 'config', 'wheel_calibration.yaml'),
            description='实机四轮轴映射、方向和 D 模式曲线 YAML 路径'),
        DeclareLaunchArgument(
            'device_candidates',
            default_value=','.join(hardware_defaults['device_candidates']),
            description='实机候选串口设备，逗号分隔并按控制板身份自动匹配'),
        DeclareLaunchArgument(
            'serial_timeout_ms',
            default_value=str(hardware_defaults['serial_timeout_ms']),
            description='实机单次串口事务超时，单位毫秒'),
        DeclareLaunchArgument(
            'inter_command_delay_ms',
            default_value=str(hardware_defaults['inter_command_delay_ms']),
            description='实机同一串口相邻协议指令间隔，单位毫秒'),
        DeclareLaunchArgument(
            'discovery_retry_count',
            default_value=str(hardware_defaults['discovery_retry_count']),
            description='实机控制板身份发现的最大轮数'),
        DeclareLaunchArgument(
            'discovery_retry_delay_ms',
            default_value=str(hardware_defaults['discovery_retry_delay_ms']),
            description='控制板重枚举后再次发现前的等待时间，单位毫秒'),
        DeclareLaunchArgument(
            'encoder_query_delay_ms',
            default_value=str(hardware_defaults['encoder_query_delay_ms']),
            description='DMC1 相邻编码器参数查询间隔，单位毫秒'),
        DeclareLaunchArgument(
            'command_timeout_ms',
            default_value=str(hardware_defaults['ros_command_timeout_ms']),
            description='实机 ROS 轮速命令心跳超时，单位毫秒'),
        DeclareLaunchArgument(
            'servo_keepalive_period_ms',
            default_value=str(hardware_defaults['servo_keepalive_period_ms']),
            description='实机 SE2 舵机 U 指令独立刷新周期，单位毫秒'),
        DeclareLaunchArgument(
            'servo_command_timeout_ms',
            default_value=str(hardware_defaults['servo_command_timeout_ms']),
            description='实机舵机 PWM 缓存超时后改发中性值，单位毫秒'),
        DeclareLaunchArgument(
            'wheel_diameter',
            default_value=str(hardware_defaults['wheel_diameter']),
            description='模型、控制和硬件协议共用的车轮直径，单位米'),
        DeclareLaunchArgument(
            'wheelbase', default_value=str(hardware_defaults['wheelbase']),
            description='运动学、里程计和平衡共用的前后轮轴距，单位米'),
        DeclareLaunchArgument(
            'track', default_value=str(hardware_defaults['track']),
            description='运动学、里程计和平衡共用的左右轮距，单位米'),
        DeclareLaunchArgument(
            'leg_length', default_value=str(hardware_defaults['leg_length']),
            description='里程计和平衡共用的主动腿长度，单位米'),
        DeclareLaunchArgument(
            'sonar_lpf_alpha',
            default_value=str(hardware_defaults['sonar_lpf_alpha']),
            description='实机超声波低通滤波新样本权重'),
        DeclareLaunchArgument(
            'sonar_min_valid_mm',
            default_value=str(hardware_defaults['sonar_min_valid_mm']),
            description='实机超声波最小有效距离，单位毫米'),
        DeclareLaunchArgument(
            'sonar_max_valid_mm',
            default_value=str(hardware_defaults['sonar_max_valid_mm']),
            description='实机超声波最大有效距离，单位毫米'),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(
                perception_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'left.yaml'),
            description='左目相机标定 YAML 路径'),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(
                perception_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'right.yaml'),
            description='右目相机标定 YAML 路径'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='false',
            description='是否发布额外的压缩深度预览'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启相机自动曝光和白平衡'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2.yaml'),
            description='Nav2 基础参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'nav2_overrides_file',
            default_value=os.path.join(nav_share, 'config', 'stereo_robot.yaml'),
            description='实机双目障碍层 Nav2 参数覆盖 YAML 路径'),
        DeclareLaunchArgument(
            'start_semantic_perception',
            default_value=str(defaults['start_semantic_perception']).lower(),
            description='实机模式是否启动语义识别和分割'),
        DeclareLaunchArgument(
            'detection_mode', default_value='continuous',
            description='YOLO 检测模式'),
        DeclareLaunchArgument(
            'start_segmentation', default_value='true',
            description='是否启动 SegFormer 室内语义分割'),
        DeclareLaunchArgument(
            'enable_semantic_navigation', default_value='true',
            description='是否把语义分割结果送入 Nav2'),
        DeclareLaunchArgument(
            'segmentation_rate', default_value='5.0',
            description='语义分割目标频率，单位 Hz'),
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='本地 NPU 推理网关基础 URL'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='在线建图临时快照目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='在线建图长期快照根目录'),
        DeclareLaunchArgument(
            'foxglove_port', default_value=str(defaults['foxglove_port']),
            description='Foxglove Bridge WebSocket 端口'),
        DeclareLaunchArgument(
            'foxglove_log_level', default_value='warn',
            description='Foxglove Bridge 日志级别'),
        DeclareLaunchArgument(
            'node_start_interval', default_value=str(defaults['node_start_interval']),
            description='同阶段相邻功能错峰启动间隔，单位秒'),
        DeclareLaunchArgument(
            'sensor_start_delay', default_value=str(defaults['sensor_start_delay']),
            description='实机传感器与建图链启动延迟，单位秒'),
        DeclareLaunchArgument(
            'control_start_delay', default_value=str(defaults['control_start_delay']),
            description='底盘控制链启动延迟，单位秒'),
        DeclareLaunchArgument(
            'navigation_start_delay',
            default_value=str(defaults['navigation_start_delay']),
            description='Nav2 启动延迟，单位秒'),
        DeclareLaunchArgument(
            'perception_start_delay',
            default_value=str(defaults['perception_start_delay']),
            description='语义感知启动延迟，单位秒'),
        DeclareLaunchArgument(
            'log_level', default_value=str(defaults['log_level']),
            description='整车各 ROS 节点日志级别'),
        DeclareLaunchArgument(
            'start_logging', default_value='false',
            description='是否随整车启动 MCAP 日常采集与异常录制管理器'),
        DeclareLaunchArgument(
            'logging_config_file', default_value='',
            description='日志参数 YAML 路径；留空使用 robot_logging 的默认配置'),
    ]

    def compose(context):
        mode = LaunchConfiguration('mode').perform(context).strip().lower()
        if mode not in ('real', 'virtual'):
            raise RuntimeError("mode 只允许 'real' 或 'virtual'")
        map_yaml = LaunchConfiguration('map_yaml_file').perform(context).strip()
        if map_yaml:
            map_yaml = validate_map_yaml(map_yaml)
        localization_required = mode == 'real' and bool(map_yaml)
        initial_enabled = LaunchConfiguration('initial_pose_enabled').perform(context).lower()
        if initial_enabled not in ('true', 'false'):
            raise RuntimeError('initial_pose_enabled 只能为 true 或 false')
        if initial_enabled == 'true' and not localization_required:
            raise RuntimeError('启动视觉初始估计仅用于实机已有地图模式')
        if initial_enabled == 'true' and not all(math.isfinite(float(
                LaunchConfiguration(key).perform(context))) for key in ('initial_x', 'initial_y', 'initial_yaw')):
            raise RuntimeError('启动初始估计必须为有限数值')
        database_path = ''
        if localization_required:
            database_path = validate_bundle(map_yaml,
                LaunchConfiguration('database_path').perform(context).strip() or
                str(Path(map_yaml).with_suffix('.db')))
        if mode == 'virtual' and not map_yaml:
            raise RuntimeError(
                'virtual 模式没有实时双目建图源，请显式传入 map_yaml_file；'
                '实机在线建图请使用 mode:=real')
        initial_yaw_source = LaunchConfiguration(
            'initial_yaw_source').perform(context).strip().lower()
        if initial_yaw_source not in ('manual', 'magnetic'):
            raise RuntimeError("initial_yaw_source 只允许 'manual' 或 'magnetic'")
        if initial_yaw_source == 'magnetic':
            if localization_required:
                raise RuntimeError('已有地图实机改用视觉持续定位；请使用 initial_yaw_source:=manual，并在网页设置初始位置和朝向')
            if mode != 'real' or not map_yaml:
                raise RuntimeError(
                    'magnetic 初始航向只允许实机加载已有地图时使用')
            if not LaunchConfiguration(
                    'magnetic_east_yaw_in_map').perform(context).strip():
                raise RuntimeError(
                    'magnetic 初始航向必须显式提供 magnetic_east_yaw_in_map')
            if LaunchConfiguration('use_imu').perform(context).strip().lower() != 'true':
                raise RuntimeError('magnetic 初始航向必须启用 use_imu:=true')
        log_level = LaunchConfiguration('log_level').perform(context)
        control_delay = float(LaunchConfiguration('control_start_delay').perform(context))
        sensor_delay = float(LaunchConfiguration('sensor_start_delay').perform(context))
        navigation_delay = float(
            LaunchConfiguration('navigation_start_delay').perform(context))
        perception_delay = float(
            LaunchConfiguration('perception_start_delay').perform(context))
        plugin = ('robot_hardware/RobotSerialSystem' if mode == 'real'
                  else 'mock_components/GenericSystem')
        hardware_arguments = {
            name: LaunchConfiguration(name) for name in (
                'device_candidates', 'serial_timeout_ms',
                'inter_command_delay_ms', 'discovery_retry_count',
                'discovery_retry_delay_ms', 'encoder_query_delay_ms', 'command_timeout_ms',
                'servo_keepalive_period_ms', 'servo_command_timeout_ms',
                'wheel_diameter', 'sonar_lpf_alpha',
                'sonar_min_valid_mm', 'sonar_max_valid_mm')}
        geometry_arguments = {
            'wheelbase': LaunchConfiguration('wheelbase'),
            'track': LaunchConfiguration('track'),
            'wheel_radius': str(
                float(LaunchConfiguration('wheel_diameter').perform(context)) / 2.0),
        }
        nav_overrides = (LaunchConfiguration('nav2_overrides_file')
                         if mode == 'real' else
                         LaunchConfiguration('nav2_params_file'))
        common = [
            LogInfo(msg=(
                f'robot_main 已选择 mode={mode}，ros2_control 插件={plugin}')),
            include('robot_web', 'web_console.launch.py', {
                'config_file': LaunchConfiguration('web_console_config_file'),
                'mode': mode,
                'static_map': 'true' if map_yaml else 'false',
                'log_level': log_level}),
            include('robot_description', 'description.launch.py', {
                'hardware_plugin': plugin,
                'servo_calibration_file': LaunchConfiguration(
                    'servo_calibration_file'),
                'wheel_calibration_file': LaunchConfiguration(
                    'wheel_calibration_file'),
                'log_level': log_level,
                **hardware_arguments}),
            include('robot_description', 'foxglove.launch.py', {
                'port': LaunchConfiguration('foxglove_port'),
                'log_level': LaunchConfiguration('foxglove_log_level')}),
            delayed(control_delay, include('robot_control', 'controllers.launch.py', {
                'hardware_plugin': plugin,
                'servo_calibration_file': LaunchConfiguration(
                    'servo_calibration_file'),
                'wheel_calibration_file': LaunchConfiguration(
                    'wheel_calibration_file'),
                'manager_config_file': LaunchConfiguration(
                    'controller_manager_config_file'),
                'controller_config_file': LaunchConfiguration(
                    'controller_config_file'),
                'controller_service_timeout': LaunchConfiguration(
                    'controller_service_timeout'),
                'controller_max_attempts': LaunchConfiguration(
                    'controller_max_attempts'),
                'log_level': log_level,
                **hardware_arguments})),
            delayed(control_delay + 0.8, include(
                'robot_control', 'chassis_control.launch.py', {
                    'config_file': (LaunchConfiguration('chassis_control_config_file')
                                    if mode == 'real' else
                                    os.path.join(control_share, 'config', 'chassis_control.yaml')),
                    'chassis_cmd_topic': '/cmd_vel_safe',
                    'publish_odometry': 'false',
                    'node_start_interval': LaunchConfiguration(
                        'node_start_interval'),
                    'log_level': log_level,
                    **geometry_arguments})),
            delayed(control_delay + 1.6, include(
                'robot_control', 'wheel_motion.launch.py', {
                    'require_encoder_service': 'true' if mode == 'real' else 'false',
                    'require_console_arm': 'true',
                    'log_level': log_level})),
            delayed(control_delay + 2.4, include(
                'robot_control', 'wheel_odometry.launch.py', {
                    'enabled': LaunchConfiguration('use_wheel_odometry'),
                    'leg_length': LaunchConfiguration('leg_length'),
                    'log_level': log_level,
                    **geometry_arguments})),
            delayed(control_delay + 3.2, include(
                'robot_control', 'safety.launch.py', {
                    'input_topic': '/cmd_vel_console',
                    'output_topic': '/cmd_vel_nav_safe',
                    'require_localization': 'true' if localization_required else 'false',
                    'log_level': log_level})),
            include('robot_navigation', 'state_estimation.launch.py', {
                'two_d_mode_override': 'true' if mode == 'virtual' else 'false',
                'log_level': log_level}),
            delayed(navigation_delay, include('robot_navigation', 'nav2.launch.py', {
                'map_yaml_file': map_yaml,
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'nav2_overrides_file': nav_overrides,
                'use_map_server': 'true' if map_yaml else 'false',
                'publish_map_to_odom': 'false' if localization_required else 'true',
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'initial_yaw_source': LaunchConfiguration('initial_yaw_source'),
                'magnetic_east_yaw_in_map': LaunchConfiguration(
                    'magnetic_east_yaw_in_map'),
                'node_start_interval': LaunchConfiguration(
                    'node_start_interval'),
                'log_level': log_level})),
        ]
        if LaunchConfiguration('start_logging').perform(context).lower() == 'true':
            logging_share = get_package_share_directory('robot_logging')
            logging_config = LaunchConfiguration('logging_config_file').perform(context)
            common.insert(1, include('robot_logging', 'logging.launch.py', {
                'mode': mode,
                'config_file': logging_config or os.path.join(
                    logging_share, 'config', 'logging.yaml')}))
        if mode == 'virtual':
            common.extend([
                include('robot_navigation', 'ply_map.launch.py', {
                    'ply_file': LaunchConfiguration('ply_file'),
                    'log_level': log_level}),
                include('robot_perception', 'virtual_sensors.launch.py', {
                    'start_virtual_imu': 'false',
                    'log_level': log_level}),
            ])
            return common

        if LaunchConfiguration('use_imu').perform(context).lower() == 'true':
            common.append(include('robot_perception', 'imu.launch.py', {
                'device': LaunchConfiguration('imu_device'),
                'vendor_orientation_enabled': (
                    'true' if initial_yaw_source == 'magnetic' else 'false'),
                'log_level': log_level}))
        common.extend([
            delayed(control_delay + 2.0, include(
                'robot_control', 'body_balance.launch.py', {
                    'wheelbase': LaunchConfiguration('wheelbase'),
                    'track': LaunchConfiguration('track'),
                    'leg_length': LaunchConfiguration('leg_length'),
                    'log_level': log_level})),
            include('robot_navigation', 'stereo_odometry.launch.py', {
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'log_level': log_level}),
            delayed(sensor_delay, include('robot_perception', 'stereo_camera.launch.py', {
                'camera_config': LaunchConfiguration('camera_config_file'),
                'video_device': LaunchConfiguration('video_device'),
                'left_calibration_file': LaunchConfiguration(
                    'left_calibration_file'),
                'right_calibration_file': LaunchConfiguration(
                    'right_calibration_file'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'start_foxglove_bridge': 'false',
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
                'node_start_interval': LaunchConfiguration('node_start_interval'),
                'log_level': log_level})),
            delayed(sensor_delay + 1.0, include(
                'robot_perception', 'stereo_pointcloud_filter.launch.py', {
                    'output_topic': '/nav/stereo_obstacle_points',
                    'log_level': log_level})),
            delayed(sensor_delay + 1.5, include(
                'robot_perception', 'range_to_scan.launch.py', {
                    'log_level': log_level})),
        ])
        if localization_required:
            common.extend([
                delayed(sensor_delay + 2.0, include(
                    'robot_control', 'head_mapping_lock.launch.py', {
                        'require_feedback': LaunchConfiguration('require_head_feedback'),
                        'log_level': log_level})),
                delayed(sensor_delay + 2.8, include(
                    'robot_navigation', 'map_localization.launch.py', {
                        'database_path': database_path,
                        'config_file': LaunchConfiguration('localization_config_file'),
                        'initial_pose_enabled': LaunchConfiguration('initial_pose_enabled'),
                        'initial_x': LaunchConfiguration('initial_x'),
                        'initial_y': LaunchConfiguration('initial_y'),
                        'initial_yaw': LaunchConfiguration('initial_yaw'),
                        'log_level': log_level})),
            ])
        if not map_yaml:
            common.extend([
                delayed(sensor_delay + 2.0, include(
                    'robot_control', 'head_mapping_lock.launch.py', {
                        'require_feedback': LaunchConfiguration(
                            'require_head_feedback'),
                        'log_level': log_level})),
                delayed(sensor_delay + 2.8, include(
                    'robot_navigation', 'rtabmap_mapping.launch.py', {
                        'log_level': log_level})),
                delayed(sensor_delay + 3.6, include(
                    'robot_navigation', 'mapping_snapshot.launch.py', {
                        'preview_directory': LaunchConfiguration(
                            'preview_directory'),
                        'save_directory': LaunchConfiguration('save_directory'),
                        'log_level': log_level})),
            ])
        if LaunchConfiguration('start_semantic_perception').perform(context).lower() == 'true':
            common.append(delayed(perception_delay, include(
                'robot_perception', 'semantic_detection.launch.py', {
                    'inference_url': LaunchConfiguration('inference_url'),
                    'detection_mode': LaunchConfiguration('detection_mode'),
                    'start_segmentation': LaunchConfiguration(
                        'start_segmentation'),
                    'enable_semantic_navigation': LaunchConfiguration(
                        'enable_semantic_navigation'),
                    'segmentation_rate': LaunchConfiguration(
                        'segmentation_rate'),
                    'log_level': log_level})))
        return common

    return LaunchDescription(declarations + [OpaqueFunction(function=compose)])

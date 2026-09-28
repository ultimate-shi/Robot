"""使用方法：ros2 launch robot_main robot.launch.py mode:=real|virtual 启动整车."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


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
            'initial_x', default_value='0.0',
            description='已有地图模式 map 到 odom 的初始 X，单位米'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='已有地图模式 map 到 odom 的初始 Y，单位米'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='已有地图模式 map 到 odom 的初始偏航角，单位弧度'),
        DeclareLaunchArgument(
            'controller_manager_config_file',
            default_value=os.path.join(control_share, 'config', 'controllers.yaml'),
            description='controller_manager 参数 YAML 路径'),
        DeclareLaunchArgument(
            'controller_config_file',
            default_value=os.path.join(control_share, 'config', 'controllers.yaml'),
            description='统一 ros2_control 控制器参数 YAML 路径'),
        DeclareLaunchArgument(
            'chassis_control_config_file',
            default_value=os.path.join(control_share, 'config', 'chassis_control.yaml'),
            description='实机底盘运动学与轮速看门狗参数 YAML；可加载已验收的逐轮补偿配置'),
        DeclareLaunchArgument(
            'servo_calibration_file',
            default_value=os.path.join(
                hardware_share, 'config', 'servo_calibration.yaml'),
            description='实机六路舵机标定 YAML 路径'),
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
    ]

    def compose(context):
        mode = LaunchConfiguration('mode').perform(context).strip().lower()
        if mode not in ('real', 'virtual'):
            raise RuntimeError("mode 只允许 'real' 或 'virtual'")
        map_yaml = LaunchConfiguration('map_yaml_file').perform(context)
        if mode == 'virtual' and not map_yaml:
            raise RuntimeError(
                'virtual 模式没有实时双目建图源，请显式传入 map_yaml_file；'
                '实机在线建图请使用 mode:=real')
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
            include('robot_description', 'description.launch.py', {
                'hardware_plugin': plugin,
                'servo_calibration_file': LaunchConfiguration(
                    'servo_calibration_file'),
                'log_level': log_level,
                **hardware_arguments}),
            include('robot_description', 'foxglove.launch.py', {
                'port': LaunchConfiguration('foxglove_port'),
                'log_level': LaunchConfiguration('foxglove_log_level')}),
            delayed(control_delay, include('robot_control', 'controllers.launch.py', {
                'hardware_plugin': plugin,
                'servo_calibration_file': LaunchConfiguration(
                    'servo_calibration_file'),
                'manager_config_file': LaunchConfiguration(
                    'controller_manager_config_file'),
                'controller_config_file': LaunchConfiguration(
                    'controller_config_file'),
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
                    'log_level': log_level})),
            delayed(control_delay + 2.4, include(
                'robot_control', 'wheel_odometry.launch.py', {
                    'enabled': LaunchConfiguration('use_wheel_odometry'),
                    'leg_length': LaunchConfiguration('leg_length'),
                    'log_level': log_level,
                    **geometry_arguments})),
            delayed(control_delay + 3.2, include(
                'robot_control', 'safety.launch.py', {
                    'input_topic': '/cmd_vel_nav',
                    'output_topic': '/cmd_vel_safe',
                    'log_level': log_level})),
            include('robot_navigation', 'state_estimation.launch.py', {
                'log_level': log_level}),
            delayed(navigation_delay, include('robot_navigation', 'nav2.launch.py', {
                'map_yaml_file': map_yaml,
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'nav2_overrides_file': nav_overrides,
                'use_map_server': 'true' if map_yaml else 'false',
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'log_level': log_level})),
        ]
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

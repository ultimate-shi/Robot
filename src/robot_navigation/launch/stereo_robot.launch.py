"""使用方法：旧实机入口，仅转发到 robot_main；建议直接使用新入口。."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    control_share = get_package_share_directory('robot_control')
    perception_share = get_package_share_directory('robot_perception')
    declarations = [
        DeclareLaunchArgument('map_yaml_file', default_value='',
                              description='已有地图 YAML；留空在线建图'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='双目相机设备路径'),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=os.path.join(perception_share, 'config', 'stereo_camera.yaml'),
            description='双目相机参数 YAML'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='是否启动真实 IMU'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false',
                              description='视觉里程计是否等待 IMU'),
        DeclareLaunchArgument('require_head_feedback', default_value='false',
                              description='在线建图是否强制头部反馈归中'),
        DeclareLaunchArgument('imu_device', default_value='/dev/gy95t',
                              description='GY95T 设备路径'),
        DeclareLaunchArgument('use_wheel_odometry', default_value='true',
                              description='是否融合四轮反馈里程计'),
        DeclareLaunchArgument('initial_x', default_value='0.0',
                              description='已有地图初始 X，单位米'),
        DeclareLaunchArgument('initial_y', default_value='0.0',
                              description='已有地图初始 Y，单位米'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0',
                              description='已有地图初始偏航角，单位弧度'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2.yaml'),
            description='Nav2 参数 YAML'),
        DeclareLaunchArgument(
            'nav2_overrides_file',
            default_value=os.path.join(nav_share, 'config', 'stereo_robot.yaml'),
            description='实机 Nav2 覆盖 YAML'),
        DeclareLaunchArgument(
            'controller_manager_config_file',
            default_value=os.path.join(control_share, 'config', 'controllers.yaml'),
            description='controller_manager 参数 YAML'),
        DeclareLaunchArgument(
            'controller_config_file',
            default_value=os.path.join(control_share, 'config', 'controllers.yaml'),
            description='控制器参数 YAML'),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(
                perception_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'left.yaml'),
            description='左目相机标定 YAML'),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(
                perception_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'right.yaml'),
            description='右目相机标定 YAML'),
        DeclareLaunchArgument('publish_compressed', default_value='false',
                              description='是否发布压缩深度预览'),
        DeclareLaunchArgument('apply_auto_camera_controls', default_value='true',
                              description='是否开启相机自动曝光和白平衡'),
        DeclareLaunchArgument('inference_url', default_value='http://127.0.0.1:9100',
                              description='本地推理服务地址'),
        DeclareLaunchArgument('detection_mode', default_value='continuous',
                              description='YOLO 检测模式'),
        DeclareLaunchArgument('start_segmentation', default_value='true',
                              description='兼容参数：是否启动语义分割'),
        DeclareLaunchArgument('enable_semantic_navigation', default_value='true',
                              description='兼容参数：是否为导航提供语义障碍'),
        DeclareLaunchArgument('segmentation_rate', default_value='5.0',
                              description='语义分割目标频率'),
        DeclareLaunchArgument('preview_directory', default_value='/tmp/robot_preview',
                              description='临时地图快照目录'),
        DeclareLaunchArgument('save_directory', default_value='/workspace/maps',
                              description='长期地图快照根目录'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='Foxglove WebSocket 端口'),
        DeclareLaunchArgument('foxglove_log_level', default_value='warn',
                              description='Foxglove 日志级别'),
        DeclareLaunchArgument('node_start_interval', default_value='0.8',
                              description='节点错峰间隔，单位秒'),
        DeclareLaunchArgument('sensor_start_delay', default_value='3.0',
                              description='传感器启动延迟，单位秒'),
        DeclareLaunchArgument('control_start_delay', default_value='3.0',
                              description='控制链启动延迟，单位秒'),
        DeclareLaunchArgument('navigation_start_delay', default_value='14.0',
                              description='Nav2 启动延迟，单位秒'),
        DeclareLaunchArgument('perception_start_delay', default_value='8.0',
                              description='语义感知启动延迟，单位秒'),
        DeclareLaunchArgument('log_level', default_value='warn',
                              description='整车日志级别'),
    ]
    forwarded = {
        name: LaunchConfiguration(name) for name in (
            'map_yaml_file', 'video_device', 'camera_config_file', 'use_imu',
            'wait_imu_to_init', 'require_head_feedback', 'imu_device',
            'use_wheel_odometry', 'initial_x', 'initial_y', 'initial_yaw',
            'nav2_params_file', 'nav2_overrides_file', 'controller_config_file',
            'controller_manager_config_file', 'left_calibration_file',
            'right_calibration_file', 'publish_compressed',
            'apply_auto_camera_controls', 'inference_url', 'detection_mode',
            'start_segmentation', 'enable_semantic_navigation',
            'segmentation_rate', 'preview_directory', 'save_directory',
            'foxglove_port', 'foxglove_log_level', 'node_start_interval',
            'sensor_start_delay', 'control_start_delay',
            'navigation_start_delay', 'perception_start_delay', 'log_level')}
    forwarded['mode'] = 'real'
    forwarded['start_semantic_perception'] = LaunchConfiguration(
        'enable_semantic_navigation')
    return LaunchDescription(declarations + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_main'), 'launch', 'robot.launch.py')),
        launch_arguments=forwarded.items())])

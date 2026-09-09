"""使用方法：ros2 launch robot_navigation stereo_robot.launch.py 启动完整在线双目机器人。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def source(package, filename):
    """返回指定功能包内独立 launch 的描述源。"""
    return PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(package), 'launch', filename))


def delayed_include(package, filename, delay, arguments=None, condition=None):
    """在当前作用域解析阶段延迟，再引用独立功能 launch。"""
    include = IncludeLaunchDescription(
        source(package, filename),
        launch_arguments=(arguments or {}).items(),
        condition=condition)

    def create_timer(context):
        resolved_delay = float(delay.perform(context))
        return [TimerAction(
            period=resolved_delay,
            actions=[GroupAction(actions=[include])],
        )]

    return OpaqueFunction(function=create_timer)


def generate_launch_description():
    """组合唯一功能实例，并根据地图参数选择在线建图或已有地图模式。"""
    nav_share = get_package_share_directory('robot_navigation')
    perception_share = get_package_share_directory('robot_perception')
    control_share = get_package_share_directory('robot_control')
    map_yaml = LaunchConfiguration('map_yaml_file')
    log_level = LaunchConfiguration('log_level')
    node_interval = LaunchConfiguration('node_start_interval')
    has_saved_map = PythonExpression(["'", map_yaml, "' != ''"])

    def stage_delay(stage, index=0):
        """计算阶段基础延迟和子功能错峰延迟之和。"""
        return PythonExpression([
            stage, ' + ', node_interval, ' * ', str(index),
        ])

    declarations = [
        DeclareLaunchArgument(
            'map_yaml_file', default_value='',
            description='已有地图 YAML 路径；留空时启用 RTAB-Map 边建图边导航'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value=os.path.join(
                perception_share, 'config', 'stereo_camera.yaml'),
            description='双目相机和图像处理参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='是否启动真实 GY95T，并融合到双目里程计和 /odom'),
        DeclareLaunchArgument(
            'wait_imu_to_init', default_value='false',
            description='是否强制视觉里程计收到 IMU 后才启动'),
        DeclareLaunchArgument(
            'imu_device', default_value='/dev/gy95t',
            description='GY95T USB 转串口在当前运行环境中的设备路径'),
        DeclareLaunchArgument(
            'use_wheel_odometry', default_value='false',
            description='是否融合真实四轮转角和轮速生成的 /wheel/odom'),
        DeclareLaunchArgument(
            'require_head_feedback', default_value='false',
            description='在线建图时是否必须收到头部实际零位反馈'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='已有地图模式下 map 到 odom 的 X 平移，单位为米'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='已有地图模式下 map 到 odom 的 Y 平移，单位为米'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='已有地图模式下 map 到 odom 的偏航角，单位为弧度'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Nav2 基础参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'nav2_overrides_file',
            default_value=os.path.join(
                nav_share, 'config', 'nav2_stereo_overrides.yaml'),
            description='双目实时障碍点云使用的 Nav2 参数覆盖 YAML 文件路径'),
        DeclareLaunchArgument(
            'controller_manager_config_file',
            default_value=os.path.join(
                control_share, 'config', 'controller_manager.yaml'),
            description='ros2_control 管理器参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'controller_config_file',
            default_value=os.path.join(
                control_share, 'config', 'controllers.yaml'),
            description='关节控制器和生成器参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(
                perception_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'left.yaml'),
            description='左目相机标定 YAML 文件路径'),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(
                perception_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'right.yaml'),
            description='右目相机标定 YAML 文件路径'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='false',
            description='是否额外发布深度预览 compressed 图像'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='在线建图临时地图快照目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='在线建图长期地图快照根目录'),
        DeclareLaunchArgument(
            'inference_url', default_value='http://127.0.0.1:9100',
            description='语义感知节点调用的本地推理服务基础 URL'),
        DeclareLaunchArgument(
            'detection_mode', default_value='on_demand',
            description='YOLO 检测模式，可选 on_demand 或 continuous'),
        DeclareLaunchArgument(
            'foxglove_port', default_value='8765',
            description='Foxglove Bridge 监听的 WebSocket 端口'),
        DeclareLaunchArgument(
            'foxglove_log_level', default_value='warn',
            description='Foxglove 日志级别；排查端口监听时可传入 info'),
        DeclareLaunchArgument(
            'node_start_interval', default_value='0.8',
            description='同一阶段相邻功能或节点的错峰启动间隔，单位为秒'),
        DeclareLaunchArgument(
            'sensor_start_delay', default_value='3.0',
            description='启动相机和在线建图链前的秒数'),
        DeclareLaunchArgument(
            'control_start_delay', default_value='3.0',
            description='启动底盘控制链前的秒数'),
        DeclareLaunchArgument(
            'navigation_start_delay', default_value='14.0',
            description='启动 Nav2 前等待里程计和地图输出的秒数'),
        DeclareLaunchArgument(
            'perception_start_delay', default_value='8.0',
            description='启动语义感知和验收采样前的秒数'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='完整在线双目机器人各 ROS 节点的日志级别'),
    ]

    actions = [
        delayed_include(
            'robot_description', 'description.launch.py',
            stage_delay('0.0'), {'log_level': log_level}),
        delayed_include(
            'robot_description', 'foxglove.launch.py',
            stage_delay('0.0'), {
                'port': LaunchConfiguration('foxglove_port'),
                'log_level': LaunchConfiguration('foxglove_log_level'),
            }),
        delayed_include(
            'robot_perception', 'imu.launch.py',
            stage_delay('0.0', 1), {
                'device': LaunchConfiguration('imu_device'),
                'log_level': log_level,
            },
            condition=IfCondition(LaunchConfiguration('use_imu'))),
        delayed_include(
            'robot_navigation', 'stereo_odometry.launch.py',
            stage_delay('0.0', 2), {
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'log_level': log_level,
            }),
        delayed_include(
            'robot_navigation', 'state_estimation.launch.py',
            stage_delay('0.0', 3), {'log_level': log_level}),
        delayed_include(
            'robot_control', 'wheel_odometry.launch.py',
            stage_delay('0.0', 4), {'log_level': log_level},
            condition=IfCondition(
                LaunchConfiguration('use_wheel_odometry'))),
        delayed_include(
            'robot_perception', 'stereo_camera.launch.py',
            stage_delay(LaunchConfiguration('sensor_start_delay')), {
                'camera_config': LaunchConfiguration('camera_config_file'),
                'video_device': LaunchConfiguration('video_device'),
                'left_calibration_file': LaunchConfiguration(
                    'left_calibration_file'),
                'right_calibration_file': LaunchConfiguration(
                    'right_calibration_file'),
                'publish_compressed': LaunchConfiguration(
                    'publish_compressed'),
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
                'start_foxglove_bridge': 'false',
                'node_start_interval': node_interval,
                'log_level': log_level,
            }),
        delayed_include(
            'robot_perception', 'stereo_pointcloud_filter.launch.py',
            stage_delay(LaunchConfiguration('sensor_start_delay'), 9), {
                'output_topic': '/nav/stereo_obstacle_points',
                'log_level': log_level,
            }),
        delayed_include(
            'robot_control', 'head_mapping_lock.launch.py',
            stage_delay(LaunchConfiguration('sensor_start_delay'), 10), {
                'require_feedback': LaunchConfiguration(
                    'require_head_feedback'),
                'log_level': log_level,
            },
            condition=UnlessCondition(has_saved_map)),
        delayed_include(
            'robot_navigation', 'rtabmap_mapping.launch.py',
            stage_delay(LaunchConfiguration('sensor_start_delay'), 11),
            {'log_level': log_level},
            condition=UnlessCondition(has_saved_map)),
        delayed_include(
            'robot_navigation', 'mapping_snapshot.launch.py',
            stage_delay(LaunchConfiguration('sensor_start_delay'), 12), {
                'preview_directory': LaunchConfiguration('preview_directory'),
                'save_directory': LaunchConfiguration('save_directory'),
                'log_level': log_level,
            },
            condition=UnlessCondition(has_saved_map)),
        delayed_include(
            'robot_control', 'controllers.launch.py',
            stage_delay(LaunchConfiguration('control_start_delay')), {
                'manager_config_file': LaunchConfiguration(
                    'controller_manager_config_file'),
                'controller_config_file': LaunchConfiguration(
                    'controller_config_file'),
                'log_level': log_level,
            }),
        delayed_include(
            'robot_control', 'chassis_control.launch.py',
            stage_delay(LaunchConfiguration('control_start_delay'), 1), {
                'chassis_cmd_topic': '/cmd_vel_safe',
                'publish_odometry': 'false',
                'node_start_interval': node_interval,
                'log_level': log_level,
            }),
        delayed_include(
            'robot_control', 'nav_velocity_gate.launch.py',
            stage_delay(LaunchConfiguration('control_start_delay'), 3), {
                'log_level': log_level,
            }),
        delayed_include(
            'robot_control', 'obstacle_avoidance.launch.py',
            stage_delay(LaunchConfiguration('control_start_delay'), 4), {
                'input_topic': '/cmd_vel_nav',
                'output_topic': '/cmd_vel_safe',
                'log_level': log_level,
            }),
        delayed_include(
            'robot_navigation', 'nav2.launch.py',
            stage_delay(LaunchConfiguration('navigation_start_delay')), {
                'map_yaml_file': map_yaml,
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'nav2_overrides_file': LaunchConfiguration(
                    'nav2_overrides_file'),
                'use_map_server': has_saved_map,
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
                'node_start_interval': node_interval,
                'log_level': log_level,
            }),
        delayed_include(
            'robot_perception', 'semantic_detection.launch.py',
            stage_delay(LaunchConfiguration('perception_start_delay')), {
                'inference_url': LaunchConfiguration('inference_url'),
                'detection_mode': LaunchConfiguration('detection_mode'),
                'log_level': log_level,
            }),
        delayed_include(
            'robot_perception', 'acceptance_sampler.launch.py',
            stage_delay(LaunchConfiguration('perception_start_delay'), 1), {
                'log_level': log_level,
            }),
    ]
    return LaunchDescription(declarations + actions)

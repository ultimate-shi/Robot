"""RK3588 实机入口：公共底盘/Nav2 加独立真实双目感知."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    launch_dir = os.path.join(pkg_share, 'launch')
    stereo_overrides = os.path.join(
        pkg_share, 'config', 'nav2_stereo_overrides.yaml')
    pointcloud_config = os.path.join(
        pkg_share, 'config', 'stereo_pointcloud.yaml')
    collision_config = os.path.join(
        pkg_share, 'config', 'stereo_collision_monitor.yaml')
    terrain_config = os.path.join(
        pkg_share, 'config', 'terrain_params.yaml')
    ultrasonic_enabled = LaunchConfiguration('enable_ultrasonic_avoidance')
    collision_output_topic = PythonExpression([
        '"/cmd_vel_stereo_safe" if "', ultrasonic_enabled,
        '" == "true" else "/cmd_vel_safe"',
    ])

    shared_args = {
        name: LaunchConfiguration(name)
        for name in (
            'initial_x', 'initial_y', 'initial_yaw', 'map_yaml_file',
            'nav2_params_file', 'log_level', 'foxglove_port',
        )
    }
    declarations = [
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(pkg_share, 'map', 'studyroom.yaml')),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument(
            'enable_ultrasonic_avoidance', default_value='false'),
        DeclareLaunchArgument('calibration_mode', default_value='false'),
        DeclareLaunchArgument(
            'video_device', default_value='/dev/stereo_camera'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(
                pkg_share, 'config', 'stereo_camera.yaml')),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(
                pkg_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480',
                'left.yaml')),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(
                pkg_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480',
                'right.yaml')),
    ]
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'common_bringup.launch.py')),
        launch_arguments={
            **shared_args,
            'nav2_overrides_file': stereo_overrides,
            # 底盘只接受全部安全层处理后的速度，安全节点未激活时不会运动。
            'chassis_cmd_topic': '/cmd_vel_safe',
        }.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'stereo_camera.launch.py')),
        launch_arguments={
            'calibration_mode': LaunchConfiguration('calibration_mode'),
            'camera_config': LaunchConfiguration('camera_config'),
            'video_device': LaunchConfiguration('video_device'),
            'left_calibration_file': LaunchConfiguration(
                'left_calibration_file'),
            'right_calibration_file': LaunchConfiguration(
                'right_calibration_file'),
            # 完整实机入口由 common_bringup 启动 Bridge，内部关闭相机重复实例。
            '_camera_start_foxglove_bridge': 'false',
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )
    point_filter = Node(
        package='robot',
        executable='stereo_pointcloud_filter',
        name='stereo_pointcloud_filter',
        parameters=[pointcloud_config, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        output='screen',
    )
    scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[pointcloud_config, {'use_sim_time': False}],
        remappings=[
            ('cloud_in', '/nav/stereo_obstacle_points'),
            ('scan', '/stereo/scan'),
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        output='screen',
    )
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        parameters=[
            collision_config,
            {
                'use_sim_time': False,
                'cmd_vel_in_topic': '/cmd_vel_nav',
                'cmd_vel_out_topic': collision_output_topic,
            },
        ],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        output='screen',
    )
    collision_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_stereo_collision',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['collision_monitor'],
        }],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        output='screen',
    )
    ultrasonic_avoidance = Node(
        package='robot',
        executable='obstacle_avoidance',
        name='ultrasonic_obstacle_avoidance',
        parameters=[
            terrain_config,
            {
                'use_sim_time': False,
                # 实机紧急层禁止自行倒车，避免脱离双目可视范围。
                'escape_reverse_enabled': False,
                'require_valid_ranges': True,
            },
        ],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_stereo_safe'),
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        condition=IfCondition(ultrasonic_enabled),
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        output='screen',
    )
    return LaunchDescription(declarations + [
        common,
        camera,
        point_filter,
        scan,
        collision_monitor,
        collision_lifecycle,
        ultrasonic_avoidance,
    ])

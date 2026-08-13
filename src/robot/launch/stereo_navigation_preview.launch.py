"""地图预演入口：加载真实双目快照并驱动虚拟机器人导航。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    launch_dir = os.path.join(pkg_share, 'launch')
    terrain_config = os.path.join(
        pkg_share, 'config', 'terrain_params.yaml')
    preview_config = os.path.join(
        pkg_share, 'config', 'navigation_preview.yaml')
    collision_config = os.path.join(
        pkg_share, 'config', 'stereo_collision_monitor.yaml')
    nav2_overrides = os.path.join(
        pkg_share, 'config', 'nav2_stereo_overrides.yaml')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]

    declarations = [
        DeclareLaunchArgument(
            'map_yaml_file', default_value='/tmp/robot_preview/current.yaml'),
        DeclareLaunchArgument(
            'ply_file', default_value='/tmp/robot_preview/current.ply'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'preview_config', default_value=preview_config),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('log_level', default_value='warn'),
    ]

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'common_bringup.launch.py')),
        launch_arguments={
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'map_yaml_file': LaunchConfiguration('map_yaml_file'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'nav2_overrides_file': nav2_overrides,
            'chassis_cmd_topic': '/cmd_vel_safe',
            'foxglove_port': LaunchConfiguration('foxglove_port'),
            'foxglove_send_buffer_limit': '4000000',
            'log_level': log_level,
        }.items(),
    )
    environment_cloud = Node(
        package='robot',
        executable='publish_ply',
        name='preview_environment_cloud',
        parameters=[{
            'ply_file': LaunchConfiguration('ply_file'),
            'frame_id': 'map',
            'display_topic': '/mapping/cloud_map',
            'perception_topic': '/perception/points',
            'publish_period': 0.5,
            'allow_fallback': False,
        }],
        arguments=ros_args,
        output='screen',
    )
    local_observer = Node(
        package='robot',
        executable='snapshot_local_observer',
        name='snapshot_local_observer',
        parameters=[LaunchConfiguration('preview_config')],
        arguments=ros_args,
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
                'cmd_vel_out_topic': '/cmd_vel_stereo_safe',
                'base_shift_correction': False,
            },
        ],
        arguments=ros_args,
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
        arguments=ros_args,
        output='screen',
    )
    virtual_ultrasonic = Node(
        package='robot',
        executable='virtual_ultrasonic',
        name='virtual_ultrasonic',
        parameters=[
            terrain_config,
            {
                'use_sim_time': False,
                'input_topic': '/perception/points',
                'cache_static_source': True,
                'source_timeout': 0.0,
            },
        ],
        arguments=ros_args,
        output='screen',
    )
    ultrasonic_safety = Node(
        package='robot',
        executable='obstacle_avoidance',
        name='ultrasonic_obstacle_avoidance',
        parameters=[
            terrain_config,
            {
                'use_sim_time': False,
                'escape_reverse_enabled': False,
                'require_valid_ranges': True,
            },
        ],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_stereo_safe'),
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        arguments=ros_args,
        output='screen',
    )
    goal_manager = Node(
        package='robot',
        executable='goal_manager',
        name='goal_manager',
        parameters=[LaunchConfiguration('preview_config')],
        arguments=ros_args,
        output='screen',
    )

    return LaunchDescription(declarations + [
        common,
        environment_cloud,
        local_observer,
        collision_monitor,
        collision_lifecycle,
        virtual_ultrasonic,
        ultrasonic_safety,
        goal_manager,
    ])

"""现有仿真入口：公共 bringup 加 PLY、虚拟地形/IMU/超声波感知。"""

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
    terrain_params = os.path.join(
        pkg_share, 'config', 'terrain_params.yaml')
    use_pointcloud = LaunchConfiguration('use_pointcloud_map')
    enable_ultrasonic = LaunchConfiguration(
        'enable_ultrasonic_avoidance')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]
    ultrasonic_enabled = PythonExpression([
        '"', use_pointcloud, '" == "true" and "',
        enable_ultrasonic, '" == "true"',
    ])
    chassis_topic = PythonExpression([
        '"/cmd_vel_safe" if ("', use_pointcloud,
        '" == "true" and "', enable_ultrasonic,
        '" == "true") else "/cmd_vel_nav"',
    ])

    declarations = [
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_pointcloud_map', default_value='true'),
        DeclareLaunchArgument(
            'enable_ultrasonic_avoidance', default_value='true'),
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=os.path.join(
                pkg_share, 'map', 'studyroom.yaml')),
        DeclareLaunchArgument(
            'ply_file',
            default_value=os.path.join(
                pkg_share, 'map', 'studyroom.ply')),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('foxglove_enabled', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
    ]
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share, 'launch', 'common_bringup.launch.py')),
        launch_arguments={
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
            'initial_yaw': LaunchConfiguration('initial_yaw'),
            'map_yaml_file': LaunchConfiguration('map_yaml_file'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'chassis_cmd_topic': chassis_topic,
            'log_level': log_level,
            'foxglove_enabled': LaunchConfiguration('foxglove_enabled'),
            'foxglove_port': LaunchConfiguration('foxglove_port'),
        }.items(),
    )
    publish_ply = Node(
        package='robot',
        executable='publish_ply',
        name='publish_ply',
        parameters=[{'ply_file': LaunchConfiguration('ply_file')}],
        condition=IfCondition(use_pointcloud),
        arguments=ros_args,
        output='screen',
    )
    point_filter = Node(
        package='robot',
        executable='pointcloud_obstacle_filter',
        name='pointcloud_obstacle_filter',
        parameters=[terrain_params, {'use_sim_time': False}],
        condition=IfCondition(use_pointcloud),
        arguments=ros_args,
        output='screen',
    )
    terrain = Node(
        package='robot',
        executable='terrain_analyzer',
        name='terrain_analyzer',
        parameters=[terrain_params, {'use_sim_time': False}],
        condition=IfCondition(use_pointcloud),
        arguments=ros_args,
        output='screen',
    )
    virtual_ultrasonic = Node(
        package='robot',
        executable='virtual_ultrasonic',
        name='virtual_ultrasonic',
        parameters=[terrain_params, {'use_sim_time': False}],
        condition=IfCondition(ultrasonic_enabled),
        arguments=ros_args,
        output='screen',
    )
    range_to_scan = Node(
        package='robot',
        executable='range_to_scan',
        name='range_to_scan',
        parameters=[terrain_params, {'use_sim_time': False}],
        condition=IfCondition(use_pointcloud),
        arguments=ros_args,
        output='screen',
    )
    obstacle_avoidance = Node(
        package='robot',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[terrain_params, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_nav'),
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        condition=IfCondition(ultrasonic_enabled),
        arguments=ros_args,
        output='screen',
    )
    virtual_imu = Node(
        package='robot',
        executable='virtual_imu',
        name='virtual_imu',
        parameters=[terrain_params, {'use_sim_time': False}],
        arguments=ros_args,
        output='screen',
    )
    return LaunchDescription(declarations + [
        common,
        publish_ply,
        point_filter,
        terrain,
        virtual_ultrasonic,
        range_to_scan,
        obstacle_avoidance,
        virtual_imu,
    ])

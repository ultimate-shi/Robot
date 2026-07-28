"""RK3588 实机入口：公共底盘/Nav2 加独立真实双目感知。"""

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
    stereo_overrides = os.path.join(
        pkg_share, 'config', 'nav2_stereo_overrides.yaml')
    pointcloud_config = os.path.join(
        pkg_share, 'config', 'stereo_pointcloud.yaml')

    shared_args = {
        name: LaunchConfiguration(name)
        for name in (
            'initial_x', 'initial_y', 'initial_yaw', 'map_yaml_file',
            'nav2_params_file', 'log_level', 'foxglove_enabled',
            'foxglove_port',
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
        DeclareLaunchArgument('foxglove_enabled', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('calibration_mode', default_value='false'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(
                pkg_share, 'config', 'stereo_camera.yaml')),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(
                pkg_share, 'config', 'cameras', '_template_640x480',
                'left.yaml')),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(
                pkg_share, 'config', 'cameras', '_template_640x480',
                'right.yaml')),
    ]
    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'common_bringup.launch.py')),
        launch_arguments={
            **shared_args,
            'nav2_overrides_file': stereo_overrides,
            'chassis_cmd_topic': '/cmd_vel_nav',
        }.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'stereo_camera.launch.py')),
        launch_arguments={
            'calibration_mode': LaunchConfiguration('calibration_mode'),
            'camera_config': LaunchConfiguration('camera_config'),
            'left_calibration_file': LaunchConfiguration(
                'left_calibration_file'),
            'right_calibration_file': LaunchConfiguration(
                'right_calibration_file'),
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
    return LaunchDescription(declarations + [
        common, camera, point_filter, scan])

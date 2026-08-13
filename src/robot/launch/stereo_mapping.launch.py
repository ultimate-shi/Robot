"""真实双目建图入口：只建图和显示，不启动导航或虚拟底盘."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    launch_dir = os.path.join(pkg_share, 'launch')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    mapping_config = os.path.join(
        pkg_share, 'config', 'rtabmap_stereo_mapping.yaml')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]

    declarations = [
        DeclareLaunchArgument(
            'video_device', default_value='/dev/stereo_camera'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=os.path.join(
                pkg_share, 'config', 'stereo_camera.yaml')),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true'),
        DeclareLaunchArgument(
            'left_calibration_file',
            default_value=os.path.join(
                pkg_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'left.yaml')),
        DeclareLaunchArgument(
            'right_calibration_file',
            default_value=os.path.join(
                pkg_share, 'config', 'cameras',
                'usb_camera_01_00_00_640x480', 'right.yaml')),
        DeclareLaunchArgument(
            'mapping_config', default_value=mapping_config),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('log_level', default_value='warn'),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=ros_args,
        output='screen',
    )
    # 当前实机云台没有反馈，建图期间持续发布全部关节零位，
    # 确保相机 TF 固定。
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description, {
            'rate': 10,
            'publish_default_positions': True,
        }],
        arguments=ros_args,
        output='screen',
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'stereo_camera.launch.py')),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_config': LaunchConfiguration('camera_config'),
            'apply_auto_camera_controls': LaunchConfiguration(
                'apply_auto_camera_controls'),
            'left_calibration_file': LaunchConfiguration(
                'left_calibration_file'),
            'right_calibration_file': LaunchConfiguration(
                'right_calibration_file'),
            # 建图入口在父级启动 Bridge，内部关闭相机重复实例。
            '_camera_start_foxglove_bridge': 'false',
            'publish_compressed': 'false',
            'navigation_processing_enabled': 'false',
            'log_level': log_level,
        }.items(),
    )
    stereo_odometry = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        parameters=[LaunchConfiguration('mapping_config')],
        remappings=[
            ('left/image_rect', '/stereo/left/image_rect'),
            ('right/image_rect', '/stereo/right/image_rect'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
            ('odom', '/visual_odom'),
            ('odom_info', '/visual_odom_info'),
            ('imu', '/sensors/imu/data'),
        ],
        arguments=ros_args,
        output='screen',
    )
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[LaunchConfiguration('mapping_config')],
        remappings=[
            ('left/image_rect', '/stereo/left/image_rect'),
            ('right/image_rect', '/stereo/right/image_rect'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
            ('odom', '/visual_odom'),
            ('odom_info', '/visual_odom_info'),
            ('map', '/map'),
            ('mapData', '/rtabmap/mapData'),
            # RTAB-Map 本身直接发布全局点云，无需额外转换节点。
            ('cloud_map', '/mapping/cloud_map'),
        ],
        arguments=['--delete_db_on_start'] + ros_args,
        output='screen',
    )
    snapshot = Node(
        package='robot',
        executable='mapping_snapshot_manager',
        name='mapping_snapshot_manager',
        parameters=[
            LaunchConfiguration('mapping_config'),
            {
                'preview_directory': LaunchConfiguration(
                    'preview_directory'),
                'save_directory': LaunchConfiguration('save_directory'),
            },
        ],
        arguments=ros_args,
        output='screen',
    )
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': ParameterValue(
                LaunchConfiguration('foxglove_port'), value_type=int),
            'address': '0.0.0.0',
            'asset_uri_allowlist': ['package://robot/.*'],
            'allow_file_transfer': True,
            'send_buffer_limit': 2000000,
            'max_packet_messages': 50,
            'client_timeout_ms': 300000,
            'keep_alive_interval_ms': 5000,
        }],
        arguments=ros_args,
        output='screen',
    )

    return LaunchDescription(declarations + [
        robot_state_publisher,
        joint_state_publisher,
        camera,
        stereo_odometry,
        rtabmap,
        snapshot,
        foxglove,
    ])

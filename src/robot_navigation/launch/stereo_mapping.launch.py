"""使用方法：ros2 launch robot_navigation stereo_mapping.launch.py 进行真实双目在线建图。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('robot_navigation')
    perception_share = get_package_share_directory('robot_perception')
    description_share = get_package_share_directory('robot_description')
    mapping_config = os.path.join(nav_share, 'config', 'rtabmap_stereo_mapping.yaml')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device', default_value='/dev/video0',
            description='容器内的双目相机视频设备路径'),
        DeclareLaunchArgument(
            'navigation_processing_enabled', default_value='false',
            description='建图时是否额外启动导航用视差、深度和点云处理'),
        DeclareLaunchArgument(
            'publish_compressed', default_value='false',
            description='建图时是否显式发布深度预览压缩话题'),
        DeclareLaunchArgument(
            'apply_auto_camera_controls', default_value='true',
            description='取流前是否开启并回读相机自动曝光和自动白平衡'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='临时导航预演地图快照的保存目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='长期地图快照的根保存目录'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='双目建图、地图快照及可视化节点的日志级别'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_share, 'launch', 'description.launch.py')),
            launch_arguments={'log_level': log_level}.items()),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'rate': 10, 'publish_default_positions': True}],
             arguments=ros_args, output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                perception_share, 'launch', 'stereo_camera.launch.py')),
            launch_arguments={
                'video_device': LaunchConfiguration('video_device'),
                'navigation_processing_enabled': LaunchConfiguration('navigation_processing_enabled'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'apply_auto_camera_controls': LaunchConfiguration(
                    'apply_auto_camera_controls'),
                '_camera_start_foxglove_bridge': 'false',
                'log_level': log_level,
            }.items()),
        Node(package='rtabmap_odom', executable='stereo_odometry', name='stereo_odometry',
             parameters=[mapping_config], remappings=[
                 ('left/image_rect', '/stereo/left/image_rect'),
                 ('right/image_rect', '/stereo/right/image_rect'),
                 ('left/camera_info', '/stereo/left/camera_info'),
                 ('right/camera_info', '/stereo/right/camera_info'),
                 ('odom', '/visual_odom'), ('odom_info', '/visual_odom_info'),
                 ('imu', '/sensors/imu/data')], arguments=ros_args, output='screen'),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             parameters=[mapping_config], remappings=[
                 ('left/image_rect', '/stereo/left/image_rect'),
                 ('right/image_rect', '/stereo/right/image_rect'),
                 ('left/camera_info', '/stereo/left/camera_info'),
                 ('right/camera_info', '/stereo/right/camera_info'),
                 ('odom', '/visual_odom'), ('odom_info', '/visual_odom_info'),
                 ('map', '/map'), ('mapData', '/rtabmap/mapData'),
                 ('cloud_map', '/mapping/cloud_map')],
             arguments=['--delete_db_on_start', *ros_args], output='screen'),
        Node(package='robot_navigation', executable='mapping_snapshot_manager',
             parameters=[mapping_config, {
                 'preview_directory': LaunchConfiguration('preview_directory'),
                 'save_directory': LaunchConfiguration('save_directory')}],
             arguments=ros_args, output='screen'),
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[{'port': 8765, 'address': '0.0.0.0',
                          'asset_uri_allowlist': ['package://robot_description/.*'],
                          'allow_file_transfer': True}],
             arguments=ros_args, output='screen'),
    ])

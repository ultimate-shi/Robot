"""作用：启动 Astra Pro 彩色/原生深度、精确配准及统一 RGB-D 接口；用法：ros2 launch robot_perception astra_camera.launch.py，需安装官方驱动。"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = LaunchConfiguration('config_file')
    level_args = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(get_package_share_directory('robot_perception'), 'config', 'astra_camera.yaml'), description='Astra Pro 驱动、同步、配准、限速与点云参数 YAML'),
        DeclareLaunchArgument('start_driver', default_value='true', description='是否启动 astra_camera 驱动；回放或外部驱动时关闭'),
        DeclareLaunchArgument('log_level', default_value='warn', description='相机和 RGB-D 处理日志级别'),
        Node(package='astra_camera', executable='astra_camera_node', namespace='astra', name='camera',
             parameters=[config], condition=IfCondition(LaunchConfiguration('start_driver')), arguments=level_args, output='screen'),
        Node(package='image_proc', executable='rectify_node', name='color_rectify',
             remappings=[('image', '/astra/color/image_raw'), ('camera_info', '/astra/color/camera_info'),
                         ('image_rect', '/astra/rect/color')], arguments=level_args, output='screen'),
        Node(package='image_proc', executable='rectify_node', name='depth_rectify', parameters=[config],
             remappings=[('image', '/astra/depth/image_raw'), ('camera_info', '/astra/depth/camera_info'),
                         ('image_rect', '/astra/rect/depth')], arguments=level_args, output='screen'),
        Node(package='depth_image_proc', executable='register_node', name='depth_register', parameters=[config],
             remappings=[('depth/image_rect', '/astra/rect/depth'), ('depth/camera_info', '/astra/depth/camera_info'),
                         ('rgb/camera_info', '/astra/color/camera_info'),
                         ('depth_registered/image_rect', '/astra/registered/depth'),
                         ('depth_registered/camera_info', '/astra/registered/camera_info')], arguments=level_args, output='screen'),
        Node(package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync', parameters=[config],
             remappings=[('rgb/image', '/astra/rect/color'), ('depth/image', '/astra/registered/depth'),
                         ('rgb/camera_info', '/astra/color/camera_info'), ('rgbd_image', '/astra/rgbd_raw')], arguments=level_args, output='screen'),
        Node(package='robot_stereo_components', executable='rgbd_throttle_cpp', name='rgbd_throttle_cpp', parameters=[config],
             remappings=[('rgbd_in', '/astra/rgbd_raw'), ('rgbd_out', '/camera/rgbd_image'),
                         ('rgb_out', '/camera/rgb/image_rect'), ('depth_out', '/camera/depth/image_rect'),
                         ('info_out', '/camera/rgb/camera_info'), ('preview_out', '/camera/depth/image_visual')], arguments=level_args, output='screen'),
        Node(package='rtabmap_util', executable='point_cloud_xyz', name='astra_cloud', parameters=[config],
             remappings=[('depth/image', '/camera/depth/image_rect'),
                         ('depth/camera_info', '/camera/rgb/camera_info'), ('cloud', '/camera/points')], arguments=level_args, output='screen'),
    ])

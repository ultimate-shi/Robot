"""使用方法：ros2 launch robot_navigation stereo_odometry.launch.py 发布独立双目视觉里程计。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'stereo_odometry.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='双目视觉里程计使用的 RTAB-Map 参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'wait_imu_to_init', default_value='false',
            description='是否强制收到 IMU 姿态后才启动视觉里程计'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='双目视觉里程计节点的 ROS 日志级别'),
        Node(
            package='rtabmap_odom', executable='stereo_odometry',
            name='stereo_odometry',
            parameters=[LaunchConfiguration('config_file'), {
                'publish_tf': False,
                'wait_imu_to_init': ParameterValue(
                    LaunchConfiguration('wait_imu_to_init'), value_type=bool),
            }],
            remappings=[
                ('left/image_rect', '/stereo/left/image_rect'),
                ('right/image_rect', '/stereo/right/image_rect'),
                ('left/camera_info', '/stereo/left/camera_info'),
                ('right/camera_info', '/stereo/right/camera_info'),
                ('odom', '/visual_odom'),
                ('odom_info', '/visual_odom_info'),
                ('imu', '/sensors/imu/data'),
            ],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

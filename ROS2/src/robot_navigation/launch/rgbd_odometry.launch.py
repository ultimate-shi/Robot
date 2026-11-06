"""作用：以已配准 RGB-D 替代双目视差里程计；用法：ros2 launch robot_navigation rgbd_odometry.launch.py。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(get_package_share_directory('robot_navigation'), 'config', 'rgbd_odometry.yaml'), description='RGB-D 视觉里程计参数 YAML'),
        DeclareLaunchArgument('rgbd_topic', default_value='/camera/rgbd_image', description='已配准、已校验时间差的 RGBDImage 话题'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false', description='是否等待 IMU 姿态后初始化'),
        DeclareLaunchArgument('log_level', default_value='warn', description='RGB-D 里程计日志级别'),
        Node(package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
             parameters=[LaunchConfiguration('config_file'), {'wait_imu_to_init': ParameterValue(LaunchConfiguration('wait_imu_to_init'), value_type=bool)}],
             remappings=[('rgbd_image', LaunchConfiguration('rgbd_topic')), ('odom', '/visual_odom_raw'),
                         ('odom_info', '/visual_odom_info'), ('imu', '/sensors/imu/data')],
             arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')], output='screen'),
    ])

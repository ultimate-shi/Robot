"""作用：启动分类话题录制与独立精简索引；使用方法：ros2 launch robot_logging logging.launch.py mode:=real|virtual。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_logging')
    default_config = os.path.join(share, 'config', 'logging.yaml')
    default_mcap = os.path.join(share, 'config', 'mcap.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='virtual',
            description='日志会话的运行模式：real 或 virtual'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='日志路径、配额、切片及两个话题策略文件路径'),
        DeclareLaunchArgument(
            'mcap_config_file', default_value=default_mcap,
            description='MCAP 分块压缩参数文件'),
        Node(
            package='robot_logging', executable='log_manager',
            name='robot_log_manager', output='screen', sigterm_timeout='45',
            parameters=[{'config_file': LaunchConfiguration('config_file'),
                         'mcap_config_file': LaunchConfiguration('mcap_config_file'),
                         'mode': LaunchConfiguration('mode')}]),
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot_logging', 'log_indexer',
                 '--config-file', LaunchConfiguration('config_file')],
            output='screen', sigterm_timeout='120'),
    ])

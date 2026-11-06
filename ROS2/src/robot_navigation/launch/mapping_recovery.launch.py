"""作用：单独恢复并保存异常退出的地图；用法：ros2 launch robot_navigation mapping_recovery.launch.py database_path:=残留库。"""

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_navigation'), 'config', 'mapping_recovery.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config, description='离线地图恢复参数 YAML'),
        DeclareLaunchArgument('database_path', default_value='', description='残留活跃数据库，留空使用恢复配置；自动比较 .back'),
        DeclareLaunchArgument('save_directory', default_value='', description='地图根目录，留空使用恢复配置'),
        DeclareLaunchArgument('preview_directory', default_value='', description='缓存快照目录，留空使用恢复配置'),
        ExecuteProcess(cmd=[os.path.join(get_package_prefix('robot_navigation'), 'lib', 'robot_navigation', 'mapping_recover'),
                            '--config', LaunchConfiguration('config_file'),
                            '--database-path', LaunchConfiguration('database_path'),
                            '--save-directory', LaunchConfiguration('save_directory'),
                            '--preview-directory', LaunchConfiguration('preview_directory')], output='screen'),
    ])

"""作用：管理在线保存及退出后恢复；使用方法：mapping_snapshot.launch.py database_path:=与建图相同的库。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'mapping_snapshot.yaml')

    def start(context):
        parameters = {
            'preview_directory': LaunchConfiguration('preview_directory').perform(context),
            'save_directory': LaunchConfiguration('save_directory').perform(context),
            'recovery_config_file': LaunchConfiguration('recovery_config_file').perform(context),
        }
        database = LaunchConfiguration('database_path').perform(context).strip()
        if database:
            parameters['database_path'] = database
        return [Node(
            package='robot_navigation', executable='mapping_snapshot_manager',
            parameters=[LaunchConfiguration('config_file').perform(context), parameters],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level').perform(context)],
            output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path', default_value='',
            description='与在线建图一致的活跃库；留空沿用快照 YAML，退出后用于恢复'),
        DeclareLaunchArgument(
            'recovery_config_file', default_value=os.path.join(
                get_package_share_directory('robot_navigation'), 'config', 'mapping_recovery.yaml'),
            description='退出后地图恢复导出参数 YAML'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='地图快照管理器使用的建图参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'preview_directory', default_value='/tmp/robot_preview',
            description='临时地图快照的保存目录'),
        DeclareLaunchArgument(
            'save_directory', default_value='/workspace/maps',
            description='长期地图快照的根保存目录'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='地图快照管理节点的 ROS 日志级别'),
        OpaqueFunction(function=start),
    ])

"""作用：保留数据库建图并在异常退出后离线恢复保存；用法：rtabmap_mapping.launch.py camera_source:=stereo|astra。"""

import os

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config', 'rtabmap_mapping.yaml')
    def start(context):
        source = LaunchConfiguration('camera_source').perform(context)
        if source not in ('stereo', 'astra'):
            raise ValueError('camera_source 只能为 stereo 或 astra')
        config = LaunchConfiguration('config_file').perform(context)
        with open(config, encoding='utf-8') as stream:
            default_database = yaml.safe_load(stream)['rtabmap']['ros__parameters']['database_path']
        database = str(Path(LaunchConfiguration('database_path').perform(context).strip() or
                            default_database).expanduser().resolve())
        Path(database).parent.mkdir(parents=True, exist_ok=True)
        # 现有数据库永远不删除；需新建时指定新的 database_path。
        recovery_enabled = LaunchConfiguration('recover_on_exit').perform(context).lower() == 'true'
        recovery_config = LaunchConfiguration('recovery_config_file').perform(context)
        save_directory = LaunchConfiguration('save_directory').perform(context)
        preview_directory = LaunchConfiguration('preview_directory').perform(context)
        node = Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            parameters=[config, {'database_path': database,
                'subscribe_stereo': source == 'stereo', 'subscribe_depth': False,
                'subscribe_rgbd': source == 'astra'}],
            remappings=[('rgbd_image', '/camera/rgbd_image'),
                ('left/image_rect', '/stereo/left/image_rect'), ('right/image_rect', '/stereo/right/image_rect'),
                ('left/camera_info', '/stereo/left/camera_info'), ('right/camera_info', '/stereo/right/camera_info'),
                ('odom', '/odom'), ('map', '/map'), ('mapData', '/rtabmap/mapData'), ('cloud_map', '/mapping/cloud_map')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level').perform(context)],
            output='screen')

        def after_exit(event, launch_context):
            # 正常退出/用户关闭整车不触发；不自动重启硬件或在线建图。
            if not recovery_enabled or event.returncode == 0 or launch_context.is_shutdown:
                return []
            return [LogInfo(msg='RTAB-Map 异常退出，正在独立恢复保存残留数据库；不重启底盘'),
                    ExecuteProcess(cmd=[os.path.join(get_package_prefix('robot_navigation'), 'lib',
                                                     'robot_navigation', 'mapping_recover'),
                                        '--config', recovery_config, '--database-path', database,
                                        '--save-directory', save_directory,
                                        '--preview-directory', preview_directory], output='screen')]

        return [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=after_exit)), node]
    return LaunchDescription([
        DeclareLaunchArgument('camera_source', default_value='stereo', description='建图输入 stereo 或 astra RGB-D'),
        DeclareLaunchArgument('database_path', default_value='', description='活跃数据库路径，留空使用建图 YAML；不清库，新会话请指定新文件名'),
        DeclareLaunchArgument('recover_on_exit', default_value='true', description='建图异常退出后是否自动离线恢复保存地图'),
        DeclareLaunchArgument('recovery_config_file', default_value=os.path.join(
            get_package_share_directory('robot_navigation'), 'config', 'mapping_recovery.yaml'), description='异常退出后独立恢复参数 YAML'),
        DeclareLaunchArgument('save_directory', default_value='/workspace/maps', description='异常退出恢复地图的根目录'),
        DeclareLaunchArgument('preview_directory', default_value='/tmp/robot_preview', description='用于核对缓存点云的快照目录'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='RTAB-Map 在线建图参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='RTAB-Map 在线建图节点的 ROS 日志级别'),
        OpaqueFunction(function=start),
    ])

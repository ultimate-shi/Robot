"""作用：重载 RTAB-Map 数据库并监督定位；使用方法：传入与静态地图成套的 database_path。"""

import os
import tempfile
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_navigation.mapping.database import copy_database


def generate_launch_description():
    share = get_package_share_directory('robot_navigation')
    config = LaunchConfiguration('config_file')
    def start(context):
        # 定位仅修改临时副本，长期保存的地图数据库保持原样。
        temporary = tempfile.TemporaryDirectory(prefix='robot-localization-')
        database = os.path.join(temporary.name, 'map.db')
        try:
            copy_database(LaunchConfiguration('database_path').perform(context), database)
        except Exception:
            temporary.cleanup()
            raise
        parameters = config.perform(context)
        log_level = LaunchConfiguration('log_level').perform(context)
        enabled = LaunchConfiguration('initial_pose_enabled').perform(context).lower()
        if enabled not in ('true', 'false'):
            raise ValueError('initial_pose_enabled 只能为 true 或 false')
        initial = {key: float(LaunchConfiguration(key).perform(context))
                   for key in ('initial_x', 'initial_y', 'initial_yaw')}
        if not all(math.isfinite(value) for value in initial.values()):
            raise ValueError('启动初始位姿必须是有限数值')
        rtabmap = Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             parameters=[os.path.join(share, 'config', 'rtabmap_mapping.yaml'), parameters,
                         {'database_path': database,
                          'Mem/IncrementalMemory': 'false',
                          'pub_loc_pose_only_when_localizing': True,
                          'publish_tf': True}],
             remappings=[('left/image_rect', '/stereo/left/image_rect'),
                         ('right/image_rect', '/stereo/right/image_rect'),
                         ('left/camera_info', '/stereo/left/camera_info'),
                         ('right/camera_info', '/stereo/right/camera_info'),
                         ('odom', '/odom'), ('map', '/localization/map'),
                         ('localization_pose', '/rtabmap/localization_pose'),
                         ('initialpose', '/initialpose')],
             arguments=['--ros-args', '--log-level', log_level], output='screen')
        return [RegisterEventHandler(OnProcessExit(
            target_action=rtabmap, on_exit=lambda event, context: temporary.cleanup())), rtabmap,
        Node(package='robot_navigation', executable='localization_guard', name='localization_guard',
             parameters=[parameters, {'initial_pose_enabled': enabled == 'true', **initial}], output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument('database_path', description='已有地图配套的 RTAB-Map 数据库绝对路径'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'map_localization.yaml'),
                              description='重定位及定位监督参数 YAML'),
        DeclareLaunchArgument('log_level', default_value='warn', description='定位节点日志级别'),
        DeclareLaunchArgument('initial_pose_enabled', default_value='false',
                              description='是否在双目实际处理、odom 与定位订阅就绪后发送明确提供的启动初始估计'),
        DeclareLaunchArgument('initial_x', default_value='0.0', description='启动初始估计的地图 X，单位米'),
        DeclareLaunchArgument('initial_y', default_value='0.0', description='启动初始估计的地图 Y，单位米'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='启动初始估计的地图朝向，单位弧度'),
        OpaqueFunction(function=start),
    ])

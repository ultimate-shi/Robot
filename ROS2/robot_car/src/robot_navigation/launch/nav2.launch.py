"""作用：编排静态或在线地图的 Nav2 与唯一 map→odom 发布者。

使用方法：ros2 launch robot_navigation nav2.launch.py；磁模式需设置 initial_yaw_source:=magnetic 与 magnetic_east_yaw_in_map。
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    share = get_package_share_directory('robot_navigation')
    map_yaml = LaunchConfiguration('map_yaml_file')
    nav2_params = LaunchConfiguration('nav2_params_file')
    overrides = LaunchConfiguration('nav2_overrides_file')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    initial_yaw_source = LaunchConfiguration('initial_yaw_source')
    magnetic_east_yaw = LaunchConfiguration('magnetic_east_yaw_in_map')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]

    def staggered(index, action):
        """在当前作用域解析间隔，再创建不会依赖子作用域的定时器。"""
        def create_timer(context):
            interval = float(LaunchConfiguration(
                'node_start_interval').perform(context))
            return [TimerAction(period=interval * index, actions=[action])]

        return OpaqueFunction(function=create_timer)

    actions = [
        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='/workspace/maps/studyroom/studyroom.yaml',
            description='Nav2 地图服务器加载的工作区二维占据栅格 YAML 文件路径'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2.yaml'),
            description='Nav2 各节点使用的基础参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'map_recovery_config_file',
            default_value=os.path.join(share, 'config', 'map_server_recovery.yaml'),
            description='静态地图服务未激活时的自动恢复参数 YAML'),
        DeclareLaunchArgument(
            'nav2_overrides_file',
            default_value=os.path.join(
                share, 'config', 'nav2.yaml'),
            description='叠加到基础配置上的 Nav2 参数 YAML；默认重复加载基础配置'),
        DeclareLaunchArgument(
            'use_map_server', default_value='true',
            description='是否加载静态地图；在线 RTAB-Map 导航时关闭'),
        DeclareLaunchArgument(
            'publish_map_to_odom', default_value='true',
            description='是否发布固定 map 到 odom；已有地图视觉定位时关闭以避免 TF 冲突'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='map 到 odom 初始静态变换的 X 坐标，单位为米'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='map 到 odom 初始静态变换的 Y 坐标，单位为米'),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='map 到 odom 初始静态变换的偏航角，单位为弧度'),
        DeclareLaunchArgument(
            'initial_yaw_source', default_value='manual',
            description='静态地图航向来源：manual 用 initial_yaw；magnetic 等待校准厂家姿态和 EKF TF'),
        DeclareLaunchArgument(
            'magnetic_east_yaw_in_map', default_value='',
            description='磁 ENU 东轴在 map 坐标中的偏航角，弧度；magnetic 模式必须显式设置'),
        DeclareLaunchArgument(
            'magnetic_map_config_file',
            default_value=os.path.join(share, 'config', 'magnetic_map_initializer.yaml'),
            description='磁航向初始化的超时和数据新鲜度参数 YAML 路径'),
        DeclareLaunchArgument(
            'node_start_interval', default_value='0.0',
            description='Nav2 相邻节点的错峰启动间隔，单位为秒'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='静态地图与 Nav2 各 ROS 节点的日志级别'),
    ]
    actions.append(staggered(0, LifecycleNode(
        package='nav2_map_server', executable='map_server',
        name='map_server', namespace='',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        arguments=ros_args, output='screen')))
    actions.append(staggered(1, Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        arguments=ros_args, output='screen')))
    def create_map_recovery(context):
        """定时器触发前解析子作用域中的恢复参数，避免上层延时 include 后丢失配置。"""
        config = LaunchConfiguration('map_recovery_config_file').perform(context)
        level = log_level.perform(context)
        interval = float(LaunchConfiguration('node_start_interval').perform(context))
        return [TimerAction(period=interval, actions=[Node(
            package='robot_navigation', executable='map_server_recovery',
            name='map_server_recovery', parameters=[config],
            arguments=['--ros-args', '--log-level', level], output='screen')])]

    actions.append(OpaqueFunction(function=create_map_recovery,
        condition=IfCondition(LaunchConfiguration('use_map_server'))))
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    definitions = [
        ('nav2_controller', 'controller_server', 'controller_server',
         remaps + [('cmd_vel', 'cmd_vel_nav_raw')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', remaps),
        ('nav2_planner', 'planner_server', 'planner_server', remaps),
        ('nav2_behaviors', 'behavior_server', 'behavior_server',
         remaps + [('cmd_vel', 'cmd_vel_nav_raw')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', remaps),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', remaps),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         remaps + [('cmd_vel', 'cmd_vel_nav_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav_smoothed')]),
    ]
    def navigation_actions(resolved):
        """磁航向模式仅在就绪后启动导航生命周期和运动节点。"""
        scheduled = []
        for index, (package, executable, name, node_remaps) in enumerate(
                definitions, start=3):
            scheduled.append(TimerAction(period=resolved['interval'] * index,
                actions=[Node(
                package=package, executable=executable, name=name,
                parameters=[resolved['nav2_params'], resolved['overrides'],
                            {'use_sim_time': False}],
                remappings=node_remaps, arguments=resolved['ros_args'],
                output='screen')]))
        scheduled.append(TimerAction(
            period=resolved['interval'] * (3 + len(definitions)), actions=[Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [item[2] for item in definitions],
            }], arguments=resolved['ros_args'], output='screen')]))
        return scheduled

    def select_initialization(context):
        """在线模式不占用 map→odom；磁模式在就绪事件前不启动 Nav2。"""
        resolved = {
            'interval': float(LaunchConfiguration('node_start_interval').perform(context)),
            'nav2_params': nav2_params.perform(context),
            'overrides': overrides.perform(context),
            'ros_args': ['--ros-args', '--log-level', log_level.perform(context)],
            'initial_x': initial_x.perform(context),
            'initial_y': initial_y.perform(context),
        }
        if resolved['interval'] < 0.0 or not math.isfinite(resolved['interval']):
            raise ValueError('node_start_interval 必须是非负有限值')
        source = initial_yaw_source.perform(context).strip().lower()
        if source not in ('manual', 'magnetic'):
            raise ValueError('initial_yaw_source 只能为 manual 或 magnetic')
        static_map = LaunchConfiguration('use_map_server').perform(context).lower()
        if static_map not in ('true', 'false'):
            raise ValueError('use_map_server 只能为 true 或 false')
        publish_map_tf = LaunchConfiguration('publish_map_to_odom').perform(context).lower()
        if publish_map_tf not in ('true', 'false'):
            raise ValueError('publish_map_to_odom 只能为 true 或 false')
        if source == 'magnetic' and publish_map_tf != 'true':
            raise ValueError('视觉定位发布 map 到 odom 时不能同时启用磁航向静态变换')
        if source == 'magnetic' and static_map != 'true':
            raise ValueError('magnetic 模式只能用于静态地图')
        if source == 'manual':
            result = []
            if static_map == 'true' and publish_map_tf == 'true':
                result.append(TimerAction(period=resolved['interval'] * 2,
                    actions=[Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name='static_tf_map', arguments=[
                        '--x', resolved['initial_x'], '--y', resolved['initial_y'], '--z', '0.0',
                        '--yaw', initial_yaw.perform(context), '--pitch', '0.0', '--roll', '0.0',
                        '--frame-id', 'map', '--child-frame-id', 'odom',
                        *resolved['ros_args'],
                    ], output='screen')]))
            return result + navigation_actions(resolved)
        raw_east = magnetic_east_yaw.perform(context).strip()
        try:
            east = float(raw_east)
        except ValueError as error:
            raise ValueError('magnetic_east_yaw_in_map 必须显式设置为弧度数值') from error
        if not math.isfinite(east):
            raise ValueError('magnetic_east_yaw_in_map 必须为有限数值')
        initializer = Node(
            package='robot_navigation', executable='magnetic_map_initializer',
            name='magnetic_map_initializer',
            parameters=[LaunchConfiguration('magnetic_map_config_file').perform(context), {
                'initial_x': float(resolved['initial_x']),
                'initial_y': float(resolved['initial_y']),
                'magnetic_east_yaw_in_map': east,
            }], arguments=resolved['ros_args'], output='screen')
        deferred_navigation = navigation_actions(resolved)
        marker = {'seen': False, 'buffer': b''}

        def on_initializer_output(event):
            marker['buffer'] = (marker['buffer'] + event.text)[-256:]
            if not marker['seen'] and b'MAGNETIC_MAP_READY' in marker['buffer']:
                marker['seen'] = True
                return deferred_navigation
            return []

        return [RegisterEventHandler(OnProcessIO(
            target_action=initializer, on_stdout=on_initializer_output)),
            TimerAction(period=resolved['interval'] * 2,
                        actions=[initializer])]

    actions.append(OpaqueFunction(function=select_initialization))
    return LaunchDescription(actions)

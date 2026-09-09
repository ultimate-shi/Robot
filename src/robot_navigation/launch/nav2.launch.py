"""使用方法：ros2 launch robot_navigation nav2.launch.py 启动工作区静态地图和完整 Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
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
            default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
            description='Nav2 各节点使用的基础参数 YAML 文件路径'),
        DeclareLaunchArgument(
            'nav2_overrides_file',
            default_value=os.path.join(
                share, 'config', 'nav2_empty_overrides.yaml'),
            description='叠加到基础配置上的 Nav2 参数覆盖 YAML 文件路径'),
        DeclareLaunchArgument(
            'use_map_server', default_value='true',
            description='是否加载静态地图并发布固定 map 到 odom；在线 RTAB-Map 导航时关闭'),
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
    actions.append(staggered(2, Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_tf_map',
        arguments=[
            '--x', initial_x, '--y', initial_y, '--z', '0.0',
            '--yaw', initial_yaw, '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
            '--ros-args', '--log-level', log_level,
        ],
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        output='screen')))
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
    for index, (package, executable, name, node_remaps) in enumerate(
            definitions, start=3):
        actions.append(staggered(index, Node(
            package=package, executable=executable, name=name,
            parameters=[nav2_params, overrides, {'use_sim_time': False}],
            remappings=node_remaps, arguments=ros_args, output='screen')))
    actions.append(staggered(3 + len(definitions), Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [item[2] for item in definitions],
        }],
        arguments=ros_args, output='screen')))
    return LaunchDescription(actions)

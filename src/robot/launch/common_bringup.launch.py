"""机器人模型、地图、底盘、Nav2 与 Foxglove 的公共启动入口。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    default_map = os.path.join(pkg_share, 'map', 'studyroom.yaml')
    default_nav2 = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_overrides = os.path.join(
        pkg_share, 'config', 'nav2_empty_overrides.yaml')
    terrain_params = os.path.join(
        pkg_share, 'config', 'terrain_params.yaml')
    manager_config = os.path.join(
        pkg_share, 'config', 'controller_manager.yaml')
    controller_config = os.path.join(
        pkg_share, 'config', 'controllers.yaml')

    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    map_yaml = LaunchConfiguration('map_yaml_file')
    nav2_params = LaunchConfiguration('nav2_params_file')
    nav2_overrides = LaunchConfiguration('nav2_overrides_file')
    chassis_cmd_topic = LaunchConfiguration('chassis_cmd_topic')
    log_level = LaunchConfiguration('log_level')
    foxglove_enabled = LaunchConfiguration('foxglove_enabled')
    foxglove_port = LaunchConfiguration('foxglove_port')
    ros_args = ['--ros-args', '--log-level', log_level]

    declarations = [
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('map_yaml_file', default_value=default_map),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2),
        DeclareLaunchArgument(
            'nav2_overrides_file', default_value=default_overrides),
        DeclareLaunchArgument(
            'chassis_cmd_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('log_level', default_value='warn'),
        DeclareLaunchArgument('foxglove_enabled', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
    ]

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]), value_type=str)
    }
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=ros_args,
        output='screen',
    )
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
        arguments=ros_args,
        output='screen',
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        arguments=ros_args,
        output='screen',
    )
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=[
            '--x', initial_x, '--y', initial_y, '--z', '0.0',
            '--yaw', initial_yaw, '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ] + ros_args,
        output='screen',
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description, manager_config, {'use_sim_time': False}],
        arguments=ros_args,
        output='screen',
    )
    controller_names = [
        'joint_state_broadcaster', 'steering_controller',
        'wheel_controller', 'lap_fr_position_controller',
        'lap_fl_position_controller', 'lap_rr_position_controller',
        'lap_rl_position_controller', 'shin_fr_position_controller',
        'shin_fl_position_controller', 'shin_rr_position_controller',
        'shin_rl_position_controller',
    ]
    spawners = [
        TimerAction(
            period=2.0 + index * 1.5,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                name=f'spawner_{name}',
                arguments=[name, '--param-file', controller_config] + ros_args,
                output='screen',
            )],
        )
        for index, name in enumerate(controller_names)
    ]
    zero_commands = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/wheel_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0,0.0,0.0,0.0]}',
                ],
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/steering_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0,0.0,0.0,0.0]}',
                ],
                output='screen',
            ),
        ],
    )
    chassis_feedback = Node(
        package='robot',
        executable='chassis_feedback_node',
        name='chassis_feedback_node',
        arguments=ros_args,
        output='screen',
    )
    nav_controller = Node(
        package='robot',
        executable='nav_controller_node',
        name='nav_controller_node',
        parameters=[terrain_params, {'use_sim_time': False}],
        arguments=ros_args,
        output='screen',
    )
    chassis_controller = Node(
        package='robot',
        executable='chassis_controller_node',
        name='chassis_controller',
        parameters=[terrain_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel', chassis_cmd_topic)],
        arguments=ros_args,
        output='screen',
    )

    nav2_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav2_cmd_remaps = nav2_remaps + [('cmd_vel', 'cmd_vel_nav_raw')]

    def nav2_node(package, executable, name, remappings):
        return Node(
            package=package,
            executable=executable,
            name=name,
            parameters=[
                nav2_params, nav2_overrides, {'use_sim_time': False}],
            remappings=remappings,
            arguments=ros_args,
            output='screen',
        )

    nav_nodes = [
        nav2_node(
            'nav2_controller', 'controller_server', 'controller_server',
            nav2_cmd_remaps),
        nav2_node(
            'nav2_smoother', 'smoother_server', 'smoother_server',
            nav2_remaps),
        nav2_node(
            'nav2_planner', 'planner_server', 'planner_server',
            nav2_remaps),
        nav2_node(
            'nav2_behaviors', 'behavior_server', 'behavior_server',
            nav2_cmd_remaps),
        nav2_node(
            'nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
            nav2_remaps),
        nav2_node(
            'nav2_waypoint_follower', 'waypoint_follower',
            'waypoint_follower', nav2_remaps),
        nav2_node(
            'nav2_velocity_smoother', 'velocity_smoother',
            'velocity_smoother',
            nav2_remaps + [
                ('cmd_vel', 'cmd_vel_nav_raw'),
                ('cmd_vel_smoothed', 'cmd_vel_nav_smoothed'),
            ]),
    ]
    nav_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server', 'smoother_server', 'planner_server',
                'behavior_server', 'velocity_smoother', 'bt_navigator',
                'waypoint_follower',
            ],
        }],
        arguments=ros_args,
        output='screen',
    )
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': ParameterValue(foxglove_port, value_type=int),
            'address': '0.0.0.0',
            'asset_uri_allowlist': ['package://robot/.*'],
            'allow_file_transfer': True,
            'send_buffer_limit': 1000000,
            'max_packet_messages': 100,
            'client_timeout_ms': 300000,
            'keep_alive_interval_ms': 5000,
        }],
        condition=IfCondition(foxglove_enabled),
        arguments=ros_args,
        output='screen',
    )

    launch_description = LaunchDescription(declarations + [
        robot_state_publisher,
        map_server,
        map_lifecycle,
        map_to_odom,
        controller_manager,
        zero_commands,
        chassis_feedback,
        nav_controller,
        chassis_controller,
        *nav_nodes,
        nav_lifecycle,
        foxglove,
    ])
    for spawner in spawners:
        launch_description.add_action(spawner)
    return launch_description

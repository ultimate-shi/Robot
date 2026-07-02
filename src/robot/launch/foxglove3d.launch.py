"""
foxglove3d.launch.py - 3D terrain-aware simulation launch file.

Based on foxglove.launch.py with these changes:
- Replaces chassis_controller with chassis_controller_3d (6DOF + terrain)
- Adds virtual_imu node (20Hz)
- Adds obstacle_avoidance node (filters cmd_vel)
- Remaps teleop output to /cmd_vel_raw (obstacle_avoidance publishes /cmd_vel)
- Loads terrain_params.yaml for all new nodes
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess
import subprocess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

subprocess.run(['pkill', '-f', 'foxglove_bridge'], capture_output=True)
subprocess.run(['sleep', '1'], capture_output=True)

# Global clock sync
os.environ['ROS_USE_SIM_TIME'] = '0'

# Auto-generate latest URDF
pkg_share = get_package_share_directory('robot')
xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
nav2_share = get_package_share_directory(
    'nav2_bringup'
)
NAV2_PARAMS = os.path.join(
    pkg_share,
    'config',
    'nav2_params.yaml'
)

result = subprocess.run(
    ['ros2', 'run', 'xacro', 'xacro', xacro_file, '-o', urdf_file],
    capture_output=True, text=True
)
if result.returncode != 0:
    print(f"xacro generation failed: {result.stderr}")
else:
    print(f"URDF generated: {urdf_file}")


def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    MAP_YAML_PATH = os.path.join(pkg_share, 'map', 'studyroom.yaml')
    TERRAIN_PARAMS = os.path.join(pkg_share, 'config', 'terrain_params.yaml')

    # Virtual environment paths
    VENV = "/home/shijiahao/ros2_pythonenv"

    venv_env = {
        "PATH": VENV + "/bin:" + os.environ["PATH"],
        "PYTHONPATH":
            VENV + "/lib/python3.12/site-packages:"
            + os.environ.get("PYTHONPATH", "")
    }

    # ==================== 1. URDF Robot Description ====================
    robot_description_content = Command(f'ros2 run xacro xacro {xacro_file}')
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    manager_config = PathJoinSubstitution([pkg_share, 'config', 'controller_manager.yaml'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # ==================== 2. Robot State Publisher ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ==================== 3. Map Server ====================
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'yaml_filename': MAP_YAML_PATH},
            {'use_sim_time': False}
        ]
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # Static TF: map -> odom
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        output="screen",
    )

    # ==================== 4. Point Cloud Publisher ====================
    publish_ply_node = Node(
        package='robot',
        executable='publish_ply',
        name='publish_ply',
        output='screen',
        additional_env=venv_env
    )

    # ==================== 5. Virtual Ultrasonic (original, 8 sensors) ====================
    virtual_ultrasonic_node = Node(
        package='robot',
        executable='virtual_ultrasonic',
        name='virtual_ultrasonic',
        output='screen',
        additional_env=venv_env
    )

    # ==================== 6. ros2_control ====================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, manager_config, {'use_sim_time': False}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    controller_names = [
        'joint_state_broadcaster', 'steering_controller', 'wheel_controller',
        'lap_fr_position_controller', 'lap_fl_position_controller',
        'lap_rr_position_controller', 'lap_rl_position_controller',
        'shin_fr_position_controller', 'shin_fl_position_controller',
        'shin_rr_position_controller', 'shin_rl_position_controller',
    ]
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{name}',
            arguments=[name, '--param-file', controller_config,
                       '--ros-args', '--log-level', 'warn'],
            output='screen'
        )
        for name in controller_names
    ]

    # Zero commands on startup
    zero_commands = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once',
                     '/wheel_controller/commands',
                     'std_msgs/msg/Float64MultiArray',
                     '{data: [0.0,0.0,0.0,0.0]}'],
                output='screen'),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once',
                     '/steering_controller/commands',
                     'std_msgs/msg/Float64MultiArray',
                     '{data: [0.0,0.0,0.0,0.0]}'],
                output='screen')
        ]
    )

    # ==================== 7. Chassis Feedback (original) ====================
    chassis_feedback_node = Node(
        package='robot',
        executable='chassis_feedback_node',
        name='chassis_feedback_node',
        output='screen'
    )

    # ==================== 8. Chassis Controller 3D (NEW - replaces original) ====================
    chassis_controller_3d_node = Node(
        package='robot',
        executable='chassis_controller_3d',
        name='chassis_controller',  # Same node name for compatibility
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        additional_env=venv_env
    )

    # ==================== 9. Virtual IMU (NEW) ====================
    virtual_imu_node = Node(
        package='robot',
        executable='virtual_imu',
        name='virtual_imu',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        additional_env=venv_env
    )

    # ==================== 10. Obstacle Avoidance (NEW) ====================
    obstacle_avoidance_node = Node(
        package='robot',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel'),
            ('/cmd_vel', '/cmd_vel_safe'),
        ]
    )

    # RVIZ
    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )


    pointcloud_obstacle_filter_node = Node(
        package='robot',
        executable='pointcloud_obstacle_filter',
        name='pointcloud_obstacle_filter',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        additional_env=venv_env
    )


    range_to_scan_node = Node(
        package='robot',
        executable='range_to_scan',
        name='range_to_scan',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )



    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_share,
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'map': MAP_YAML_PATH,
            'params_file': NAV2_PARAMS,
            'use_sim_time': 'false'
        }.items()
    )

    # ==================== 11. Foxglove Bridge ====================
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'asset_uri_allowlist': ['package://robot/.*'],
            # ✅ 关键：降低负载，防止虚拟网卡崩溃
            'allow_file_transfer': True,
            'send_buffer_limit': 1000000,   # 缩小缓冲区（原来50M太大了！）
            'max_packet_messages': 100,
            'client_timeout_ms': 300000,
            'keep_alive_interval_ms': 5000
        }],
        output='screen'
    )

    # ==================== Assemble Launch ====================
    ld = LaunchDescription([
        robot_state_publisher,
        map_server_node,
        map_lifecycle_manager,
        static_tf_map,
        publish_ply_node,
        virtual_ultrasonic_node,
        range_to_scan_node,
        pointcloud_obstacle_filter_node,
        controller_manager,
        zero_commands,
        chassis_feedback_node,
        chassis_controller_3d_node,
        virtual_imu_node,
        obstacle_avoidance_node,
        # rviz_node,
        foxglove_bridge,
        nav2_launch,
    ])

    for spawner in spawners:
        ld.add_action(spawner)

    return ld

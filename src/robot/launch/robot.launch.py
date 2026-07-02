"""
robot.launch 使用说明：
本文件是当前项目的主启动入口，用户明确要求以后重点关注该 launch。
它把机器人模型、地图、点云、虚拟超声波、点云过滤、Nav2、底盘控制、IMU、
安全避障和 Foxglove Bridge 组合到同一个 ROS 2 系统中。

启动后主要链路：
- robot_state_publisher 发布 /robot_description 和 URDF 产生的 TF。
- nav2_map_server 根据 map/studyroom.yaml 发布 /map。
- publish_ply 把 map/studyroom.ply 发布为 /pointcloud 和 /perception/points。
- pointcloud_obstacle_filter 把 /perception/points 过滤为 /nav/obstacle_points，供 Nav2 local_costmap 使用。
- virtual_ultrasonic 订阅 /perception/points 并结合 TF，发布 8 路 /ultrasonic/* 虚拟超声波距离。
- range_to_scan 把 8 路超声波拼成稀疏 /scan，主要用于调试或兼容 LaserScan 显示。
- obstacle_avoidance 把 /cmd_vel 过滤为 /cmd_vel_safe，底盘控制器只消费安全速度。
- chassis_controller_node 根据 /cmd_vel_safe 输出转向/轮速控制，并发布 /odom、odom->base_link TF。
- virtual_imu 根据 /odom 生成 /imu/data。
- foxglove_bridge 对外提供 Mac Foxglove 前端连接。

删除文件时的判断：
只要某个节点、配置、地图或模型资源在本文件中被引用，或被本文件启动的节点直接读取，
就不能简单删除；README 中有完整清单。
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
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
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

    # ==================== 8. Chassis Controller Node ====================
    chassis_controller_node = Node(
        package='robot',
        executable='chassis_controller_node',
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



    terrain_analyzer_node = Node(
        package='robot',
        executable='terrain_analyzer',
        name='terrain_analyzer',
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
        terrain_analyzer_node,
        controller_manager,
        zero_commands,
        chassis_feedback_node,
        chassis_controller_node,
        virtual_imu_node,
        obstacle_avoidance_node,
        # rviz_node,
        foxglove_bridge,
        nav2_launch,
    ])

    for spawner in spawners:
        ld.add_action(spawner)

    return ld

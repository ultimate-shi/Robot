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
- reverse_node 记录 /odom 历史轨迹，并在卡住恢复时输出 /cmd_vel_reverse。
- nav_controller_node 接收 Nav2 的 /cmd_vel_nav，在正常导航和原路回退之间仲裁并输出 /cmd_vel。
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
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
import subprocess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

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
NAV2_2D_PARAMS = os.path.join(
    pkg_share,
    'config',
    'nav2_2d_params.yaml'
)
BLANK_MAP_YAML = os.path.join(
    pkg_share,
    'map',
    'blank.yaml'
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
    DEFAULT_PLY_PATH = os.path.join(pkg_share, 'map', 'studyroom.ply')
    TERRAIN_PARAMS = os.path.join(pkg_share, 'config', 'terrain_params.yaml')

    # Virtual environment paths
    VENV = "/home/shijiahao/ros2_pythonenv"

    venv_env = {
        "PATH": VENV + "/bin:" + os.environ["PATH"],
        "PYTHONPATH":
            VENV + "/lib/python3.12/site-packages:"
            + os.environ.get("PYTHONPATH", "")
    }

    # 初始位姿只设置 map->odom 的平面偏移。
    # 高度、横滚和俯仰由 /perception/points 生成的 /terrain_status 动态写入 /odom，
    # 不在静态 TF 中写死 z，避免和凹凸地形点云冲突。
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    use_pointcloud_map = LaunchConfiguration('use_pointcloud_map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    ply_file = LaunchConfiguration('ply_file')
    # 地图文件现在作为显式启动参数传入，避免表达式展开失败导致 /map 不发布。
    # 默认仍使用 studyroom.yaml；空白地图或障碍物测试地图需要通过 map_yaml_file:=... 指定。
    map_yaml_file = LaunchConfiguration('map_yaml_file')

    declare_initial_x = DeclareLaunchArgument(
        'initial_x',
        default_value='0.0',
        description='Initial robot x position in the map frame'
    )
    declare_initial_y = DeclareLaunchArgument(
        'initial_y',
        default_value='0.0',
        description='Initial robot y position in the map frame'
    )
    declare_initial_yaw = DeclareLaunchArgument(
        'initial_yaw',
        default_value='0.0',
        description='Initial robot yaw in radians in the map frame'
    )
    # 默认进入 PLY 点云地图模式，让 Nav2、虚拟超声波和 obstacle_avoidance 一起工作，避免小车贴近障碍物时直接执行未过滤速度。
    declare_use_pointcloud_map = DeclareLaunchArgument(
        'use_pointcloud_map',
        default_value='true',
        description='Start PLY point cloud, terrain, ultrasonic and point cloud obstacle nodes'
    )
    declare_map_yaml_file = DeclareLaunchArgument(
        'map_yaml_file',
        default_value=MAP_YAML_PATH,
        description='Map YAML file used by map_server and Nav2'
    )
    declare_ply_file = DeclareLaunchArgument(
        'ply_file',
        default_value=DEFAULT_PLY_PATH,
        description='PLY point cloud file used by publish_ply when use_pointcloud_map is true'
    )

    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=NAV2_PARAMS,
        description='Nav2 parameter file. Default uses PLY point cloud obstacle parameters'
    )

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
            {'yaml_filename': map_yaml_file},
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
    # 通过移动 odom 原点设置小车在 map 中的初始平面位置。
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=[
            '--x', initial_x,
            '--y', initial_y,
            '--z', '0.0',
            '--yaw', initial_yaw,
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        output="screen",
    )

    # ==================== 4. Point Cloud Publisher ====================
    # PLY 地图模式下发布 studyroom.ply，后续点云过滤、虚拟超声波和地形分析都共用 /perception/points。
    publish_ply_node = Node(
        package='robot',
        executable='publish_ply',
        name='publish_ply',
        output='screen',
        parameters=[{'ply_file': ply_file}],
        additional_env=venv_env,
        condition=IfCondition(use_pointcloud_map)
    )

    # ==================== 5. Virtual Ultrasonic (original, 8 sensors) ====================
    # 虚拟 8 路超声波从 PLY 点云计算距离，obstacle_avoidance 依赖这些距离做近距离防撞。
    virtual_ultrasonic_node = Node(
        package='robot',
        executable='virtual_ultrasonic',
        name='virtual_ultrasonic',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        additional_env=venv_env,
        condition=IfCondition(use_pointcloud_map)
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

    # ==================== 8. Navigation Recovery Controller ====================
    # reverse_node 只记录 /odom 历史并输出 /cmd_vel_reverse，不直接取消 Nav2 目标。
    reverse_node = Node(
        package='robot',
        executable='reverse_node',
        name='reverse_node',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}]
    )

    # nav_controller_node 负责 Nav2 速度仲裁、卡住检测、回退请求和超过次数后的目标取消。
    # 正常链路：Nav2 -> /cmd_vel_nav -> nav_controller_node -> /cmd_vel。
    # 回退链路：reverse_node -> /cmd_vel_reverse -> nav_controller_node -> /cmd_vel。
    nav_controller_node = Node(
        package='robot',
        executable='nav_controller_node',
        name='nav_controller_node',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}]
    )

    # ==================== 8. Chassis Controller Node ====================
    # 空白地图调试模式下，底盘订阅 nav_controller_node 输出的 /cmd_vel，避免安全层持续发布零速度干扰底盘排查。
    chassis_controller_direct_node = Node(
        package='robot',
        executable='chassis_controller_node',
        name='chassis_controller',  # Same node name for compatibility
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        additional_env=venv_env,
        condition=UnlessCondition(use_pointcloud_map)
    )

    # PLY 点云模式下保留 /cmd_vel_nav -> nav_controller_node -> /cmd_vel -> obstacle_avoidance -> /cmd_vel_safe -> chassis 的安全链路。
    chassis_controller_safe_node = Node(
        package='robot',
        executable='chassis_controller_node',
        name='chassis_controller',  # Same node name for compatibility
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        additional_env=venv_env,
        condition=IfCondition(use_pointcloud_map)
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
    # 安全避障层读取 8 路 /ultrasonic/* 和 /terrain_status，将 nav_controller_node 输出的 /cmd_vel 过滤成 /cmd_vel_safe。
    obstacle_avoidance_node = Node(
        package='robot',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel'),
            ('/cmd_vel', '/cmd_vel_safe'),
        ],
        condition=IfCondition(use_pointcloud_map)
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


    # 点云障碍过滤输出 /nav/obstacle_points，供 Nav2 local_costmap 的 VoxelLayer 使用。
    pointcloud_obstacle_filter_node = Node(
        package='robot',
        executable='pointcloud_obstacle_filter',
        name='pointcloud_obstacle_filter',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        additional_env=venv_env,
        condition=IfCondition(use_pointcloud_map)
    )



    # 地形分析从 PLY 点云估计坡度、台阶和跌落风险，并发布 /terrain_status 给底盘和避障层。
    terrain_analyzer_node = Node(
        package='robot',
        executable='terrain_analyzer',
        name='terrain_analyzer',
        output='screen',
        parameters=[TERRAIN_PARAMS, {'use_sim_time': False}],
        additional_env=venv_env,
        condition=IfCondition(use_pointcloud_map)
    )

    # 将 8 路超声波转换成稀疏 /scan，便于 Foxglove/RViz 显示和兼容需要 LaserScan 的调试工具。
    range_to_scan_node = Node(
        package='robot',
        executable='range_to_scan',
        name='range_to_scan',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_pointcloud_map)
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
            'map': map_yaml_file,
            'params_file': nav2_params_file,
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
        declare_initial_x,
        declare_initial_y,
        declare_initial_yaw,
        declare_use_pointcloud_map,
        declare_map_yaml_file,
        declare_ply_file,
        declare_nav2_params_file,
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
        reverse_node,
        nav_controller_node,
        chassis_controller_direct_node,
        chassis_controller_safe_node,
        virtual_imu_node,
        obstacle_avoidance_node,
        # rviz_node,
        foxglove_bridge,
        nav2_launch,
    ])

    for spawner in spawners:
        ld.add_action(spawner)

    return ld

# 底盘控制启动文件
# 启动内容：
#  1. 机器人状态发布器
#  2. ros2_control
#  3. 底盘控制器
#  4. 反馈节点
#  5. RVIZ
#  6. 2D地图 3D点云


import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess

# 全局时钟同步
os.environ['ROS_USE_SIM_TIME'] = '0'

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')
    # 地图配置文件路径（固定你的路径）
    MAP_YAML_PATH = "/home/shijiahao/Downloads/studyroom.yaml"

    # ==================== 1. URDF 机器人描述（原有代码，无修改） ====================
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description_content = Command(f'ros2 run xacro xacro {xacro_file}')
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # 控制器配置文件
    manager_config = PathJoinSubstitution([pkg_share, 'config', 'controller_manager.yaml'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # ==================== 2. 机器人状态发布器（原有） ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ==================== 3. ros2_control 控制器管理器（原有） ====================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, manager_config, {'use_sim_time': False}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ==================== 4. 控制器Spawner列表（原有） ====================
    controller_names = [
        'joint_state_broadcaster',
        'steering_controller',
        'wheel_controller',
        'lap_fr_position_controller',
        'lap_fl_position_controller',
        'lap_rr_position_controller',
        'lap_rl_position_controller',
        'shin_fr_position_controller',
        'shin_fl_position_controller',
        'shin_rr_position_controller',
        'shin_rl_position_controller',
    ]
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{name}', 
            arguments=[name, '--param-file', controller_config, '--ros-args', '--log-level', 'warn'],
            output='screen'
        )
        for name in controller_names
    ]

    # ==================== 5. 初始零命令发布（原有） ====================
    zero_commands = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/wheel_controller/commands', 'std_msgs/msg/Float64MultiArray', '{data: [0.0, 0.0, 0.0, 0.0]}'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/steering_controller/commands', 'std_msgs/msg/Float64MultiArray', '{data: [0.0, 0.0, 0.0, 0.0]}'],
                output='screen'
            )
        ]
    )

    # ==================== 6. 底盘控制器 + 反馈节点（原有） ====================
    chassis_controller_node = Node(
        package='robot',
        executable='chassis_controller',
        name='chassis_controller',
        output='screen',
        parameters=[{'wheelbase': 0.4, 'track': 0.2, 'radius': 0.05, 'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'info']
    )
    chassis_feedback_node = Node(
        package='robot',
        executable='chassis_feedback_node',
        name='chassis_feedback_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # ==================== 7. RVIZ（原有） ====================
    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # ==================== 🔥 新增：2D地图服务（自动激活） ====================
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
    # 地图生命周期管理器（自动configure+activate）
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

    # ==================== 🔥 新增：静态TF map -> odom（坐标对齐） ====================
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # ==================== 🔥 新增：3D点云发布节点（publish_ply） ====================
    ply_publisher_node = Node(
        package='robot',
        executable='publish_ply',  # 严格按照你的要求
        name='ply_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ==================== 整合所有节点 ====================
    ld = LaunchDescription([
        # 基础机器人模型
        robot_state_publisher,
        controller_manager,
        # 地图相关
        map_server_node,
        map_lifecycle_manager,
        static_tf_map,
        ply_publisher_node,
        # 底盘控制
        zero_commands,
        chassis_feedback_node,
        chassis_controller_node,
        # RVIZ
        rviz_node
    ])
    
    # 添加所有控制器Spawner
    for spawner in spawners:
        ld.add_action(spawner)

    return ld
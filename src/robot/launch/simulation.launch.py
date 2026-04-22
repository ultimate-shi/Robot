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
    MAP_YAML_PATH = "/home/shijiahao/Downloads/studyroom.yaml"
    # ✅ 已定义：你的虚拟环境Python路径
    VENV_PYTHON = "/home/shijiahao/ros2_pythonenv/bin/python3"

    # ==================== 1. URDF 机器人描述 ====================
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description_content = Command(f'ros2 run xacro xacro {xacro_file}')
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    manager_config = PathJoinSubstitution([pkg_share, 'config', 'controller_manager.yaml'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # ==================== 2. 机器人状态发布器 ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # ==================== 地图服务器 ====================
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

    # 地图自动激活
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

    # 新语法静态TF，无警告
    static_tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--translation', '0', '0', '0',
            '--rotation', '0', '0', '0', '1'
        ]
    )

    # ✅ 终极修复：虚拟环境Python + 无ROS参数 + 路径正确
    ply_publisher_node = Node(
        package='robot',
        executable=VENV_PYTHON,
        name='ply_publisher',
        arguments=["/home/shijiahao/Downloads/ros2/robot_ws/src/robot/robot/publish_ply.py"],
        output='screen',
        parameters=[],
        ros_arguments=[],
        remappings=[],
    )

    # ==================== ros2_control ====================
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, manager_config, {'use_sim_time': False}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 控制器列表
    controller_names = [
        'joint_state_broadcaster','steering_controller','wheel_controller',
        'lap_fr_position_controller','lap_fl_position_controller','lap_rr_position_controller','lap_rl_position_controller',
        'shin_fr_position_controller','shin_fl_position_controller','shin_rr_position_controller','shin_rl_position_controller',
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

    # 零命令
    zero_commands = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '/wheel_controller/commands', 'std_msgs/msg/Float64MultiArray', '{data: [0.0,0.0,0.0,0.0]}'], output='screen'),
            ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '/steering_controller/commands', 'std_msgs/msg/Float64MultiArray', '{data: [0.0,0.0,0.0,0.0]}'], output='screen')
        ]
    )

    # 底盘节点
    chassis_controller_node = Node(
        package='robot', executable='chassis_controller', name='chassis_controller',
        output='screen', parameters=[{'wheelbase': 0.4, 'track': 0.2, 'radius': 0.05, 'use_sim_time': False}]
    )
    chassis_feedback_node = Node(
        package='robot', executable='chassis_feedback_node', name='chassis_feedback_node', output='screen'
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

    # ==================== 整合所有节点 ====================
    ld = LaunchDescription([
        robot_state_publisher,
        map_server_node,
        map_lifecycle_manager,
        static_tf_map,
        ply_publisher_node,
        controller_manager,
        zero_commands,
        chassis_feedback_node,
        chassis_controller_node,
        rviz_node
    ])

    for spawner in spawners:
        ld.add_action(spawner)

    return ld
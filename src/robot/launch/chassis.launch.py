import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction, ExecuteProcess

# 全局时钟同步
os.environ['ROS_USE_SIM_TIME'] = '0'

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')

    # URDF —— 🔥 修复1：修正 xacro 命令格式，解决路径报错
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description_content = Command(f'ros2 run xacro xacro {xacro_file}')
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }


    # config
    manager_config = PathJoinSubstitution([pkg_share, 'config', 'controller_manager.yaml'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    

    # ros2_control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, manager_config, {'use_sim_time': False}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'warn']
    )

    # 需要 spawn 的控制器列表
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

    # 生成 spawner
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

    # -----------------------------
    # 发布初始零命令（防止启动抖动）
    # -----------------------------
    zero_commands = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/wheel_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0, 0.0, 0.0, 0.0]}'
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/steering_controller/commands',
                    'std_msgs/msg/Float64MultiArray',
                    '{data: [0.0, 0.0, 0.0, 0.0]}'
                ],
                output='screen'
            )
        ]
    )


    # 🔥 修复2：删除重复启动的 four_wis_controller + 延时指令（分离根源）
    # 删除了 zero_wheel_cmd、zero_steer_cmd、zero_commands

    # FourWIS 控制器（只启动这一次！）
    four_wis_controller_node = Node(
        package='robot',
        executable='four_wis_controller',
        name='four_wis_controller',
        output='screen',
        parameters=[{'wheelbase': 0.4, 'track': 0.2, 'radius': 0.05, 'use_sim_time': False}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # 反馈节点
    wheel_feedback_node = Node(
        package='robot',
        executable='wheel_feedback_node',
        name='wheel_feedback_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # RVIZ
    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        parameters=[robot_description, {'use_sim_time': False}]
    )


    ld = LaunchDescription([
        robot_state_publisher, 
        controller_manager,
        zero_commands,
        # 反馈节点 + 控制器 同时启动
        wheel_feedback_node,
        four_wis_controller_node,
        rviz_node
    ])
    
    for spawner in spawners:
        ld.add_action(spawner)

    return ld
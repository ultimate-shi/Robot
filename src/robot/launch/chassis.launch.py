import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('robot')

    # URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    robot_description_content = Command([
        FindExecutable(name='ros2'), ' run ', 'xacro', ' xacro ', xacro_file
    ])
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
        parameters=[robot_description],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    

    # ros2_control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, manager_config],
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

    # 用循环生成 spawner Node
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

    # FourWISController Python 节点
    four_wis_controller_node = Node(
        package='robot',
        executable='four_wis_controller',
        name='four_wis_controller',
        output='screen',
        parameters=[{'wheelbase': 0.4, 'track': 0.2, 'radius': 0.05}],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    rviz_config = os.path.join(pkg_share, 'config', 'view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config,
            '--ros-args', '--log-level', 'warn'
        ],
        parameters=[robot_description]
    )

    ld = LaunchDescription([
        robot_state_publisher, 
        controller_manager, 
        four_wis_controller_node,
        rviz_node
    ])
    
    for spawner in spawners:
        ld.add_action(spawner)

    return ld
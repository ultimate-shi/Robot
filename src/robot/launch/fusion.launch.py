import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 1. 基础配置：包路径、文件路径
    pkg_name = 'robot'
    pkg_path = get_package_share_directory(pkg_name)
    
    # URDF文件路径
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    # 控制器配置文件路径
    controller_config = os.path.join(pkg_path, 'config', 'controller.yaml')
    # RViz配置文件路径
    rviz_config_path = os.path.join(pkg_path, 'config', 'robot_view.rviz')

    # 2. 生成机器人描述（修复Jazzy的xacro调用）
    robot_description_content = Command(
        [
            FindExecutable(name='ros2'),
            ' run ',
            'xacro',
            ' xacro ',
            xacro_file
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 3. 启动Gazebo仿真环境（官方launch文件）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        # Gazebo启动参数：空世界
        launch_arguments={'world': os.path.join(pkg_path, 'worlds', 'empty.world')}.items()
    )

    # 4. 加载Gazebo中的机器人模型
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',          # 模型名称
            '-topic', 'robot_description' # 机器人描述话题
        ],
        output='screen'
    )

    # 5. 机器人状态发布器（核心，所有模块依赖）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 6. 关节状态GUI控制器（可选，手动调试关节）
    joint_state_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[
            robot_description,
            {'use_gui': True},
            {'rate': 100}
        ]
    )

    # 7. Gazebo控制器管理器（命名空间：/robot/gazebo）
    gazebo_controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='/robot/gazebo',  # 仿真命名空间
        parameters=[
            robot_description,
            controller_config       # 统一的控制器配置文件
        ],
        output='screen'
    )

    # 8. 物理机器人控制器管理器（命名空间：/robot/physical）
    physical_controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='/robot/physical', # 物理命名空间
        parameters=[
            robot_description,
            controller_config       # 同一套控制器配置文件
        ],
        output='screen'
    )

    # 9. 加载Gazebo控制器（延迟2秒，等控制器管理器启动）
    load_gazebo_controllers = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                namespace='/robot/gazebo',
                arguments=[
                    # 关节状态广播器（必需）
                    'joint_state_broadcaster',
                    # 大腿控制器
                    'lap_rf_position_controller',
                    'lap_lf_position_controller',
                    'lap_rr_position_controller',
                    'lap_lr_position_controller',
                    # 小腿控制器
                    'shin_rf_position_controller',
                    'shin_lf_position_controller',
                    'shin_rr_position_controller',
                    'shin_lr_position_controller',
                    # 转向控制器
                    'motor_rf_position_controller',
                    'motor_lf_position_controller',
                    'motor_rr_position_controller',
                    'motor_lr_position_controller',
                    # 轮子控制器
                    'wheel_rf_velocity_controller',
                    'wheel_lf_velocity_controller',
                    'wheel_rr_velocity_controller',
                    'wheel_lr_velocity_controller',
                    # 指定控制器管理器节点
                    '--controller-manager', '/robot/gazebo/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # 10. 加载物理机器人控制器（延迟3秒，等Gazebo控制器加载完成）
    load_physical_controllers = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                namespace='/robot/physical',
                arguments=[
                    'joint_state_broadcaster',
                    # 所有控制器（与Gazebo一致）
                    'lap_rf_position_controller',
                    'lap_lf_position_controller',
                    'lap_rr_position_controller',
                    'lap_lr_position_controller',
                    'shin_rf_position_controller',
                    'shin_lf_position_controller',
                    'shin_rr_position_controller',
                    'shin_lr_position_controller',
                    'motor_rf_position_controller',
                    'motor_lf_position_controller',
                    'motor_rr_position_controller',
                    'motor_lr_position_controller',
                    'wheel_rf_velocity_controller',
                    'wheel_lf_velocity_controller',
                    'wheel_rr_velocity_controller',
                    'wheel_lr_velocity_controller',
                    '--controller-manager', '/robot/physical/controller_manager'
                ],
                output='screen'
            )
        ]
    )

    # 11. 指令转发节点（核心：同步Gazebo→物理指令）
    cmd_forwarder_node = Node(
        package=pkg_name,          # 你的包名
        executable='cmd_forwarder_node',  # 指令转发节点的可执行文件名
        name='cmd_forwarder_node',
        output='screen',
        # 可选：开启debug日志（调试时用）
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    # 12. 状态融合节点（核心：合并仿真/物理状态）
    state_fusion_node = Node(
        package=pkg_name,          # 你的包名
        executable='state_fusion_node',   # 状态融合节点的可执行文件名
        name='state_fusion_node',
        output='screen',
        # 可选：开启debug日志
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    # 13. RViz2（延迟4秒，等所有模块启动）
    rviz_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
                output='screen'
            )
        ]
    )

    # 组装所有启动项（按依赖顺序）
    return LaunchDescription([
        # 第一步：启动Gazebo环境
        gazebo_launch,
        # 第二步：启动核心状态发布器
        robot_state_publisher_node,
        # 第三步：加载Gazebo机器人模型
        spawn_entity_node,
        # 第四步：启动控制器管理器（仿真+物理）
        gazebo_controller_manager_node,
        physical_controller_manager_node,
        # 第五步：加载控制器（延迟执行）
        load_gazebo_controllers,
        load_physical_controllers,
        # 第六步：启动指令转发、状态融合节点
        cmd_forwarder_node,
        state_fusion_node,
        # 第七步：启动GUI和RViz（调试用）
        joint_state_gui_node,
        rviz_node
    ])
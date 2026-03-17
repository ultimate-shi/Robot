import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    TimerAction, IncludeLaunchDescription, SetEnvironmentVariable,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 获取包路径和文件路径
    pkg_name = 'robot'
    pkg_path = get_package_share_directory(pkg_name)  # robot包的绝对路径
    
    # 关键修复：设置Gazebo Harmonic的资源搜索路径
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            pkg_path,
            os.pathsep,
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )
    
    # 2. 定义文件路径（注意：你的配置文件是controller.yaml，需确保和实际文件名一致）
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    world_file = os.path.join(pkg_path, 'world', 'my.sdf')
    controller_config = os.path.join(pkg_path, 'config', 'controller.yaml')
    
    # 3. 处理xacro文件
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
    
    # 4. 启动Gazebo Sim
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_file, ' -v 4'],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # 5. 控制器管理器节点（适配Jazzy+命名空间）
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_config,
            {'use_sim_time': True}  # 关键：匹配Gazebo仿真时间
        ],
        output="screen",
        # 确保节点依赖Gazebo启动完成
        remappings=[
            ('/joint_states', '/robot/joint_states')
        ]
    )

    # ========== 核心添加：ros_gz_bridge 桥接节点 ==========
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 桥接关节状态（Gz → ROS 2）
            "/world/default/model/robot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # 桥接控制指令（ROS 2 → Gz）
            "/lap_rf_position_controller/commands@std_msgs/msg/Float64[gz.msgs.Double",
            # 按需添加其他桥接话题（如轮子速度指令）
            "/wheel_rf_velocity_controller/commands@std_msgs/msg/Float64[gz.msgs.Double"
        ],
        remappings=[
            ("/world/default/model/robot/joint_states", "/robot/joint_states")
        ],
        output="screen"
    )

    # 4. 启动Gz仿真（Jazzy中用ros_gz_sim）
    gz_sim = Node(
        package="ros_gz_sim",
        executable="gz_sim",
        arguments=["-r", "my.sdf"],  # 启动空世界，加载机器人模型
        output="screen"
    )

    # 5. 加载机器人模型到Gz
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "robot",
            "-z", "0.1"  # 离地高度
        ],
        output="screen"
    )

    # 6. 定义加载独立控制器的通用函数（核心修改）
    def load_controller(name):
        return TimerAction(
            period=5.0,  # 延迟5秒，确保Gazebo和控制器管理器完全启动
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        name,
                        "--controller-manager", "/robot/controller_manager",
                        "--activate",  # 自动激活控制器（关键！）
                        "--controller-manager-timeout", "10"  # 超时时间，避免卡死
                    ],
                    output="screen"
                )
            ]
        )
    
    # 7. 定义所有需要加载的独立控制器（和你的controller.yaml完全匹配）
    controllers = [
        # 基础：关节状态广播器
        "joint_state_broadcaster",
        # 大腿位置控制器
        "lap_rf_position_controller",
        "lap_lf_position_controller",
        "lap_rr_position_controller",
        "lap_lr_position_controller",
        # 小腿位置控制器
        "shin_rf_position_controller",
        "shin_lf_position_controller",
        "shin_rr_position_controller",
        "shin_lr_position_controller",
        # 转向位置控制器
        "motor_rf_position_controller",
        "motor_lf_position_controller",
        "motor_rr_position_controller",
        "motor_lr_position_controller",
        # 轮子速度控制器
        "wheel_rf_velocity_controller",
        "wheel_lf_velocity_controller",
        "wheel_rr_velocity_controller",
        "wheel_lr_velocity_controller"
    ]
    
    # 8. 生成所有控制器的加载动作
    load_controller_actions = [load_controller(name) for name in controllers]

    # 9. 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}  # 关键：匹配仿真时间
        ]
    )
    
    # 10. 关节状态GUI
    joint_state_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[
            robot_description, 
            {'use_gui': True}, 
            {'rate': 100},
            {'use_sim_time': True}
        ]
    )
    
    # 11. 生成机器人模型（延迟增加到3秒，确保控制器管理器先启动）
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'robot', 
                    '-topic', '/robot_description', 
                    '-x', '0.0', '-y', '0.0', '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )
    
    # 12. ROS2与GZ桥接（补充关节状态桥接）
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 补充关节状态桥接，确保GUI能获取关节数据
            '/robot/joint_states@sensor_msgs/msg/JointState[gz.msgs.JointState'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 13. 关节转发节点（GUI滑块→独立控制器指令）
    joint_forwarder_node = Node(
        package='robot',
        executable='joint_forwarder_node',  # 需确保scripts目录下有该文件
        name='joint_forwarder',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # 延迟启动，确保控制器都加载完成
        remappings=[
            ('/joint_states', '/robot/joint_states')
        ]
    )

    # 组装启动项（严格按依赖顺序）
    return LaunchDescription([
        # 第一步：设置环境变量
        gz_resource_path,
        # 第二步：启动Gazebo
        gz_sim,
        # 第三步：核心节点（状态发布、GUI）
        robot_state_publisher_node,
        joint_state_gui_node,
        # 第四步：桥接和模型生成
        gz_bridge,
        spawn_entity,
        # 第五步：控制器相关（核心）
        controller_manager,
        # 第六步：加载所有独立控制器
        *load_controller_actions,
        # 第七步：启动转发节点（GUI联动）
        joint_forwarder_node
    ])
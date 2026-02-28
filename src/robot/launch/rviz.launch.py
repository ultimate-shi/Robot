import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 获取包路径和文件路径
    pkg_name = 'robot'
    pkg_path = get_package_share_directory(pkg_name)
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.xacro')
    
    # 2. 修复：ROS2 Jazzy正确的xacro调用方式
    # 使用FindExecutable找到xacro的可执行文件，通过ros2 run调用
    robot_description_content = Command(
        [
            FindExecutable(name='ros2'),  # 找到ros2可执行文件
            ' run ',
            'xacro',                     # xacro包名
            ' xacro ',                   # xacro可执行文件名
            xacro_file                   # 要处理的xacro文件路径
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # 3. 启动机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 4. 启动joint_state_publisher_gui（GUI控制器）
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
    
    # 5. 启动RViz（延迟2秒）
    rviz_config_path = os.path.join(pkg_path, 'config', 'robot_view.rviz')
    rviz_node = TimerAction(
        period=2.0,
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

    # 组装所有节点
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_gui_node,
        rviz_node
    ])
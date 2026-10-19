"""使用方法：ros2 launch robot_control wheel_motion.launch.py 启动四轮定距 Action。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_control'),
        'config', 'wheel_motion.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=config,
            description='四轮定距速度、容差和超时参数 YAML 路径'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='定距 Action 节点日志级别'),
        DeclareLaunchArgument(
            'require_encoder_service', default_value='true',
            description='是否必须有实机编码器 T 服务；虚拟模式设为 false'),
        Node(
            package='robot_control', executable='wheel_motion_server_node',
            name='wheel_motion_server', parameters=[LaunchConfiguration('config_file'), {
                'require_encoder_service': ParameterValue(
                    LaunchConfiguration('require_encoder_service'), value_type=bool)}],
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')], output='screen'),
    ])

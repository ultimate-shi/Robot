"""使用方法：ros2 launch robot_description joint_states.launch.py 发布模型默认关节状态。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rate', default_value='10',
            description='默认关节状态的发布频率，单位为 Hz'),
        DeclareLaunchArgument(
            'publish_default_positions', default_value='true',
            description='是否为没有反馈的模型关节发布默认零位'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='默认关节状态发布节点的 ROS 日志级别'),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{
                'rate': ParameterValue(
                    LaunchConfiguration('rate'), value_type=int),
                'publish_default_positions': ParameterValue(
                    LaunchConfiguration('publish_default_positions'),
                    value_type=bool),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

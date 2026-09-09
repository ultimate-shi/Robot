"""使用方法：ros2 launch robot_navigation ply_map.launch.py 发布保存的 PLY 环境点云。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ply_file',
            default_value='/workspace/maps/studyroom/studyroom.ply',
            description='需要发布的环境 PLY 点云文件路径'),
        DeclareLaunchArgument(
            'allow_fallback', default_value='true',
            description='PLY 文件不可用时是否允许节点使用配置的降级行为'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='PLY 环境点云发布节点的 ROS 日志级别'),
        Node(
            package='robot_navigation', executable='publish_ply',
            parameters=[{
                'ply_file': LaunchConfiguration('ply_file'),
                'allow_fallback': ParameterValue(
                    LaunchConfiguration('allow_fallback'), value_type=bool),
            }],
            arguments=[
                '--ros-args', '--log-level',
                LaunchConfiguration('log_level'),
            ],
            output='screen'),
    ])

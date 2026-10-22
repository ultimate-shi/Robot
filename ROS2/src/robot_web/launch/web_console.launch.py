"""作用：启动机器人网页控制台；使用方法：ros2 launch robot_web web_console.launch.py。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_web'),
                          'config', 'web_console.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config,
                              description='网页控制台的 ROS 接口与手动直达参数 YAML 路径'),
        DeclareLaunchArgument('mode', default_value='virtual',
                              description='机器人数据源模式：real 或 virtual'),
        DeclareLaunchArgument('static_map', default_value='false',
                              description='地图是否由静态地图服务器发布一次后长期有效'),
        DeclareLaunchArgument('log_level', default_value='warn',
                              description='网页控制台节点的 ROS 日志级别'),
        Node(package='robot_web', executable='robot_web_console',
             parameters=[LaunchConfiguration('config_file'),
                         {'mode': LaunchConfiguration('mode'),
                          'static_map': ParameterValue(
                              LaunchConfiguration('static_map'),
                              value_type=bool)}],
             arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
             output='screen'),
    ])

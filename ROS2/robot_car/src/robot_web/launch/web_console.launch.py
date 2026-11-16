"""作用：启动机器人网页控制台；使用方法：ros2 launch robot_web web_console.launch.py。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('robot_web'),
                          'config', 'web_console.yaml')
    def start(context):
        source = LaunchConfiguration('camera_source').perform(context)
        if source not in ('stereo', 'astra'):
            raise ValueError('camera_source 只能为 stereo 或 astra')
        overrides = {'camera_source': source,
                     'mode': LaunchConfiguration('mode').perform(context),
                     'static_map': LaunchConfiguration('static_map').perform(context).lower() == 'true'}
        leg_length = LaunchConfiguration('leg_length').perform(context).strip()
        if leg_length:
            overrides['leg_length_mm'] = float(leg_length) * 1000.0
        if source == 'astra':
            overrides.update(left_image_topic='/camera/rgb/image_rect',
                             right_image_topic='/camera/rgb/image_rect',
                             depth_image_topic='/camera/depth/image_visual')
        return [Node(package='robot_web', executable='robot_web_console',
                     parameters=[LaunchConfiguration('config_file').perform(context), overrides],
                     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level').perform(context)], output='screen')]
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config,
                              description='网页控制台的 ROS 接口与手动直达参数 YAML 路径'),
        DeclareLaunchArgument('leg_length', default_value='',
                              description='与整车共用的主动腿长度，单位米；空值沿用 YAML 的 leg_length_mm'),
        DeclareLaunchArgument('camera_source', default_value='stereo', description='stereo 双目画面或 astra 单路彩色/深度画面'),
        DeclareLaunchArgument('mode', default_value='virtual',
                              description='机器人数据源模式：real 或 virtual'),
        DeclareLaunchArgument('static_map', default_value='false',
                              description='地图是否由静态地图服务器发布一次后长期有效'),
        DeclareLaunchArgument('log_level', default_value='warn',
                              description='网页控制台节点的 ROS 日志级别'),
        OpaqueFunction(function=start),
    ])

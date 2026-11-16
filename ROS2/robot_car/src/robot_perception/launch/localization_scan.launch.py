"""作用：选择双目或预留 Astra 点云输入；使用方法：cloud_source:=stereo|astra，可覆盖 cloud_topic 和 sensor_frame。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def start(context):
        source = LaunchConfiguration('cloud_source').perform(context)
        if source not in ('stereo', 'astra'):
            raise ValueError('cloud_source 只能为 stereo 或 astra')
        defaults = {'stereo': ('/stereo/points2', 'stereo_left_camera_frame'),
                    'astra': ('/camera/points', 'astra_color_optical_frame')}
        topic = LaunchConfiguration('cloud_topic').perform(context).strip() or defaults[source][0]
        frame = LaunchConfiguration('sensor_frame').perform(context).strip() or defaults[source][1]
        return [Node(package='robot_perception', executable='localization_scan', name='localization_scan',
                     parameters=[LaunchConfiguration('config_file').perform(context),
                                 {'input_topic': topic, 'sensor_frame': frame}],
                     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level').perform(context)],
                     output='screen')]

    return LaunchDescription([
        DeclareLaunchArgument('cloud_source', default_value='stereo', description='定位点云来源：stereo 或 astra；Astra 驱动和安装 TF 由外部提供'),
        DeclareLaunchArgument('cloud_topic', default_value='', description='定位 PointCloud2 话题覆盖；留空使用所选来源默认值'),
        DeclareLaunchArgument('sensor_frame', default_value='', description='真实深度光心坐标系覆盖；留空使用所选来源默认值'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(get_package_share_directory('robot_perception'), 'config', 'localization_scan.yaml'), description='定位扫描过滤和时间保护参数 YAML'),
        DeclareLaunchArgument('log_level', default_value='warn', description='扫描节点日志级别'),
        OpaqueFunction(function=start),
    ])

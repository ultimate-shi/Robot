"""作用：发布 Astra 头部安装 TF；用法：默认沿用用户确认的旧双目中心，astra_mount.yaml 可按实测校正或禁用。"""
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def mount_arguments(config):
    mount = config['mount']
    if mount['enabled'] is not True:
        return None
    xyz, rpy = mount['xyz'], mount['rpy']
    if (len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(float(v)) for v in xyz+rpy) or
            not mount['parent_frame'] or not mount['child_frame'] or mount['parent_frame'] == mount['child_frame']):
        raise ValueError('Astra 安装标定必须包含不同父子坐标系和六个有限实测值')
    return [item for key, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), xyz+rpy)
            for item in ('--'+key, str(value))]+['--frame-id', mount['parent_frame'], '--child-frame-id', mount['child_frame']]


def generate_launch_description():
    def start(context):
        with open(LaunchConfiguration('config_file').perform(context), encoding='utf-8') as stream:
            arguments = mount_arguments(yaml.safe_load(stream))
        if arguments is None:
            return [LogInfo(msg='Astra 安装 TF 发布已禁用；请由外部提供 head_pitch_link → astra_link，视觉定位和导航需要有效 TF')]
        return [Node(package='tf2_ros', executable='static_transform_publisher', name='astra_mount', arguments=arguments)]
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(get_package_share_directory('robot_perception'), 'config', 'astra_mount.yaml'), description='Astra 安装标定 YAML；外部已发布相同 TF 时保持 disabled'),
        OpaqueFunction(function=start),
    ])

"""使用方法：在工作区执行 colcon build 安装感知节点、配置和标定文件。"""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_perception'
camera_profiles = [(os.path.join('share', package_name, os.path.dirname(path)), [path])
                   for path in glob('config/cameras/*/*.yaml')]
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        *camera_profiles,
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人双目、语义、地形和虚拟传感器感知', license='TODO: License declaration',
    entry_points={'console_scripts': [
        'stereo_splitter_node = robot_perception.stereo.stereo_splitter:main',
        'stereo_pair_throttle_node = robot_perception.stereo.stereo_pair_throttle:main',
        'stereo_depth_node = robot_perception.stereo.stereo_depth:main',
        'stereo_pointcloud_filter = robot_perception.stereo.stereo_pointcloud_filter:main',
        'pointcloud_obstacle_filter = robot_perception.pointcloud_obstacle_filter:main',
        'terrain_analyzer = robot_perception.terrain.terrain_analyzer:main',
        'semantic_perception = robot_perception.semantic.semantic_perception:main',
        'snapshot_local_observer = robot_perception.snapshot_local_observer:main',
        'virtual_imu = robot_perception.virtual_sensors.virtual_imu:main',
        'virtual_ultrasonic = robot_perception.virtual_sensors.virtual_ultrasonic:main',
        'range_to_scan = robot_perception.virtual_sensors.range_to_scan:main',
        'acceptance_sampler = robot_perception.diagnostics.acceptance_sampler:main',
        'stereo_pipeline_benchmark = robot_perception.diagnostics.stereo_pipeline_benchmark:main',
    ]},
)

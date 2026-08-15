"""使用方法：在工作区执行 colcon build 安装导航节点、地图、配置和 launch。"""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_navigation'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*.*')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.py')),
    ],
    install_requires=['setuptools', 'plyfile>=1.0,<2'], zip_safe=True,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人 RTAB-Map、Nav2、目标停靠和路径预演', license='TODO: License declaration',
    entry_points={'console_scripts': [
        'mapping_snapshot_manager = robot_navigation.mapping.snapshot_manager:main',
        'publish_ply = robot_navigation.mapping.ply_publisher:main',
        'goal_manager = robot_navigation.goal_manager:main',
        'mission_planner = robot_navigation.mission_planner:main',
        'brain_mission = robot_navigation.mission_planner:main',
    ]},
)

"""使用方法：在工作区执行 colcon build 安装底盘控制节点和配置。"""
import os
from glob import glob
from setuptools import setup

package_name = 'robot_control'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人四轮转向底盘、速度门控与安全控制',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'chassis_controller_node = robot_control.chassis_controller:main',
        'chassis_feedback_node = robot_control.chassis_feedback:main',
        'nav_controller_node = robot_control.nav_velocity_gate:main',
        'obstacle_avoidance = robot_control.obstacle_avoidance:main',
    ]},
)

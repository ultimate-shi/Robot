"""作用：安装机械臂 Python 包及资源；使用方法：在独立工作区运行 colcon build。"""
from glob import glob
from setuptools import find_packages, setup

setup(
    name='robot_arm', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_arm']),
        ('share/robot_arm', ['package.xml']),
        ('share/robot_arm/launch', glob('launch/*.launch.py')),
        ('share/robot_arm/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机械臂独立功能包骨架', license='TODO: License declaration',
    entry_points={'console_scripts': []},
)

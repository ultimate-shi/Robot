"""使用方法：在工作区执行 colcon build 安装本地大脑、网页和启动配置。"""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_brain'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
    ],
    install_requires=[
        'setuptools', 'fastapi>=0.110,<1', 'uvicorn>=0.29,<1',
        'websockets>=12,<16',
    ],
    zip_safe=True, maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人本地大模型交互、多用户控制权和任务编排', license='TODO: License declaration',
    entry_points={'console_scripts': [
        'brain_web = robot_brain.web_server:main',
    ]},
)

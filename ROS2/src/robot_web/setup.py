"""作用：安装 9100 网页入口与 ROS 控制台；使用方法：colcon build --packages-select robot_web。"""

from glob import glob
import os

from setuptools import setup


package_name = 'robot_web'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools', 'PyYAML', 'fastapi', 'uvicorn'],
    zip_safe=False,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='统一机器人网页、控制台和 API 入口', license='TODO: License declaration',
    entry_points={'console_scripts': [
        'robot_web = robot_web.server:main',
        'robot_web_console = robot_web.console:main',
    ]},
)

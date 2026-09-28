"""使用方法：colcon build 安装 robot_main 的统一配置和 launch."""
from glob import glob
import os

from setuptools import setup

package_name = 'robot_main'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人统一整车启动入口', license='TODO: License declaration')

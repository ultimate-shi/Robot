"""使用方法：colcon build 安装机器人日志采集与查询组件。"""
from glob import glob
import os

from setuptools import setup


package_name = 'robot_logging'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={package_name: ['web/index.html']},
    install_requires=['setuptools'],
    extras_require={'web': ['PyYAML', 'fastapi', 'uvicorn']},
    zip_safe=False,
    maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人 MCAP 日志采集、索引和查询',
    license='TODO: License declaration',
    entry_points={'console_scripts': [
        'log_manager = robot_logging.manager:main',
        'log_indexer = robot_logging.indexer:main',
    ]},
)

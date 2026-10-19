"""作用：安装本地大脑的任务策略组件；使用方法：在工作区执行 colcon build。"""
from setuptools import find_packages, setup

package_name = 'robot_brain'
setup(
    name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True, maintainer='shijiahao', maintainer_email='shijiahao@todo.todo',
    description='机器人 Qwen 决策核心与任务策略组件', license='TODO: License declaration',
    entry_points={'console_scripts': []},
)

from setuptools import find_packages, setup
import os  # 关键：确保此行存在且无注释
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加launch文件安装配置
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # 若有urdf和config目录，也建议添加
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.xacro')) + 
            glob(os.path.join('urdf', '*.urdf'))),  # 同时安装 xacro 和 urdf
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), 
            glob(os.path.join('meshes', '*.*'))),
        (os.path.join('share', package_name, 'world'), 
            glob(os.path.join('world', '*.sdf'))),
        (os.path.join('share', package_name, 'map'), 
            glob(os.path.join('map', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shijiahao',
    maintainer_email='shijiahao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_controller = robot.chassis_controller_node:main',
            'chassis_feedback_node = robot.chassis_feedback_node:main',
            'teleop_joy = robot.teleop_joy_node:main',
            'publish_ply = robot.publish_ply:main',
            'virtual_ultrasonic = robot.virtual_ultrasonic:main',
            'ultrasonic_listener = robot.ultrasonic_listener:main',
            'chassis_controller_3d = robot.chassis_controller_3d:main',
            'virtual_imu = robot.virtual_imu_node:main',
            'obstacle_avoidance = robot.obstacle_avoidance_node:main',
        ],
    },
)

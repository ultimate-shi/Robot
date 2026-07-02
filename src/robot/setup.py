"""
foxglove3d 使用说明：
这是 robot 包的 Python 安装配置文件，colcon build 时会读取它。
foxglove3d.launch.py 中 package='robot'、executable='xxx' 的所有自定义节点，
都必须在 entry_points['console_scripts'] 中注册，否则 ros2 launch 找不到可执行入口。

本文件还负责把 launch、config、urdf、meshes、world、map 等资源安装到 share/robot，
因此 get_package_share_directory('robot') 才能在运行时找到 robot.xacro、nav2_params.yaml、studyroom.ply 等资源。
"""

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
            'chassis_feedback_node = robot.chassis_feedback_node:main',
            'publish_ply = robot.publish_ply:main',
            'virtual_ultrasonic = robot.virtual_ultrasonic:main',
            'chassis_controller_3d = robot.chassis_controller_3d:main',
            'virtual_imu = robot.virtual_imu_node:main',
            'obstacle_avoidance = robot.obstacle_avoidance_node:main',
            'range_to_scan = robot.range_to_scan_node:main',
            'pointcloud_obstacle_filter = robot.pointcloud_obstacle_filter:main',
            'terrain_analyzer = robot.terrain_analyzer_node:main',
        ],
    },
)

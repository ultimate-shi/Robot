"""robot 包安装配置，注册 ROS 节点和运行时资源。"""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'robot'

# 相机 profile 按型号/序列号/分辨率分目录，新增 profile 后无需再修改 setup.py。
camera_profiles = [
    (
        os.path.join('share', package_name, os.path.dirname(path)),
        [path],
    )
    for path in glob(os.path.join('config', 'cameras', '*', '*.yaml'))
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
            + glob(os.path.join('config', '*.rviz')),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))
            + glob(os.path.join('urdf', '*.urdf')),
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*.*')),
        ),
        (
            os.path.join('share', package_name, 'world'),
            glob(os.path.join('world', '*.sdf')),
        ),
        (
            os.path.join('share', package_name, 'map'),
            glob(os.path.join('map', '*.*')),
        ),
        *camera_profiles,
    ],
    # PLY 点云读取依赖由板端容器固定版本安装，此处同时声明 Python 包元数据。
    install_requires=['setuptools', 'plyfile>=1.0,<2'],
    zip_safe=True,
    maintainer='shijiahao',
    maintainer_email='shijiahao@todo.todo',
    description='ROS 2 robot digital twin and stereo navigation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_feedback_node = robot.chassis_feedback_node:main',
            'publish_ply = robot.publish_ply:main',
            'virtual_ultrasonic = robot.virtual_ultrasonic:main',
            'chassis_controller_node = robot.chassis_controller_node:main',
            'virtual_imu = robot.virtual_imu_node:main',
            'obstacle_avoidance = robot.obstacle_avoidance_node:main',
            'range_to_scan = robot.range_to_scan_node:main',
            'pointcloud_obstacle_filter = '
            'robot.pointcloud_obstacle_filter:main',
            'terrain_analyzer = robot.terrain_analyzer_node:main',
            'nav_controller_node = robot.nav_controller_node:main',
            'stereo_splitter_node = robot.stereo_splitter_node:main',
            'stereo_depth_node = robot.stereo_depth_node:main',
            'stereo_pointcloud_filter = '
            'robot.stereo_pointcloud_filter:main',
        ],
    },
)

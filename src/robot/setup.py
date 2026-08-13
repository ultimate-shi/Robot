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
            'chassis_feedback_node = robot.control.chassis_feedback:main',
            'publish_ply = robot.mapping.ply_publisher:main',
            'virtual_ultrasonic = robot.sensing.virtual_ultrasonic:main',
            'chassis_controller_node = robot.control.chassis_controller:main',
            'virtual_imu = robot.sensing.virtual_imu:main',
            'obstacle_avoidance = robot.safety.obstacle_avoidance:main',
            'range_to_scan = robot.safety.range_to_scan:main',
            'pointcloud_obstacle_filter = '
            'robot.perception.pointcloud_obstacle_filter:main',
            'terrain_analyzer = robot.perception.terrain_analyzer:main',
            'nav_controller_node = robot.control.nav_controller:main',
            'stereo_splitter_node = robot.sensing.stereo_splitter:main',
            'stereo_pair_throttle_node = '
            'robot.sensing.stereo_pair_throttle:main',
            'stereo_depth_node = robot.perception.stereo_depth:main',
            'stereo_pipeline_benchmark = '
            'robot.diagnostics.stereo_pipeline_benchmark:main',
            'stereo_pointcloud_filter = '
            'robot.perception.stereo_pointcloud_filter:main',
            'mapping_snapshot_manager = '
            'robot.mapping.snapshot_manager:main',
            'snapshot_local_observer = '
            'robot.perception.snapshot_local_observer:main',
            'goal_manager = robot.mission.goal_manager:main',
        ],
    },
)

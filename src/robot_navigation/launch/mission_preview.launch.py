"""使用方法：由 stereo_brain.launch.py include，只启动 Nav2 planner 与任务路径预演服务。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_navigation')
    nav2 = os.path.join(share, 'config', 'nav2_params.yaml')
    overrides = os.path.join(share, 'config', 'nav2_stereo_overrides.yaml')
    mission = os.path.join(share, 'config', 'mission.yaml')
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[nav2, overrides, {'use_sim_time': False}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_brain_planner', parameters=[{
                 'use_sim_time': False, 'autostart': True,
                 'node_names': ['planner_server']}], output='screen'),
        Node(package='robot_navigation', executable='mission_planner',
             name='brain_mission', parameters=[mission, {'motion_enabled': False}], output='screen'),
    ])

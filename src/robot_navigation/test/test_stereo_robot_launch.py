"""使用方法：pytest 运行本文件，验证双目机器人组合入口的关键 launch 合同。"""

from pathlib import Path


LAUNCH_DIRECTORY = Path(__file__).parents[1] / 'launch'
SOURCE_DIRECTORY = Path(__file__).parents[2]


def launch_text(package, filename):
    """读取指定功能包的 launch 源码，避免测试依赖已安装的 ROS 环境。"""
    return (
        SOURCE_DIRECTORY / package / 'launch' / filename
    ).read_text(encoding='utf-8')


def test_stereo_robot_selects_mode_from_map_yaml_file():
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')

    assert "'map_yaml_file', default_value=''" in text
    assert "has_saved_map = PythonExpression" in text
    assert text.count('condition=UnlessCondition(has_saved_map)') == 3
    assert "'use_map_server': has_saved_map" in text


def test_stereo_robot_reuses_each_independent_function_once():
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')
    expected = [
        'description.launch.py',
        'foxglove.launch.py',
        'imu.launch.py',
        'stereo_odometry.launch.py',
        'state_estimation.launch.py',
        'wheel_odometry.launch.py',
        'stereo_camera.launch.py',
        'stereo_pointcloud_filter.launch.py',
        'head_mapping_lock.launch.py',
        'rtabmap_mapping.launch.py',
        'mapping_snapshot.launch.py',
        'controllers.launch.py',
        'chassis_control.launch.py',
        'nav_velocity_gate.launch.py',
        'obstacle_avoidance.launch.py',
        'nav2.launch.py',
        'semantic_detection.launch.py',
        'acceptance_sampler.launch.py',
    ]

    assert 'Node(' not in text
    assert 'LifecycleNode(' not in text
    for filename in expected:
        assert text.count(filename) == 1
    assert "'start_foxglove_bridge': 'false'" in text
    assert "'publish_odometry': 'false'" in text


def test_stereo_robot_preserves_staged_startup():
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')
    camera_text = launch_text('robot_perception', 'stereo_camera.launch.py')
    nav2_text = launch_text('robot_navigation', 'nav2.launch.py')
    controllers_text = launch_text('robot_control', 'controllers.launch.py')

    assert "'sensor_start_delay', default_value='3.0'" in text
    assert "'control_start_delay', default_value='3.0'" in text
    assert "'navigation_start_delay', default_value='14.0'" in text
    assert "'perception_start_delay', default_value='8.0'" in text
    assert "'node_start_interval', default_value='0.8'" in text
    assert 'def stage_delay' in text
    assert "'node_start_interval': node_interval" in text
    assert "'node_start_interval', default_value='0.0'" in camera_text
    assert "'node_start_interval', default_value='0.0'" in nav2_text
    assert 'target_action=current' in controllers_text
    assert 'on_exit=[following]' in controllers_text
    assert 'OpaqueFunction(function=create_deferred_spawners)' in controllers_text
    assert ".perform(context)" in controllers_text
    assert 'OpaqueFunction(function=register_camera_after_controls)' in camera_text


def test_delayed_callbacks_reuse_top_level_configurations():
    """异步退出回调使用的配置必须存在于顶层作用域，不能只在子 launch 中声明。"""
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')

    assert "'camera_config_file'" in text
    assert "'camera_config': LaunchConfiguration('camera_config_file')" in text
    assert "'controller_manager_config_file'" in text
    assert "'controller_config_file'" in text
    assert "'manager_config_file': LaunchConfiguration(\n                    'controller_manager_config_file')" in text
    assert "'controller_config_file': LaunchConfiguration(\n                    'controller_config_file')" in text


def test_localization_and_control_keep_single_odom_owner():
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')
    odometry_text = launch_text(
        'robot_navigation', 'stereo_odometry.launch.py')
    state_text = launch_text(
        'robot_navigation', 'state_estimation.launch.py')

    assert "('imu', '/sensors/imu/data')" in odometry_text
    assert "('odometry/filtered', LaunchConfiguration('output_topic'))" in state_text
    assert "'publish_odometry': 'false'" in text


def test_nav2_online_mode_can_disable_static_map_publishers():
    text = launch_text('robot_navigation', 'nav2.launch.py')

    assert "'use_map_server', default_value='true'" in text
    assert "name='map_server'" in text
    assert "name='lifecycle_manager_map'" in text
    assert "name='static_tf_map'" in text
    assert text.count("LaunchConfiguration('use_map_server')") == 3


def test_segformer_uses_an_independent_local_costmap_layer():
    overrides = (
        SOURCE_DIRECTORY / 'robot_navigation' / 'config' /
        'stereo_robot.yaml').read_text(encoding='utf-8')
    assert 'plugins: ["voxel_layer", "semantic_layer", "inflation_layer"]' in overrides
    assert 'topic: /nav/semantic_obstacle_points' in overrides
    assert 'topic: /nav/semantic_clear_points' in overrides
    assert 'marking: true  # 只增加语义障碍' in overrides
    assert 'marking: false  # 可通行语义不能创建障碍' in overrides
    assert 'global_costmap' in overrides
    global_section = overrides.split('global_costmap:', 1)[1]
    assert 'semantic_layer' not in global_section


def test_static_navigation_loads_maps_from_workspace_directory():
    """静态导航默认从工作区 maps 加载，不依赖已删除的 Package map 目录。"""
    nav2_text = launch_text('robot_navigation', 'nav2.launch.py')
    robot_text = launch_text('robot_navigation', 'robot.launch.py')
    publisher_text = (
        SOURCE_DIRECTORY / 'robot_navigation' / 'robot_navigation' /
        'mapping' / 'ply_publisher.py'
    ).read_text(encoding='utf-8')
    setup_text = (
        SOURCE_DIRECTORY / 'robot_navigation' / 'setup.py'
    ).read_text(encoding='utf-8')

    assert '/workspace/maps/studyroom/studyroom.yaml' in nav2_text
    assert '/workspace/maps/studyroom/studyroom.yaml' in robot_text
    assert '/workspace/maps/studyroom/studyroom.ply' in robot_text
    assert '/workspace/maps/studyroom/studyroom.ply' in publisher_text
    assert "glob('map/*.*')" not in setup_text

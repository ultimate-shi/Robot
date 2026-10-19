"""使用方法：pytest 运行本文件，验证实机兼容入口和 Nav2 的关键 launch 合同。"""

from pathlib import Path


LAUNCH_DIRECTORY = Path(__file__).parents[1] / 'launch'
SOURCE_DIRECTORY = Path(__file__).parents[2]


def launch_text(package, filename):
    """读取指定功能包的 launch 源码，避免测试依赖已安装的 ROS 环境。"""
    return (
        SOURCE_DIRECTORY / package / 'launch' / filename
    ).read_text(encoding='utf-8')


def test_stereo_robot_forwards_map_and_real_mode():
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')

    assert "'map_yaml_file', default_value=''" in text
    assert "'map_yaml_file', 'video_device', 'camera_config_file'" in text
    assert "forwarded['mode'] = 'real'" in text
    assert "get_package_share_directory('robot_main')" in text
    main_text = launch_text('robot_main', 'robot.launch.py')
    assert "'use_map_server': 'true' if map_yaml else 'false'" in main_text


def test_stereo_robot_delegates_composition_to_robot_main():
    text = launch_text('robot_navigation', 'stereo_robot.launch.py')
    main_text = launch_text('robot_main', 'robot.launch.py')

    assert 'Node(' not in text
    assert 'LifecycleNode(' not in text
    assert text.count('IncludeLaunchDescription(') == 1
    for filename in ('state_estimation.launch.py', 'nav2.launch.py',
                     'stereo_odometry.launch.py', 'rtabmap_mapping.launch.py'):
        assert filename in main_text


def test_stereo_robot_preserves_staged_startup():
    wrapper_text = launch_text('robot_navigation', 'stereo_robot.launch.py')
    main_text = launch_text('robot_main', 'robot.launch.py')
    camera_text = launch_text('robot_perception', 'stereo_camera.launch.py')
    nav2_text = launch_text('robot_navigation', 'nav2.launch.py')
    controllers_text = launch_text('robot_control', 'controllers.launch.py')

    for name in ('sensor_start_delay', 'control_start_delay',
                 'navigation_start_delay', 'perception_start_delay',
                 'node_start_interval'):
        assert f"'{name}'" in wrapper_text
        assert f"LaunchConfiguration('{name}')" in main_text
    assert 'def delayed(delay, action):' in main_text
    assert 'TimerAction(period=float(delay)' in main_text
    assert "'node_start_interval', default_value='0.0'" in camera_text
    assert "'node_start_interval', default_value='0.0'" in nav2_text
    assert 'target_action=current' in controllers_text
    assert 'on_exit=[following]' in controllers_text
    assert 'OpaqueFunction(function=create_deferred_spawners)' in controllers_text
    assert ".perform(context)" in controllers_text
    assert 'OpaqueFunction(function=register_camera_after_controls)' in camera_text


def test_delayed_callbacks_reuse_top_level_configurations():
    """异步退出回调使用的配置必须存在于顶层作用域，不能只在子 launch 中声明。"""
    wrapper_text = launch_text('robot_navigation', 'stereo_robot.launch.py')
    main_text = launch_text('robot_main', 'robot.launch.py')

    for name in ('camera_config_file', 'controller_manager_config_file',
                 'controller_config_file'):
        assert f"'{name}'" in wrapper_text
        assert f"LaunchConfiguration('{name}')" in main_text or (
            f"'{name}')" in main_text)
    assert "'camera_config': LaunchConfiguration('camera_config_file')" in main_text


def test_localization_and_control_keep_single_odom_owner():
    main_text = launch_text('robot_main', 'robot.launch.py')
    odometry_text = launch_text(
        'robot_navigation', 'stereo_odometry.launch.py')
    state_text = launch_text(
        'robot_navigation', 'state_estimation.launch.py')

    assert "('imu', '/sensors/imu/data')" in odometry_text
    assert "'publish_tf': False" in odometry_text
    assert "('odometry/filtered', LaunchConfiguration('output_topic'))" in state_text
    assert "'publish_odometry': 'false'" in main_text


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

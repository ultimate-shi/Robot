"""使用方法：pytest 运行本文件，验证复杂 launch 只组合可复用独立功能入口。"""

import ast
from pathlib import Path


SOURCE_DIRECTORY = Path(__file__).parents[2]


def launch_text(package, filename):
    """读取指定功能包的 launch 源码。"""
    return (
        SOURCE_DIRECTORY / package / 'launch' / filename
    ).read_text(encoding='utf-8')


def test_complex_launches_do_not_create_nodes_directly():
    complex_launches = [
        ('robot_brain', 'stereo_brain.launch.py'),
        ('robot_control', 'control.launch.py'),
        ('robot_control', 'safety.launch.py'),
        ('robot_navigation', 'robot.launch.py'),
        ('robot_navigation', 'stereo_mapping.launch.py'),
        ('robot_navigation', 'stereo_robot.launch.py'),
        ('robot_perception', 'stereo_perception.launch.py'),
        ('robot_perception', 'virtual_sensors.launch.py'),
    ]

    for package, filename in complex_launches:
        text = launch_text(package, filename)
        assert 'IncludeLaunchDescription' in text
        assert 'Node(' not in text
        assert 'LifecycleNode(' not in text


def test_stereo_mapping_reuses_independent_launches():
    text = launch_text('robot_navigation', 'stereo_mapping.launch.py')
    expected = [
        'description.launch.py',
        'joint_states.launch.py',
        'head_mapping_lock.launch.py',
        'stereo_camera.launch.py',
        'imu.launch.py',
        'stereo_odometry.launch.py',
        'state_estimation.launch.py',
        'wheel_odometry.launch.py',
        'stereo_pointcloud_filter.launch.py',
        'rtabmap_mapping.launch.py',
        'mapping_snapshot.launch.py',
        'foxglove.launch.py',
    ]

    for filename in expected:
        assert filename in text


def test_removed_legacy_compositions_are_not_present():
    launch_directory = SOURCE_DIRECTORY / 'robot_navigation' / 'launch'
    removed = [
        'mission_preview.launch.py',
        'navigation_preview.launch.py',
        'stereo_localization.launch.py',
    ]

    for filename in removed:
        assert not (launch_directory / filename).exists()


def test_all_launch_arguments_have_descriptions():
    for path in SOURCE_DIRECTORY.glob('*/launch/*.launch.py'):
        tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        assert ast.get_docstring(tree), f'{path} 缺少文件头使用方法'
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            function_name = getattr(node.func, 'id', '')
            if function_name != 'DeclareLaunchArgument':
                continue
            keywords = {keyword.arg for keyword in node.keywords}
            assert 'description' in keywords, f'{path}:{node.lineno} 缺少参数说明'


def test_async_launches_resolve_timer_periods_before_scope_exit():
    """定时器周期不能在子 launch 作用域结束后再读取 LaunchConfiguration。"""
    async_launches = [
        ('robot_control', 'chassis_control.launch.py'),
        ('robot_control', 'controllers.launch.py'),
        ('robot_navigation', 'nav2.launch.py'),
        ('robot_navigation', 'stereo_robot.launch.py'),
        ('robot_perception', 'stereo_camera.launch.py'),
    ]

    for package, filename in async_launches:
        text = launch_text(package, filename)
        assert 'OpaqueFunction' in text
        assert '.perform(context)' in text
        assert 'period=LaunchConfiguration(' not in text
        assert 'period=PythonExpression(' not in text


def test_process_exit_callbacks_resolve_arguments_before_scope_exit():
    """退出事件回调中的节点参数必须在注册回调时固化。"""
    camera_text = launch_text(
        'robot_perception', 'stereo_camera.launch.py')
    controllers_text = launch_text(
        'robot_control', 'controllers.launch.py')

    assert 'OpaqueFunction(function=register_camera_after_controls)' in camera_text
    assert 'resolved_config = camera_config.perform(context)' in camera_text
    assert 'OpaqueFunction(function=create_deferred_spawners)' in controllers_text
    assert "'controller_config_file').perform(context)" in controllers_text


def test_every_nested_launch_has_an_independent_configuration_scope():
    """兄弟功能的 config_file 等通用参数不能互相继承。"""
    launch_paths = sorted(SOURCE_DIRECTORY.glob('*/launch/*.launch.py'))

    for path in launch_paths:
        text = path.read_text(encoding='utf-8')
        if 'IncludeLaunchDescription(' not in text:
            continue
        assert 'GroupAction' in text, f'{path} 的子 launch 没有独立参数作用域'

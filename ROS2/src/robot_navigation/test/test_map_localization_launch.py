"""作用：验证实机已有地图由视觉定位独占地图变换，缺库时启动前报错；使用方法：pytest 本文件。"""

import importlib.util
from pathlib import Path
import sqlite3

import pytest
from launch import LaunchContext, LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, TimerAction


def main_launch():
    path = Path(__file__).parents[2] / 'robot_main' / 'launch' / 'robot.launch.py'
    spec = importlib.util.spec_from_file_location('test_robot_main_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def configuration(description, **values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def nested_includes(actions):
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            yield action
        elif isinstance(action, GroupAction):
            yield from nested_includes(action.get_sub_entities())
        elif isinstance(action, TimerAction):
            yield from nested_includes(action.actions)


def test_saved_real_map_uses_localization_and_disables_fixed_tf(tmp_path, monkeypatch):
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\n')
    with sqlite3.connect(tmp_path / 'map.db') as connection:
        connection.execute('CREATE TABLE Node (id INTEGER PRIMARY KEY)')
        connection.execute('INSERT INTO Node VALUES (1)')
    description = main_launch()
    context = configuration(description, mode='real', map_yaml_file=str(map_yaml),
                            initial_pose_enabled='true', initial_x='-0.4', initial_y='-1.0', initial_yaw='-2.2')
    composition = next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
    includes = list(nested_includes(composition))
    # 只解析子入口路径，不执行硬件子 launch。
    monkeypatch.setattr(PythonLaunchDescriptionSource, '_get_launch_description',
                        lambda self, location: LaunchDescription())
    for item in includes:
        item.launch_description_source.get_launch_description(context)
    names = [item.launch_description_source.location for item in includes]
    assert any('map_localization.launch.py' in name for name in names)
    localization = next(item for item in includes if 'map_localization.launch.py' in item.launch_description_source.location)
    arguments = dict(localization.launch_arguments)
    assert arguments['initial_pose_enabled'].perform(context) == 'true'
    assert arguments['initial_x'].perform(context) == '-0.4'
    assert arguments['initial_y'].perform(context) == '-1.0'
    assert arguments['initial_yaw'].perform(context) == '-2.2'
    assert not any('rtabmap_mapping.launch.py' in name for name in names)
    nav2 = next(item for item in includes if 'nav2.launch.py' in item.launch_description_source.location)
    assert dict(nav2.launch_arguments)['publish_map_to_odom'] == 'false'
    safety = next(item for item in includes if 'safety.launch.py' in item.launch_description_source.location)
    assert dict(safety.launch_arguments)['require_localization'] == 'true'


def test_missing_database_rejects_before_hardware_composition(tmp_path):
    (tmp_path / 'map.pgm').write_bytes(b'PGM')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\n')
    description = main_launch()
    context = configuration(description, mode='real', map_yaml_file=str(map_yaml))
    with pytest.raises(ValueError, match='缺少地图配套数据库'):
        next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)

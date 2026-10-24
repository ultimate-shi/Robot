"""作用：验证整车启动前能识别错误地图路径；使用方法：pytest -q src/robot_main/test。"""

import pytest

from robot_main.map_validation import validate_map_yaml


def test_valid_map_resolves_relative_image(tmp_path):
    (tmp_path / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: map.pgm\nresolution: 0.05\n', encoding='utf-8')
    assert validate_map_yaml(str(map_yaml)) == str(map_yaml)


def test_missing_map_reports_runtime_path(tmp_path):
    with pytest.raises(RuntimeError, match='地图 YAML 不存在.*当前 ROS 运行环境'):
        validate_map_yaml(str(tmp_path / 'missing.yaml'))


def test_missing_map_image_reports_image_path(tmp_path):
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_text('image: missing.pgm\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='地图图像不存在'):
        validate_map_yaml(str(map_yaml))

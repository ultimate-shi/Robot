"""使用方法：pytest 运行本文件，验证地图快照和预演点云纯函数。"""

from datetime import datetime, timezone
from types import SimpleNamespace

import numpy as np

from robot_navigation.mapping.snapshot_manager import (
    occupancy_to_pgm,
    occupancy_yaml,
    snapshot_time,
)
from robot_perception.snapshot_local_observer import quaternion_matrix


def make_grid():
    """构造 2x2 地图，验证 ROS 与图像坐标方向转换。"""
    orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    position = SimpleNamespace(x=-1.0, y=-2.0, z=0.0)
    origin = SimpleNamespace(position=position, orientation=orientation)
    info = SimpleNamespace(width=2, height=2, resolution=0.05, origin=origin)
    return SimpleNamespace(info=info, data=[0, 100, -1, 0])


def test_occupancy_to_pgm_flips_vertical_axis():
    payload = occupancy_to_pgm(make_grid())
    pixels = payload.split(b'\n', 4)[-1]
    # PGM 第一行对应 OccupancyGrid 最上方一行：unknown、free。
    assert list(pixels) == [205, 254, 254, 0]


def test_occupancy_yaml_is_nav2_compatible():
    text = occupancy_yaml(make_grid(), 'current.pgm')
    assert 'image: current.pgm' in text
    assert 'resolution: 0.05' in text
    assert 'origin: [-1, -2, 0]' in text
    assert 'mode: trinary' in text


def test_snapshot_time_uses_configured_timezone():
    utc_time = datetime(2026, 8, 15, 12, 3, 3, tzinfo=timezone.utc)
    local_time = snapshot_time('Asia/Shanghai', utc_time)
    assert local_time.strftime('map_%Y%m%d_%H%M%S') == (
        'map_20260815_200303')
    assert local_time.isoformat() == '2026-08-15T20:03:03+08:00'


def test_quaternion_matrix_rotates_map_points_into_base():
    half = np.sqrt(0.5)
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=half, w=half)
    rotation = quaternion_matrix(quaternion)
    point = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    transformed = point @ rotation.T
    assert np.allclose(transformed, [[0.0, 1.0, 0.0]], atol=1e-6)

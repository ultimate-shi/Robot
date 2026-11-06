"""作用：验证定位扫描的光心、未知区域和最近端点语义；使用方法：在 Jazzy 环境执行 pytest 本文件。"""

import math
import numpy as np
import pytest
from types import SimpleNamespace

from robot_perception.localization_scan import LocalizationScan, project_scan


def test_sensor_origin_and_nearest_endpoint():
    points = np.array([[1.2, .1, .3], [.7, .1, .3]])
    ranges = project_scan(points, [.2, .1, .3], -.5, .5, .1, .25, 4.)
    assert np.isclose(np.nanmin(ranges), .5)
    assert np.isfinite(ranges).sum() == 1
    assert np.isnan(ranges).sum() == len(ranges)-1


def test_unknown_depth_and_out_of_range_do_not_clear_space():
    points = np.array([[.1, 0., 0.], [5., 0., 0.], [1., 10., 0.]])
    ranges = project_scan(points, [0., 0., 0.], -.5, .5, .1, .25, 4.)
    assert np.isnan(ranges).all()
    assert np.isnan(project_scan(np.empty((0, 3)), [0., 0., 0.], -.5, .5, .1, .25, 4.)).all()


def test_horizontal_scan_has_no_fabricated_returns_outside_view():
    angles = np.linspace(-.2, .2, 20)
    points = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(20)))
    ranges = project_scan(points, np.zeros(3), -math.pi/3, math.pi/3, .01, .25, 4.)
    assert np.isnan(ranges[:70]).all()
    assert np.isnan(ranges[-70:]).all()


@pytest.mark.parametrize('case,expected', [('old', 'invalid_timestamp'), ('future', 'invalid_timestamp'),
    ('missing_tf', 'invalid_cloud_or_tf'), ('tilt', 'tilted'), ('mount', 'mount_moved')])
def test_scan_safety_rejects_unusable_frames(case, expected):
    scanner = object.__new__(LocalizationScan)
    scanner.last_stamp, scanner.max_message_age = -1, 1.5
    scanner.last_processed, scanner.max_input_rate = -math.inf, 4.
    scanner.odom_frame, scanner.target_frame, scanner.sensor_frame = 'odom', 'base_link', 'depth'
    scanner.max_tilt, scanner.mount_tolerance, scanner.origin = .174533, .005, np.array([.1, 0., .25])
    scanner.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    scanner.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
    states = []
    scanner.report = lambda state, *a: states.append(state)
    def lookup(target, source, stamp):
        if case == 'missing_tf':
            raise RuntimeError('历史 TF 缺失')
        return SimpleNamespace(rotation=SimpleNamespace(x=math.sin(.2) if case == 'tilt' else 0.,
            y=0., z=0., w=1.), translation=SimpleNamespace(x=.2, y=0., z=.25))
    scanner.lookup = lookup
    stamp = SimpleNamespace(sec=7 if case == 'old' else 11 if case == 'future' else 9, nanosec=200_000_000)
    scanner.on_cloud(SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='depth')))
    assert states == [expected]

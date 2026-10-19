"""用途：验证低精度舵机到位后持续低速修正及失效停车。

使用方法：在 Jazzy 容器内运行 python3 -m pytest -q src/robot_control/test/test_steering_motion_limiter.py。
"""

import pytest

from types import SimpleNamespace

from robot_control.chassis_controller import ChassisController3D, SteeringMotionLimiter


def limiter():
    return SteeringMotionLimiter(0.05, 0.08, 0.15, 2, 0.75)


def test_feedback_frames_release_then_angle_error_scales_speed():
    control = limiter()
    target = [0.2] * 4
    actual = [0.13] * 4

    assert control.update(target, actual, True, 1) == 0.0
    assert control.update(target, actual, True, 1) == 0.0
    first_scale = control.update(target, actual, True, 2)
    assert 0.75 < first_scale < 1.0
    assert control.update(target, [0.11] * 4, True, 3) < first_scale
    assert control.update(target, [0.16] * 4, True, 4) == 1.0


def test_large_error_stale_feedback_and_new_route_stop_wheels():
    control = limiter()
    straight = [0.0] * 4
    turn = [0.25, 0.20, -0.25, -0.20]
    assert control.update(straight, straight, True, 1) == 0.0
    assert control.update(straight, straight, True, 2) == 1.0

    assert control.update(turn, straight, True, 3) == 0.0
    assert control.update(turn, turn, True, 4) == 0.0
    assert control.update(turn, turn, True, 5) == 1.0
    assert control.update(turn, [0.42, 0.20, -0.25, -0.20], True, 6) == 0.0
    assert control.update(turn, turn, True, 7) == 0.0
    assert control.update(turn, turn, True, 8) == 1.0
    assert control.update(turn, turn, False, 8) == 0.0
    assert control.update(turn, turn, True, 9) == 0.0
    assert control.update(turn, turn, True, 10) == 1.0


def test_missing_or_invalid_feedback_stops_wheels():
    control = limiter()
    target = [0.0] * 4
    assert control.update(target, None, False, 0) == 0.0
    assert control.update(target, [0.0] * 3, True, 1) == 0.0
    assert control.update(target, [float('nan'), 0.0, 0.0, 0.0], True, 2) == 0.0


def test_zero_wheel_speed_keeps_the_requested_steering_angle():
    controller = SimpleNamespace(steering_limit=1.57)
    angle, speed = ChassisController3D.optimize_steering(
        controller, 0.32, 0.0, 0.32)
    assert angle == 0.32
    assert speed == 0.0


@pytest.mark.parametrize('full,ready,stop,samples,scale', [
    (0.0, 0.08, 0.15, 2, 0.25),
    (0.08, 0.08, 0.15, 2, 0.25),
    (0.05, 0.08, 0.08, 2, 0.25),
    (0.05, 0.08, 0.15, 0, 0.25),
    (0.05, 0.08, 0.15, 2, 0.0),
    (0.05, 0.08, 0.15, 2, float('inf')),
])
def test_invalid_limits_are_rejected(full, ready, stop, samples, scale):
    with pytest.raises(ValueError):
        SteeringMotionLimiter(full, ready, stop, samples, scale)

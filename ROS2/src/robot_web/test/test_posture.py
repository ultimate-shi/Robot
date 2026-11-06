"""作用：验证只伸展倾斜、保持倾角升降与共同限位；用法：pytest -q 本文件。"""
import math

import pytest
from robot_web.posture import angles, heights, body_step, bounded_move


@pytest.mark.parametrize('intent,expected', [
    ({'pitch': 1}, [0, 0, 5, 5]),
    ({'pitch': -1}, [5, 5, 0, 0]),
    ({'roll': 1}, [0, 5, 0, 5]),
    ({'roll': -1}, [5, 0, 5, 0]),
    ({'pitch': 1, 'roll': 1}, [0, 5, 5, 10]),
])
def test_tilt_extends_only_opposite_legs(intent, expected):
    result, limited = body_step([0]*4, intent, 5, [-30]*4, [30]*4)
    assert result == expected
    assert not limited


@pytest.mark.parametrize('lift', [-1, 1])
def test_lifting_keeps_tilt_and_stops_at_first_limit(lift):
    current = [0, 4, 10, 14]
    result, limited = body_step(current, {'lift': lift}, 50, [-20]*4, [20]*4)
    assert limited
    assert [b-a for a, b in zip(current, result)] == [6 if lift > 0 else -20]*4
    assert [v-result[0] for v in result] == current


def test_tilt_limit_does_not_block_lowering_or_reverse_tilt():
    current = [0, 0, 30, 30]
    result, limited = body_step(current, {'pitch': 1, 'lift': -1}, 2, [-30]*4, [30]*4)
    assert limited
    assert result == [-2, -2, 28, 28]
    reverse, limited = body_step(current, {'pitch': -1}, 2, [-30]*4, [30]*4)
    assert reverse == [2, 2, 30, 30]
    assert not limited


def test_first_leg_limits_all_participating_steps_proportionally():
    result, limited = bounded_move([0, 28, 0, 26], [0, 5, 5, 10], [-30]*4, [30]*4)
    assert result == [0, 30, 2, 30]
    assert limited


def test_height_conversion_has_opposite_front_rear_angles():
    target = angles([20]*4, 60)
    assert target[0] == target[1] == -target[2] == -target[3]
    assert heights(target, 60) == pytest.approx([20]*4)
    asymmetric = [-20, 0, 15, 30]
    assert angles(heights(asymmetric, 60), 60) == pytest.approx(asymmetric)


def test_five_degree_lift_preserves_existing_tilt_with_common_height_step():
    from robot_web.posture import body_angle_step
    current = [5, 12, -8, -20]
    target, limited = body_angle_step(current, 'e', 5, 60, [-30]*4, [30]*4)
    assert not limited
    changes = [abs(b-a) for a,b in zip(current,target)]
    assert max(changes) == pytest.approx(5)
    delta = [b-a for a,b in zip(heights(current,60),heights(target,60))]
    assert delta == pytest.approx([delta[0]]*4)


def test_five_degree_tilt_extends_only_opposite_pair_and_first_limit_stops_group():
    from robot_web.posture import body_angle_step
    target, limited = body_angle_step([0]*4, 'w', 5, 60, [-30]*4, [30]*4)
    assert target == pytest.approx([0,0,-5,-5])
    target, limited = body_angle_step([0,0,-29,-28], 'w', 5, 60, [-30]*4, [30]*4)
    assert limited
    assert target[2] == pytest.approx(-30)
    assert -30 < target[3] < -28

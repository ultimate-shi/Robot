"""使用方法：运行 pytest 验证四轮正反向反查、限幅和错误标定拒绝行为。"""

import copy

import pytest

from robot_control.wheel_curve_model import WHEEL_NAMES, DIRECTIONS, WheelCurveModel


def valid_curves():
    """构造有意不同的四轮正反向响应。"""
    curves = {}
    for index, wheel in enumerate(WHEEL_NAMES):
        scale = 1.0 + index * 0.1
        curves[wheel] = {
            'forward': ([1.0, 2.0], [0.8 * scale, 1.6 * scale]),
            'reverse': ([1.0, 2.0], [0.5 * scale, 1.0 * scale]),
        }
    return curves


def test_each_wheel_and_direction_has_distinct_inverse():
    model = WheelCurveModel(valid_curves(), 4.0, 3.0)
    assert model.compensate('fl', 0.0) == 0.0
    assert model.compensate('fl', -0.0) == 0.0
    assert model.compensate('fl', 1.2) == pytest.approx(1.5)
    assert model.compensate('fl', -0.75) == pytest.approx(-1.5)
    assert model.compensate('rr', 1.2) == pytest.approx(1.2 / 1.04)
    assert model.compensate('rr', -0.75) == pytest.approx(-0.75 / 0.65)


def test_no_extrapolation_and_compensation_limits():
    model = WheelCurveModel(valid_curves(), 4.0, 1.2)
    assert model.compensate('fl', 20.0) == pytest.approx(2.0)
    assert model.compensate('fl', -0.1) == pytest.approx(-0.12)
    assert model.compensate('fl', 1.2) == pytest.approx(1.44)


@pytest.mark.parametrize('mutation', [
    lambda curves: curves['rr'].pop('reverse'),
    lambda curves: curves['rr'].__setitem__('reverse', ([1.0], [0.5])),
    lambda curves: curves['rr'].__setitem__('reverse', ([1.0, 2.0], [0.5, 0.5])),
    lambda curves: curves['rr'].__setitem__('reverse', ([1.0, 2.0], [-0.5, 1.0])),
    lambda curves: curves['rr'].__setitem__('reverse', ([1.0, 2.0], [float('nan'), 1.0])),
    lambda curves: curves['rr'].__setitem__('reverse', ([1.0, 5.0], [0.5, 1.0])),
])
def test_invalid_or_incomplete_curve_rejected(mutation):
    curves = copy.deepcopy(valid_curves())
    mutation(curves)
    with pytest.raises(ValueError):
        WheelCurveModel(curves, 4.0, 2.0)


def test_nonfinite_target_rejected():
    model = WheelCurveModel(valid_curves(), 4.0, 2.0)
    with pytest.raises(ValueError):
        model.compensate('fl', float('inf'))

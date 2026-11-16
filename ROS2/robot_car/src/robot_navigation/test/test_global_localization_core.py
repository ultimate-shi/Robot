"""作用：检查全局定位的相关性、确认、冲突和地图质量；使用方法：pytest 本文件，无需启动 ROS。"""

import math

import numpy as np
import pytest

from robot_navigation.global_localization_core import (
    CorrectionFusion, MapLikelihood, compose, correction_with_covariance,
    composed_covariance, covariance_intersection, inverse, planar_covariance)


def confirm(fusion, source, x=0., yaw=0., start=1.):
    for stamp in np.arange(start, start+.3, .1):
        fusion.observe(source, np.array([x, 0., yaw]), np.diag([.02, .02, .03]), float(stamp))


def test_time_aligned_correction_is_constant_while_robot_moves_and_turns():
    correction = np.array([2., -1., .3])
    for pose in ([0., 0., 0.], [1., 2., .5], [2., 3., -2.]):
        odom = np.array(pose)
        mapped = compose(correction, odom)
        value, covariance = correction_with_covariance(mapped, np.eye(3)*.01, odom, np.eye(3)*.02)
        assert np.allclose(value, correction)
        assert np.linalg.eigvalsh(covariance).min() > 0
        assert np.allclose(compose(odom, inverse(odom)), 0.)


def test_covariance_jacobians_match_numerical_perturbation():
    a, b = np.array([2., 1., .7]), np.array([1., -2., -.4])
    _, actual = correction_with_covariance(a, np.diag([.02, .03, .04]), b, np.diag([.05, .06, .07]))
    from robot_navigation.global_localization_core import wrap
    jacobians = []
    for which in (0, 1):
        jacobian = np.zeros((3, 3))
        for axis in range(3):
            plus, minus = [a.copy(), b.copy()], [a.copy(), b.copy()]
            plus[which][axis] += 1e-6
            minus[which][axis] -= 1e-6
            delta = compose(plus[0], inverse(plus[1]))-compose(minus[0], inverse(minus[1]))
            delta[2] = wrap(delta[2])
            jacobian[:, axis] = delta/2e-6
        jacobians.append(jacobian)
    expected = jacobians[0] @ np.diag([.02, .03, .04]) @ jacobians[0].T
    expected += jacobians[1] @ np.diag([.05, .06, .07]) @ jacobians[1].T
    assert np.allclose(actual, 2*expected)


def test_intersection_does_not_double_confidence_and_handles_angle_wrap():
    covariance = np.diag([.02, .02, .03])
    value, result = covariance_intersection(np.array([0., 0., math.pi-.01]), covariance,
                                           np.array([.1, 0., -math.pi+.01]), covariance)
    assert np.allclose(result, covariance)
    assert value[0] == pytest.approx(.05)
    assert abs(abs(value[2])-math.pi) < 1e-8


def test_output_pose_covariance_accounts_for_shared_odometry():
    result = composed_covariance(np.zeros(3), np.eye(3)*.1, np.zeros(3), np.eye(3)*.1)
    assert np.allclose(result, np.eye(3)*.4)


def test_sources_confirm_independently_then_fuse_and_prioritize_visual():
    fusion = CorrectionFusion()
    assert fusion.observe('visual', np.zeros(3), np.eye(3), 1.) is False
    assert fusion.observe('amcl', np.zeros(3), np.eye(3), 1.) is False
    assert fusion.select(1.1)[0] is None
    confirm(fusion, 'visual', start=1.1)
    confirm(fusion, 'amcl', x=.1, start=1.1)
    assert fusion.select(1.5)[1:] == ('fused', False)
    confirm(fusion, 'amcl', x=2., start=2.)
    estimate, source, conflict = fusion.select(2.5)
    assert source == 'visual' and conflict
    assert estimate.value[0] == 0.


def test_dropout_falls_back_and_old_measurements_cannot_refresh():
    fusion = CorrectionFusion(freshness=1.)
    confirm(fusion, 'visual')
    confirm(fusion, 'amcl', start=1.5)
    assert fusion.select(2.5)[1] == 'amcl'
    assert fusion.observe('visual', np.zeros(3), np.eye(3), 1.) is False
    assert fusion.select(4.)[0] is None
    fusion.reset()
    assert fusion.select(1.5)[0] is None


def test_large_jump_needs_three_consistent_new_measurements():
    fusion = CorrectionFusion()
    confirm(fusion, 'visual')
    for stamp in (2., 2.1):
        assert not fusion.observe('visual', np.array([2., 0., 0.]), np.eye(3), stamp)
        assert fusion.select(stamp)[0].value[0] == 0.
    assert fusion.observe('visual', np.array([2., 0., 0.]), np.eye(3), 2.2)
    assert fusion.select(2.2)[0].value[0] == 2.


@pytest.mark.parametrize('invalid', ['nan', 'negative', 'asymmetric'])
def test_covariance_rejects_invalid_matrix(invalid):
    values = np.eye(6)*.1
    if invalid == 'nan':
        values[0, 0] = math.nan
    elif invalid == 'negative':
        values[0, 0] = -.1
    else:
        values[0, 1] = .1
    with pytest.raises(ValueError):
        planar_covariance(values.reshape(-1), [.005, .005, .01])


def test_map_score_rejects_unknown_empty_and_wrong_contour():
    cells = np.zeros((20, 20), dtype=int)
    cells[:, 10] = 100
    cells[:, 15:] = -1
    likelihood = MapLikelihood(cells, .1, np.zeros(3))
    points = np.array([[1., .2], [1., .5], [1., 1.]])
    assert likelihood.score(points, np.zeros(3), .15) == 1.
    assert likelihood.score(points, np.array([.6, 0., 0.]), .15) == 0.
    assert likelihood.score(points, np.array([-5., 0., 0.]), .15) == 0.
    empty = MapLikelihood(np.zeros((20, 20)), .1, np.zeros(3))
    assert empty.score(points, np.zeros(3), .15) == 0.


def test_rotated_map_origin_is_respected():
    cells = np.zeros((20, 20), dtype=int)
    cells[:, 10] = 100
    likelihood = MapLikelihood(cells, .1, np.array([2., 3., math.pi/2]))
    assert likelihood.score(np.array([[1., .5]]), np.array([2., 3., math.pi/2]), .15) == 1.

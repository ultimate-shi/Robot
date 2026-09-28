"""使用方法：pytest 运行本文件，验证四轮反馈运动学和异常降权。"""

import math

import numpy as np

from robot_control.four_wheel_odometry import FourWheelOdometry


def test_four_wheel_odometry_forward_and_crab_motion():
    """四轮同向时应分别还原前进和横移速度。"""
    speed = np.full(4, 2.0)
    forward = FourWheelOdometry.compute_twist(
        np.zeros(4), speed, 0.05, 0.4, 0.2)
    crab = FourWheelOdometry.compute_twist(
        np.full(4, math.pi / 2.0), speed, 0.05, 0.4, 0.2)

    assert np.allclose(forward, [0.1, 0.0, 0.0], atol=1e-8)
    assert np.allclose(crab, [0.0, 0.1, 0.0], atol=1e-8)


def test_four_wheel_odometry_recovers_rigid_body_rotation():
    """按四个轮心刚体速度生成的转角和轮速应还原原始 yaw_rate。"""
    yaw_rate = 0.5
    positions = np.array([
        [0.2, 0.1], [0.2, -0.1], [-0.2, 0.1], [-0.2, -0.1]])
    vectors = np.column_stack((
        -yaw_rate * positions[:, 1], yaw_rate * positions[:, 0]))
    steering = np.arctan2(vectors[:, 1], vectors[:, 0])
    wheel_speed = np.linalg.norm(vectors, axis=1) / 0.05

    result = FourWheelOdometry.compute_twist(
        steering, wheel_speed, 0.05, 0.4, 0.2)

    assert np.allclose(result, [0.0, 0.0, yaw_rate], atol=1e-8)


def test_dynamic_wheel_positions_follow_original_leg_geometry():
    """前后腿转动时轮心 X 坐标应按旧 chassis.c 的方向变化。"""
    positions = FourWheelOdometry.wheel_positions(
        np.full(4, math.radians(30.0)), 0.312, 0.280, 0.060)
    assert positions[0, 0] < 0.312 / 2.0
    assert positions[1, 0] < 0.312 / 2.0
    assert positions[2, 0] > -0.312 / 2.0
    assert positions[3, 0] > -0.312 / 2.0


def test_robust_solver_marks_single_wheel_outlier():
    """单轮异常不应直接拖偏全部轮子，且应被硬残差标记。"""
    positions = FourWheelOdometry.wheel_positions(
        np.zeros(4), 0.312, 0.280, 0.060)
    wheel_speed = np.full(4, 0.2 / 0.055)
    wheel_speed[0] = 1.0 / 0.055
    solution, residual, weight, suspected = FourWheelOdometry.robust_twist(
        np.zeros(4), wheel_speed, 0.055, positions,
        0.05, 0.15, 8)
    assert suspected[0]
    assert weight[0] == 0.0
    assert np.count_nonzero(weight > 0.05) >= 3
    corrected, _, _, _ = FourWheelOdometry.robust_twist(
        np.zeros(4), wheel_speed, 0.055, positions,
        0.05, 0.15, 8, initial_weight=weight)
    assert math.isclose(corrected[0], 0.2, abs_tol=0.02)

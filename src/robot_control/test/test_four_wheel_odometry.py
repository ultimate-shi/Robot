"""使用方法：pytest 运行本文件，验证未来四轮 JointState 运动学接口。"""

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

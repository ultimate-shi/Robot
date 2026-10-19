"""使用方法：在 Jazzy 环境运行 pytest，验证分腿限位与高度换算。"""

import math
import time

from robot_interfaces.srv import SetBodyBalance
from sensor_msgs.msg import JointState
from robot_control.body_balance import (
    BodyBalance, DEFAULT_LEG_MAX_DEG, DEFAULT_LEG_MIN_DEG)


def make_balance():
    """仅构造限位计算所需状态，不启动 ROS 节点。"""
    balance = object.__new__(BodyBalance)
    balance.leg_length = 0.060
    balance.leg_min_deg = DEFAULT_LEG_MIN_DEG
    balance.leg_max_deg = DEFAULT_LEG_MAX_DEG
    return balance


def test_four_leg_height_endpoints_follow_hardware_angle_limits():
    balance = make_balance()
    for index in range(4):
        lower, upper = balance._height_limits(index)
        angles = [balance._height_to_degree(height, index)
                  for height in (lower, upper)]
        assert all(math.isclose(actual, expected, abs_tol=1e-8)
                   for actual, expected in zip(
                       sorted(angles),
                       [DEFAULT_LEG_MIN_DEG[index], DEFAULT_LEG_MAX_DEG[index]]))
        assert math.isclose(balance._height_to_degree(lower - 20.0, index),
                            angles[0], abs_tol=1e-8)
        assert math.isclose(balance._height_to_degree(upper + 20.0, index),
                            angles[1], abs_tol=1e-8)


def test_published_targets_are_bounded_for_static_and_restore_paths():
    balance = make_balance()

    class Publisher:
        def __init__(self):
            self.value = None

        def publish(self, message):
            self.value = math.degrees(message.data[0])

    balance.command_publishers = [Publisher() for _ in range(4)]
    balance._publish_commands([-70.0, 70.0, 70.0, -70.0])
    assert balance.last_command_deg == [-30.0, 30.0, 30.0, -30.0]
    assert [round(item.value) for item in balance.command_publishers] == [
        -30, 30, 30, -30]


def test_asymmetric_limits_keep_rear_leg_height_direction():
    """未来若按腿重新标定，后腿的高度上下限仍应正确反向。"""
    balance = make_balance()
    balance.leg_min_deg = [-30.0, -20.0, -15.0, -35.0]
    balance.leg_max_deg = [25.0, 30.0, 35.0, 20.0]
    lower, upper = balance._height_limits(2)
    assert math.isclose(balance._height_to_degree(lower, 2), 35.0, abs_tol=1e-8)
    assert math.isclose(balance._height_to_degree(upper, 2), -15.0, abs_tol=1e-8)


def test_negative_leg_current_triggers_overload_protection():
    """DMC0 电流可为负，过流判断必须比较幅值。"""
    balance = make_balance()
    balance.leg_current = [-2400.0, 0.0, 0.0, 0.0]
    balance.overload_current_ma = 2300.0
    balance.overload_hold = 2.0
    balance.overload_since = None
    balance.overload_active = False
    balance.recover_since = None
    balance.level = 'mid'
    balance.leg_angle_deg = [0.0] * 4
    balance.target_roll = 1.0
    balance.target_pitch = 1.0
    balance._evaluate_overload(1.0)
    balance._evaluate_overload(3.1)
    assert balance.overload_active
    assert balance.level == 'low'
    assert balance.target_roll == balance.target_pitch == 0.0


def test_measured_nonzero_stance_keeps_each_leg_height():
    """右前腿未回零时，初始高度换算不应擅自改变四腿基准。"""
    balance = make_balance()
    balance.leg_angle_deg = [-0.29, 22.54, -1.75, -3.77]
    heights = balance._angles_to_heights()
    assert [round(balance._height_to_degree(value, index), 2)
            for index, value in enumerate(heights)] == balance.leg_angle_deg


def test_overload_does_not_block_emergency_disable():
    """过流时可关闭，并把已越过临时目标窗的实际角度送给底层保持。"""
    balance = make_balance()
    balance.leg_min_deg = [-1.0] * 4
    balance.leg_max_deg = [1.0] * 4

    class Publisher:
        def __init__(self):
            self.value = None

        def publish(self, message):
            self.value = math.degrees(message.data[0])

    balance.command_publishers = [Publisher() for _ in range(4)]
    balance.max_setpoint_deg = 10.0
    balance.overload_active = True
    balance.enabled = True
    balance.level = 'low'
    balance.leg_angle_deg = [2.0, 0.0, -2.0, 0.0]
    balance.heights = [0.0] * 4
    balance.height_center = 0.0
    balance.last_command_deg = [0.0] * 4
    balance.mid_restore_hold = 2.5
    request = SetBodyBalance.Request()
    request.enabled = False
    request.level = 'mid'
    response = balance._set_balance(request, SetBodyBalance.Response())
    assert response.success
    assert not balance.enabled
    assert balance.heights is None
    assert all(math.isclose(publisher.value, degree, abs_tol=1e-8)
               for publisher, degree in zip(
                   balance.command_publishers, balance.leg_angle_deg))


def test_nonfinite_leg_feedback_does_not_replace_last_valid_angles():
    """角度反馈无效时仍保留可用于停车保持的最近有效值。"""
    balance = make_balance()
    balance.leg_angle_deg = [1.0, 2.0, 3.0, 4.0]
    message = JointState()
    message.name = ['lap_body_joint_fl', 'lap_body_joint_fr',
                    'lap_body_joint_rl', 'lap_body_joint_rr']
    message.position = [0.0, float('nan'), 0.0, 0.0]
    balance._joint_callback(message)
    assert balance.leg_angle_deg == [1.0, 2.0, 3.0, 4.0]


def test_four_imu_tilt_directions_move_the_expected_sides():
    """用已实测的姿态符号核对四种倾斜下的腿部高度目标。"""
    for roll, pitch, expected in (
            (-5.0, 0.0, (1, -1, 1, -1)),
            (5.0, 0.0, (-1, 1, -1, 1)),
            (0.0, -5.0, (1, 1, -1, -1)),
            (0.0, 5.0, (-1, -1, 1, 1))):
        balance = make_balance()
        balance.enabled = True
        balance.level = 'mid'
        balance.leg_angle_deg = [0.0] * 4
        balance.leg_current = None
        balance.heights = None
        balance.height_center = None
        balance.last_command_deg = None
        balance.last_imu_time = time.monotonic()
        balance.filtered_roll = roll
        balance.filtered_pitch = pitch
        balance.filtered_gyro_roll = 0.0
        balance.filtered_gyro_pitch = 0.0
        balance.target_roll = 0.0
        balance.target_pitch = 0.0
        balance.restore_until = None
        balance.imu_timeout = 0.5
        balance.deadzone_deg = 1.0
        balance.wheelbase = 0.312
        balance.track = 0.280
        balance.kp = 0.1
        balance.kd_pitch = 2.0
        balance.kd_roll = 2.0
        balance.derivative_window = 0.05
        balance.soft_margin_mm = 4.0
        balance.hard_margin_mm = 1.0
        balance.safe_margin_mm = 0.1
        balance.max_step_deg = 5.0
        balance.command_publishers = [
            type('Publisher', (), {'publish': lambda self, message: None})()
            for _ in range(4)]
        balance._publish_status = lambda: None
        balance._control()
        height_signs = [1, 1, -1, -1]
        assert all(degree * sign * direction > 0.0
                   for degree, sign, direction in zip(
                       balance.last_command_deg, height_signs, expected))

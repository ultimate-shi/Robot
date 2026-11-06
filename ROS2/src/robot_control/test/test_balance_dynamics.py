"""作用：验证平衡控制在延迟、回正、限位和反馈中断时的行为；用法：Jazzy 中运行 pytest 本文件。"""

import math
from types import SimpleNamespace

import pytest

from robot_control.balance_motion import limit_motion, position_plane
from test_body_balance_limits import make_balance


def node(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('robot_control.body_balance.time.monotonic', lambda: clock[0])
    b = make_balance()
    b.enabled = True
    b.level = 'mid'
    b.leg_angle_deg = [0.0]*4
    b.leg_current = None
    b.last_joint_time = b.last_imu_time = clock[0]
    b.target_roll = b.target_pitch = 0.0
    b.filtered_roll = b.filtered_pitch = 0.0
    b.filtered_gyro_roll = b.filtered_gyro_pitch = 0.0
    b.last_command_deg = None
    b.restore_until = None
    b.heights = b.height_center = None
    b.saved_mid_angles = None
    b._publish_status = lambda: None
    commands = []
    b.command_publishers = [SimpleNamespace(publish=lambda msg: commands.append(msg.data[0]))
                            for _ in range(4)]
    return b, clock, commands


@pytest.mark.parametrize('dt', [0.05, 0.1, 0.15])
def test_velocity_acceleration_and_smooth_reversal(dt):
    position, velocity = [0.0]*4, [0.0]*4
    measured = position[:]
    for tick in range(30):
        target = [25.0 if tick < 10 else -25.0]*4
        result, rates = limit_motion(target, position, velocity, measured,
                                      [-30]*4, [30]*4, dt, 30, 60, 3)
        assert max(abs(v) for v in rates) <= 30+1e-8
        assert max(abs(a-b) for a, b in zip(rates, velocity)) <= 60*dt+1e-8
        measured = result[:]  # 此项只检查轨迹，机械延迟由后续测试覆盖。
        position, velocity = result, rates


@pytest.mark.parametrize('sign', [-1, 1])
def test_frozen_legs_do_not_accumulate_hidden_target(monkeypatch, sign):
    b, clock, commands = node(monkeypatch)
    b.filtered_roll = sign*15.0
    initial = None
    for _ in range(80):
        clock[0] += 0.1
        b.last_joint_time = b.last_imu_time = clock[0]
        b._control()
        assert max(abs(v) for v in b.last_command_deg) <= 30+1e-8
        if initial is None:
            initial = b.last_command_deg[:]
            assert max(abs(v) for v in initial) > 3
        assert b.last_command_deg == pytest.approx(initial)
        assert b.heights == pytest.approx([60*math.sin(math.radians(v))*s
                                         for v, s in zip(b.last_command_deg, [1, 1, -1, -1])])
    assert len(commands) == 4


def test_returning_angular_rate_brakes_before_level(monkeypatch):
    b, clock, _ = node(monkeypatch)
    b.filtered_roll = 5.0
    b._control()
    outward = b.last_command_deg[:]
    b.feedback_interrupted = True
    clock[0] += 0.1
    b.filtered_gyro_roll = -0.10  # 与倾角相反，正在回正。
    b._control()
    assert abs(b.last_command_deg[0]) < abs(outward[0]) or b.last_command_deg[0]*outward[0] < 0


def test_hysteresis_retains_hold_between_thresholds(monkeypatch):
    b, clock, commands = node(monkeypatch)
    b.filtered_roll = 0.9
    b._control()
    assert b.state == 'balanced'
    goal = b.last_command_deg[:]
    for roll in (0.9, 1.4, 1.1):
        clock[0] += 0.1
        b.filtered_roll = roll
        b.leg_angle_deg = [value+0.1 for value in b.leg_angle_deg]
        b.last_imu_time = b.last_joint_time = clock[0]
        b._control()
        assert b.state == 'balanced'
        assert b.last_command_deg == goal
    assert len(commands) == 4
    clock[0] += 0.1
    b.filtered_roll = 1.6
    b.last_imu_time = b.last_joint_time = clock[0]
    b._control()
    assert b.state == 'balancing'


@pytest.mark.parametrize('kind', ['imu', 'joint'])
def test_feedback_interruption_discards_old_target(monkeypatch, kind):
    b, clock, commands = node(monkeypatch)
    b.filtered_roll = 10.0
    b._control()
    clock[0] += 0.6
    setattr(b, f'last_{kind}_time', clock[0]-1.0)
    n = len(commands)
    b._control()
    assert len(commands) == n
    assert b.feedback_interrupted
    b.leg_angle_deg = [5.0]*4
    b.filtered_roll = b.filtered_pitch = 0.0
    b.last_joint_time = b.last_imu_time = clock[0]
    clock[0] += 0.1
    b._control()
    assert b.last_command_deg == pytest.approx([5.0]*4)


def test_long_cycle_does_not_catch_up(monkeypatch):
    b, clock, commands = node(monkeypatch)
    b.filtered_pitch = 10.0
    b._control()
    n = len(commands)
    clock[0] += 0.35
    b.last_joint_time = b.last_imu_time = clock[0]
    b._control()
    assert b.state == 'cycle_late'
    assert len(commands) == n
    assert b.last_command_deg == b.leg_angle_deg
    assert b.command_velocity_deg_s == [0]*4


@pytest.mark.parametrize('level', ['low', 'high'])
def test_static_levels_also_use_slew_and_lead_limits(monkeypatch, level):
    b, _, _ = node(monkeypatch)
    b.level = level
    b._control()
    assert 0 < max(abs(v) for v in b.last_command_deg) <= 0.6+1e-8


def test_cached_joint_broadcast_cannot_refresh_hardware_sample(monkeypatch):
    b, clock, commands = node(monkeypatch)
    b.require_hardware_timing = True
    b.last_hardware_feedback_time = clock[0]-1.0
    b.last_joint_time = clock[0]
    b._control()
    assert b.state == 'joint_stale'
    assert not commands


def test_hardware_recovery_does_not_reenable_balance(monkeypatch):
    b, _, _ = node(monkeypatch)
    b.require_hardware_timing = True
    msg = SimpleNamespace(outputs_enabled=False, manual_recovery_required=True,
                          leg_current_ma=[0.0]*4)
    b._hardware_callback(msg)
    assert not b.enabled
    assert b.state == 'hardware_fault'
    msg.outputs_enabled = True
    msg.manual_recovery_required = False
    b._hardware_callback(msg)
    assert not b.enabled


def test_restore_mid_uses_the_same_motion_limits(monkeypatch):
    b, clock, _ = node(monkeypatch)
    b.restore_until = clock[0]+2.5
    b.saved_mid_angles = [20, -20, 20, -20]
    b._control()
    assert b.state == 'restoring_mid'
    assert max(abs(v) for v in b.last_command_deg) <= 0.6+1e-8


@pytest.mark.parametrize('delay', [0.1, 0.2, 0.3])
def test_delayed_leg_model_keeps_commands_bounded(monkeypatch, delay):
    b, clock, _ = node(monkeypatch)
    history = [[0.0]*4 for _ in range(round(delay/0.1)+1)]
    for tick in range(100):
        clock[0] += 0.1
        b.last_joint_time = b.last_imu_time = clock[0]
        b.leg_angle_deg = history.pop(0)
        heights = b._angles_to_heights()
        # 左右高度差反馈到同一符号的倾角，前 0.5 秒叠加外部扰动。
        b.filtered_roll = math.degrees(math.atan2(
            (heights[0]+heights[2]-heights[1]-heights[3])/2, 280)) + (5 if tick < 5 else 0)
        b._control()
        assert all(abs(target) <= 30+1e-8 for target in b.last_command_deg)
        history.append(b.last_command_deg[:])
    assert max(abs(v) for v in b.last_command_deg) < 30


def test_plane_projection_removes_diagonal_warp():
    heights = position_plane([-6, 6, 6, -6], 0, 0, 0, 156, 140, [-30]*4, [30]*4)
    assert heights == pytest.approx([0]*4)


def test_saturation_preserves_plane_and_correction_ratio():
    heights = position_plane([0]*4, 5, math.radians(15), math.radians(10),
                             156, 140, [-30]*4, [30]*4)
    assert min(heights) >= -30 and max(heights) <= 30
    assert heights[0]-heights[1]-heights[2]+heights[3] == pytest.approx(0)


def test_mid_sends_full_target_once_and_ignores_small_noise(monkeypatch):
    b, clock, commands = node(monkeypatch)
    b.filtered_roll = 5.0
    b._control()
    assert max(abs(v) for v in b.last_command_deg) > 10
    assert len(commands) == 4
    goal = b.last_command_deg[:]
    for error in (5.02, 4.99, 5.01, 5.0):
        clock[0] += 0.1
        b.last_imu_time = b.last_joint_time = clock[0]
        b.filtered_roll = error
        b._control()
    assert len(commands) == 4
    assert b.last_command_deg == goal


def test_flat_body_does_not_keep_an_old_diagonal_target(monkeypatch):
    b, _, commands = node(monkeypatch)
    angle = math.degrees(math.asin(0.1))
    b.leg_angle_deg = [-angle, angle, -angle, angle]
    b._control()
    assert b.last_command_deg == pytest.approx([0]*4, abs=1e-8)
    assert len(commands) == 4


def test_full_pose_uses_common_height_shift_before_reducing_tilt():
    result = position_plane([0]*4, 25, 0, math.radians(5), 156, 140, [-30]*4, [30]*4)
    assert result[1]-result[0] == pytest.approx(280*math.tan(math.radians(5)))
    assert max(result) <= 30+1e-8

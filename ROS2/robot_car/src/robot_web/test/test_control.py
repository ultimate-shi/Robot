"""作用：验证手动直通状态、停车锁和多页面指令；使用方法：pytest -q src/robot_web/test。"""
import pytest

from robot_web.control import ControlState


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def test_manual_control_does_not_require_browser_heartbeat():
    clock = Clock()
    state = ControlState(clock=clock)
    assert not state.snapshot()['stopped']
    assert state.snapshot()['connected']
    state.command(0.04, 0.0, 'page-a', 1)
    assert state.manual_velocity() == (0.04, 0.0)
    clock.value = 0.61
    assert state.manual_velocity() == (0.0, 0.0)
    assert not state.snapshot()['stopped']
    assert state.snapshot()['mode'] == 'manual'
    assert state.manual_velocity() == (0.0, 0.0)


def test_manual_commands_are_not_rejected_by_sequence_order():
    state = ControlState()
    state.command(0.04, 0.0, 'page-a', 10)
    state.command(0.0, 0.1, 'page-b', 1)
    assert state.manual_velocity() == (0.0, 0.1)
    state.command(0.04, 0.0, 'page-a', 9)
    assert state.manual_velocity() == (0.04, 0.0)
    state.command(0.0, 0.0, 'page-b', 2)
    assert state.manual_velocity() == (0.0, 0.0)


def test_explicit_stop_stays_latched_until_reset():
    state = ControlState()
    state.stop()
    state.heartbeat()
    assert state.snapshot()['stopped']
    with pytest.raises(ValueError):
        state.command(0.04, 0.0, 'page-a', 1)
    state.reset_stop()
    assert not state.snapshot()['stopped']
    with pytest.raises(ValueError):
        state.command(float('nan'), 0.0, 'page-a', 2)
    state.command(1.0, 2.0, 'page-a', 3)
    assert state.manual_velocity() == (1.0, 2.0)


def test_mode_switch_and_command_timeout_never_replay_speed():
    clock = Clock()
    state = ControlState(clock=clock)
    state.command(0.08, 0.0, 'page-a', 1)
    state.set_mode('navigation')
    with pytest.raises(ValueError):
        state.command(0.04, 0.0, 'page-a', 2)
    state.set_mode('manual')
    assert state.manual_velocity() == (0.0, 0.0)
    state.command(0.04, 0.0, 'page-a', 3)
    clock.value = 0.36
    assert state.manual_velocity() == (0.0, 0.0)


def test_stream_disconnect_stops_and_old_http_cannot_replay():
    state = ControlState()
    token = object()
    state.command(0.04, 0.0, 'page', 1)
    assert state.stream_command(0.04, 0.0, 'page', 3, token)
    assert not state.command(0.0, 0.0, 'page', 2)
    assert state.manual_velocity() == (0.04, 0.0)
    assert state.stream_command(0.0, 0.0, 'page', 4, token)
    assert not state.command(0.04, 0.0, 'page', 1)
    state.release_stream(token)
    assert state.manual_velocity() == (0.0, 0.0)


def test_stream_retains_deadman_and_does_not_stop_new_owner():
    clock = Clock()
    state = ControlState(clock=clock)
    old, new = object(), object()
    state.stream_command(0.05, 0, 'old', 1, old)
    clock.value = 0.36
    assert state.manual_velocity() == (0.0, 0.0)
    state.stream_command(0.05, 0, 'new', 1, new)
    state.release_stream(old)
    assert state.manual_velocity() == (0.05, 0)
    state.release_stream(new)
    assert state.manual_velocity() == (0.0, 0.0)

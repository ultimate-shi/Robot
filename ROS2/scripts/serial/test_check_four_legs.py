"""使用方法：在仓库根目录运行 python3 -m pytest -q scripts/serial/test_check_four_legs.py。"""

import pytest

from check_four_legs import Dmc0, ProbeError, check_state, load_config, move, parse_degrees, parse_state, run


class FakeBoard:
    """模拟四腿目标与反馈，记录实际下发的原生单轴指令。"""

    def __init__(self, stalled=None, current=100):
        self.angles = [0.0] * 4
        self.commands = []
        self.stalled = stalled
        self.current = current

    def read_state(self):
        return [dict(angle_deg=angle, speed_deg_s=0, current_ma=self.current)
                for angle in self.angles]

    def target(self, index, angle):
        self.commands.append((index, angle))
        if index != self.stalled:
            self.angles[index] = angle


@pytest.fixture
def config():
    settings = load_config('src/robot_hardware/config/check_four_legs.yaml')
    settings['pause_s'] = 0
    settings['sample_period_s'] = 0.001
    settings['motion_timeout_s'] = 0.005
    return settings


def test_parse_state_applies_left_right_sign_and_signed_current():
    state = parse_state('R -200 1 -123 300 2 456 -400 3 789 500 4 -999'.split())
    assert [part['angle_deg'] for part in state] == [2, 3, 4, 5]
    assert state[0]['current_ma'] == -123


def test_four_legs_move_both_directions_and_return(config):
    board = FakeBoard()
    run(board, config, range(4), [2, 0, -2, 0])
    assert len(board.commands) == 16
    assert [index for index, _ in board.commands[::4]] == list(range(4))
    assert [angle for index, angle in board.commands if index == 0] == [2, 0, -2, 0]
    assert board.angles == [0.0] * 4


def test_stalled_leg_stops_sequence(config):
    board = FakeBoard(stalled=1)
    with pytest.raises(ProbeError, match='FR 未到位'):
        run(board, config, range(4), [2, 0])
    assert all(index <= 1 for index, _ in board.commands)


def test_signed_overcurrent_stops_before_command(config):
    board = FakeBoard(current=-config['current_limit_ma'])
    with pytest.raises(ProbeError, match='达到阈值'):
        move(board, 0, 2, config)
    assert board.commands == []


def test_other_leg_drift_is_rejected(config):
    baseline = FakeBoard().read_state()
    state = FakeBoard().read_state()
    state[2]['angle_deg'] = 2
    with pytest.raises(ProbeError, match='非目标腿 RL'):
        check_state(state, config, baseline, 0)


def test_explicit_angle_has_no_software_limit(config):
    board = FakeBoard()
    run(board, config, [0], [31])
    assert board.commands == [(0, 31)]


def test_protocol_int16_range_is_checked_before_serial_write():
    board = Dmc0.__new__(Dmc0)
    with pytest.raises(ProbeError, match='int16'):
        board.target(0, 400)


def test_single_leg_error_code_does_not_stop_target_or_hold():
    board = Dmc0.__new__(Dmc0)
    commands = []

    def exchange(command, prefix, count):
        commands.append(command)
        return [prefix, '0', '0', '0', '-3' if len(commands) == 1 else '-2']

    board.exchange = exchange
    board.target(0, 30)
    board.hold(FakeBoard().read_state())
    assert commands == ['R0 -3000', 'R0 0', 'R1 0', 'R2 0', 'R3 0']


def test_values_keep_written_order_and_allow_decimal_and_negative():
    assert parse_degrees('2.5,0,-1.25') == [2.5, 0, -1.25]

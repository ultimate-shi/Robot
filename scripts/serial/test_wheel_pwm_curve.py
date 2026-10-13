"""使用方法：运行 python3 -m pytest -q scripts/serial/test_wheel_pwm_curve.py 验证单轴采样与停车。"""

import wheel_pwm_curve


class FakeBoard:
    """只在目标轴返回运动状态，模拟 DMC1 四轴 S 响应。"""

    def __init__(self, motor_id, tps=-1160, current_ma=500):
        self.index = motor_id - 4
        self.tps = tps
        self.current_ma = current_ma
        self.encoder = 10000
        self.commands = []
        self.stops = 0

    def read_state(self):
        self.encoder += -10 if self.tps < 0 else 10
        states = [dict(tps=0, encoder=10000, current_ma=0) for _ in range(4)]
        states[self.index] = dict(
            tps=self.tps, encoder=self.encoder, current_ma=self.current_ma)
        return states

    def exchange(self, command, prefix, count):
        self.commands.append((command, prefix, count))
        return [prefix, '0', '0', '0']

    def stop(self):
        self.stops += 1


def test_each_motor_uses_its_own_state_and_ros_sign():
    for motor_id in (4, 5, 6, 7):
        board = FakeBoard(motor_id)
        point = wheel_pwm_curve.measure(
            board, motor_id, -20, 100, 100, 40, 10, 11600, -1, 3000)
        assert not point['error']
        assert point['command'] == f'D{motor_id} -20 100'
        assert point['mean_tps'] == -1160
        assert point['speed_rad_s'] > 0
        assert point['encoder_delta_ticks'] < 0
        assert board.commands[0][1] == f'D{motor_id}'
        assert board.stops == 1


def test_current_limit_stops_motor_and_marks_point_invalid():
    board = FakeBoard(6, current_ma=2800)
    point = wheel_pwm_curve.measure(
        board, 6, 20, 100, 100, 40, 10, 11600, 1, 2800)
    assert '达到停车阈值' in point['error']
    assert point['peak_current_ma'] == 2800
    assert board.stops == 1

"""使用方法：由 wheel_command_guard 载入四轮正反向实测点，反查目标轮速对应的命令。"""

import bisect
import math


WHEEL_NAMES = ('fl', 'fr', 'rl', 'rr')
DIRECTIONS = ('forward', 'reverse')


class WheelCurveModel:
    """对每个轮和方向使用单调折线反查，拒绝缺失或不可信的标定。"""

    def __init__(self, curves, maximum_command_rad_s, max_compensation_ratio):
        self.maximum = float(maximum_command_rad_s)
        self.max_ratio = float(max_compensation_ratio)
        if not math.isfinite(self.maximum) or self.maximum <= 0:
            raise ValueError('最大轮速命令必须为正且有限')
        if not math.isfinite(self.max_ratio) or self.max_ratio < 1:
            raise ValueError('最大补偿倍数必须不小于 1 且有限')
        self.curves = {}
        for wheel in WHEEL_NAMES:
            for direction in DIRECTIONS:
                try:
                    commands, measured = curves[wheel][direction]
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(f'{wheel}/{direction} 标定缺失') from error
                commands = tuple(float(value) for value in commands)
                measured = tuple(float(value) for value in measured)
                if len(commands) < 2 or len(commands) != len(measured):
                    raise ValueError(f'{wheel}/{direction} 至少需要两对同长度实测点')
                if any(not math.isfinite(value) or value <= 0 for value in commands + measured):
                    raise ValueError(f'{wheel}/{direction} 标定点必须为正且有限')
                if any(a >= b for a, b in zip(commands, commands[1:])) or any(
                        a >= b for a, b in zip(measured, measured[1:])):
                    raise ValueError(f'{wheel}/{direction} 标定点必须严格递增')
                if commands[-1] > self.maximum:
                    raise ValueError(f'{wheel}/{direction} 标定命令超过最大轮速')
                self.curves[wheel, direction] = (commands, measured)

    def compensate(self, wheel, target):
        """输入目标 rad/s，输出同方向且限幅的命令 rad/s。"""
        target = float(target)
        if not math.isfinite(target):
            raise ValueError('轮速目标必须有限')
        if target == 0.0:
            return 0.0
        direction = 'forward' if target > 0 else 'reverse'
        commands, measured = self.curves[wheel, direction]
        magnitude = abs(target)
        index = bisect.bisect_left(measured, magnitude)
        if index == 0:
            low_output, low_command = 0.0, 0.0
        else:
            low_output, low_command = measured[index - 1], commands[index - 1]
        if index == len(measured):
            # 不外推未实测的工作区间。
            command = commands[-1]
        else:
            high_output, high_command = measured[index], commands[index]
            command = low_command + (magnitude - low_output) * (
                high_command - low_command) / (high_output - low_output)
        command = min(command, magnitude * self.max_ratio, self.maximum)
        return math.copysign(command, target)

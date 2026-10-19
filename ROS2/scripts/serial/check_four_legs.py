#!/usr/bin/env python3
"""用途：独占 DMC0 串口，按 --values 指定的角度逐腿发送 R0～R3；架空后运行 --confirm-suspended。"""

import argparse
import fcntl
import math
import os
from pathlib import Path
import sys
import termios
import time

import yaml

from single_motor_curve import open_serial, send_command


NAMES = ('FL', 'FR', 'RL', 'RR')
SIGNS = (-1, 1, -1, 1)
DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / 'src/robot_hardware/config/check_four_legs.yaml'


class ProbeError(RuntimeError):
    """板卡、反馈或运动检查失败。"""


def load_config(path):
    """读取硬件检查参数并拒绝不合理配置；角度目标由命令行指定。"""
    config = yaml.safe_load(Path(path).read_text(encoding='utf-8'))['check_four_legs']
    for key in ('current_limit_ma', 'tolerance_deg', 'minimum_motion_deg',
                'other_leg_tolerance_deg', 'motion_timeout_s', 'sample_period_s',
                'response_timeout_s', 'idle_timeout_s'):
        if config[key] <= 0:
            raise ValueError(f'{key} 必须大于零')
    return config


def parse_degrees(value):
    """按命令行填写顺序解析逗号分隔的 ROS 关节角度。"""
    try:
        degrees = [float(part.strip()) for part in value.split(',')]
    except ValueError as exc:
        raise argparse.ArgumentTypeError('角度须为逗号分隔的数字') from exc
    if not degrees or any(not math.isfinite(degree) for degree in degrees):
        raise argparse.ArgumentTypeError('角度不能为空或包含非有限数字')
    return degrees


def parse_state(fields):
    """将 DMC0 四腿 R 回包转为 ROS 角度、电流；不把速度字段当错误码。"""
    if len(fields) != 13 or fields[0] != 'R':
        raise ProbeError(f'四腿 R 回包格式错误：{fields!r}')
    try:
        values = [int(value) for value in fields[1:]]
    except ValueError as exc:
        raise ProbeError(f'四腿 R 回包含非整数：{fields!r}') from exc
    return [dict(angle_deg=SIGNS[i] * values[3 * i] / 100,
                 speed_deg_s=SIGNS[i] * values[3 * i + 1],
                 current_ma=values[3 * i + 2]) for i in range(4)]


class Dmc0:
    """只在 $info 精确识别 DMC0 后允许发位置命令。"""

    def __init__(self, candidates, timeout, idle):
        self.fd = None
        self.device = None
        self.last_state = None
        self.timeout = timeout
        self.idle = idle
        for device in candidates:
            fd = None
            try:
                fd = open_serial(device, termios.B115200)
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except (OSError, ValueError):
                if fd is not None:
                    os.close(fd)
                continue
            try:
                if self._exchange(fd, '$info') == '$info DMC 4 0 1 2 3':
                    self.fd, self.device = fd, device
                    break
            except (OSError, ProbeError):
                pass
            os.close(fd)
        if self.fd is None:
            raise ProbeError('未找到独占可用的 DMC0（$info 应为 0 1 2 3）')

    def _exchange(self, fd, command):
        raw = send_command(fd, command, self.timeout, self.idle)
        lines = [line.strip() for line in raw.decode('ascii', 'replace').splitlines()
                 if line.strip()]
        if len(lines) != 1 or lines[0].startswith('!'):
            raise ProbeError(f'{command!r} 响应无效：{lines!r}')
        return lines[0]

    def exchange(self, command, prefix, count):
        fields = self._exchange(self.fd, command).split()
        if len(fields) != count or fields[0] != prefix:
            raise ProbeError(f'{command!r} 响应格式错误：{fields!r}')
        return fields

    def read_state(self):
        self.last_state = parse_state(self.exchange('R ', 'R', 13))
        return self.last_state

    def target(self, index, angle_deg):
        raw = round(SIGNS[index] * angle_deg * 100)
        if not -32768 <= raw <= 32767:
            raise ProbeError(f'R{index} 目标角度无法编码为 int16：{angle_deg}°')
        # 使用方法：仅确认单轴回包格式；板端错误码留给位置反馈检查，不因该字段终止测试。
        self.exchange(f'R{index} {raw}', f'R{index}', 5)

    def hold(self, state):
        """异常时只保持已观测位置，不自动归零或继续下一腿。"""
        errors = []
        for i in range(4):
            try:
                self.target(i, state[i]['angle_deg'])
            except (OSError, ProbeError) as exc:
                errors.append(f'{NAMES[i]}: {exc}')
        if errors:
            raise ProbeError('保持当前角度失败：' + '; '.join(errors))

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def check_state(state, config, baseline, active):
    """每次轮询均检查四条腿的电流和非目标腿漂移。"""
    for i, leg in enumerate(state):
        if abs(leg['current_ma']) >= config['current_limit_ma']:
            raise ProbeError(f'{NAMES[i]} 电流 {leg["current_ma"]} mA 达到阈值')
        if i != active and abs(leg['angle_deg'] - baseline[i]['angle_deg']) > config['other_leg_tolerance_deg']:
            raise ProbeError(f'非目标腿 {NAMES[i]} 意外移动')


def move(board, index, target, config):
    """发送单腿目标，直到到位或超时；异常由外层保持当时位置。"""
    baseline = board.read_state()
    check_state(baseline, config, baseline, index)
    start = baseline[index]['angle_deg']
    board.target(index, target)
    deadline = time.monotonic() + config['motion_timeout_s']
    best_motion = 0.0
    while True:
        state = board.read_state()
        check_state(state, config, baseline, index)
        actual = state[index]['angle_deg']
        best_motion = max(best_motion, abs(actual - start))
        if abs(actual - target) <= config['tolerance_deg']:
            if abs(target - start) >= config['minimum_motion_deg'] and best_motion < config['minimum_motion_deg']:
                raise ProbeError(f'{NAMES[index]} 未观察到足够位移')
            print(f'{NAMES[index]} {start:+.2f}° → {actual:+.2f}°，目标 {target:+.2f}°，'
                  f'电流 {state[index]["current_ma"]} mA', flush=True)
            return state
        if time.monotonic() >= deadline:
            raise ProbeError(f'{NAMES[index]} 未到位：目标 {target:+.2f}°，实际 {actual:+.2f}°，'
                             f'观测位移 {best_motion:.2f}°')
        time.sleep(config['sample_period_s'])


def run(board, config, legs, values):
    """每条指定腿按原样执行用户填写的绝对 ROS 角度序列。"""
    state = board.read_state()
    check_state(state, config, state, -1)
    for index in legs:
        for target in values:
            move(board, index, target, config)
            time.sleep(config['pause_s'])


def main():
    parser = argparse.ArgumentParser(description='DMC0 四腿直接串口小角度运动检查')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG, help='检查参数 YAML')
    parser.add_argument('--device', help='指定串口；仍须通过 $info 识别 DMC0')
    parser.add_argument('--legs', nargs='+', choices=NAMES, default=NAMES, help='按顺序检查的腿')
    parser.add_argument('--values', type=parse_degrees, required=True,
                        help='逗号分隔的绝对 ROS 角度，按填写顺序发送；负数请写 --values=-2,0,2')
    parser.add_argument('--confirm-suspended', action='store_true', help='确认实机固定架空且串口独占')
    args = parser.parse_args()
    if not args.confirm_suspended:
        parser.error('运动前须确认实机固定架空、ROS/vctrl 已停止，并添加 --confirm-suspended')
    board = None
    try:
        config = load_config(args.config)
        candidates = [args.device] if args.device else config['device_candidates']
        board = Dmc0(candidates, config['response_timeout_s'], config['idle_timeout_s'])
        print(f'已识别 DMC0：{board.device}；将按 {", ".join(args.legs)} 检查', flush=True)
        run(board, config, [NAMES.index(name) for name in args.legs], args.values)
        print('四腿检查完成', flush=True)
        return 0
    except (OSError, ValueError, KeyError, ProbeError, KeyboardInterrupt) as exc:
        print(f'检查中止：{exc or type(exc).__name__}', file=sys.stderr)
        if board is not None:
            try:
                board.hold(board.last_state or board.read_state())
            except (OSError, ValueError, ProbeError) as hold_exc:
                print(f'无法确认保持当前角度：{hold_exc}', file=sys.stderr)
        return 1
    finally:
        if board is not None:
            board.close()


if __name__ == '__main__':
    sys.exit(main())

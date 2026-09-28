#!/usr/bin/env python3
"""用途：独占 DMC1 串口采集单轮速度和电流；四轮架空后运行 record，或用 plot 绘图。"""

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import statistics
import struct
import termios
import time

IDS = (4, 5, 6, 7)
NAMES = ('FL', 'FR', 'RL', 'RR')


class ProtocolError(RuntimeError):
    """串口响应或板卡身份不符合预期。"""


def signed_delta(new, old):
    """计算考虑 32 位编码器回绕的有符号位移。"""
    return ((new - old + 2 ** 31) % (2 ** 32)) - 2 ** 31


def open_serial(device, baud):
    """以 8N1 无流控方式打开串口，并拉高 DTR/RTS。"""
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = termios.IGNBRK
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = attrs[5] = baud
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        fcntl.ioctl(fd, termios.TIOCMBIS,
                    struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd
    except Exception:
        os.close(fd)
        raise


def send_command(fd, command, timeout, idle):
    """完整发送一条命令，按首字节和空闲超时读取响应。"""
    termios.tcflush(fd, termios.TCIFLUSH)
    payload = memoryview(command.encode('ascii') + b'\n')
    while payload:
        try:
            written = os.write(fd, payload)
        except BlockingIOError:
            if not select.select([], [fd], [], timeout)[1]:
                raise TimeoutError('串口写入超时')
            continue
        if written == 0:
            raise OSError('串口写入返回 0 字节')
        payload = payload[written:]
    termios.tcdrain(fd)
    response = bytearray()
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([fd], [], [], remaining)[0]:
            break
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            break
        response.extend(chunk)
        deadline = time.monotonic() + idle
    return bytes(response)


class Dmc1:
    """只操作经 $info 验证的四轮 DMC1；关闭时逐轴停车。"""

    def __init__(self, candidates, timeout, idle, stop_duration_ms=500, baud=termios.B115200):
        self.fd = None
        self.device = None
        self.timeout = timeout
        self.idle = idle
        self.stop_duration_ms = stop_duration_ms
        for device in candidates:
            try:
                fd = open_serial(device, baud)
            except (OSError, ValueError):
                continue
            try:
                response = self._exchange_fd(fd, '$info')
                if response == '$info DMC 4 4 5 6 7':
                    self.fd, self.device = fd, device
                    break
            except (OSError, ProtocolError):
                pass
            os.close(fd)
        if self.fd is None:
            raise ProtocolError('未找到电机 ID 为 4、5、6、7 的 DMC1')

    def _exchange_fd(self, fd, command):
        raw = send_command(fd, command, self.timeout, self.idle)
        lines = [line.strip() for line in raw.decode('ascii', 'replace').splitlines()
                 if line.strip()]
        if len(lines) != 1 or lines[0].startswith('!'):
            raise ProtocolError(f'{command!r} 的响应无效：{lines!r}')
        return lines[0]

    def exchange(self, command, prefix, count):
        fields = self._exchange_fd(self.fd, command).split()
        if len(fields) != count or fields[0] != prefix:
            raise ProtocolError(f'{command!r} 的响应字段错误：{fields!r}')
        return fields

    def stop(self):
        failures = []
        for motor_id in IDS:
            command = f'S{motor_id} 0 {self.stop_duration_ms}'
            try:
                fields = self.exchange(command, f'S{motor_id}', 5)
                if int(fields[4]) != 0:
                    failures.append(f'S{motor_id} 错误码 {fields[4]}')
            except (OSError, ValueError, ProtocolError) as exc:
                failures.append(f'S{motor_id}: {exc}')
        if failures:
            raise ProtocolError('逐轴停车失败：' + '; '.join(failures))

    def close(self):
        if self.fd is not None:
            try:
                self.stop()
            finally:
                os.close(self.fd)
                self.fd = None

    def read_state(self):
        fields = self.exchange('S ', 'S', 13)
        return [dict(tps=int(fields[1 + 3 * index]),
                     encoder=int(fields[2 + 3 * index], 16),
                     current_ma=int(fields[3 + 3 * index]))
                for index in range(4)]


def parse_values(value):
    """按用户给定顺序解析测试点，不自动增减或筛选。"""
    try:
        values = [int(part.strip()) for part in value.split(',')]
    except ValueError as exc:
        raise argparse.ArgumentTypeError('测试点应为逗号分隔的整数') from exc
    if not values:
        raise argparse.ArgumentTypeError('至少填写一个测试点')
    return values


def save_report(report, path):
    """每点落盘；中断后保留此前采样。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n',
                         encoding='utf-8')
    temporary.replace(path)


def record_point(board, motor_id, mode, value, duration_ms, sample_ms, cpr, sign):
    """发送一条有限时长命令，记录四轮状态；任何退出路径都逐轮停车。"""
    index = IDS.index(motor_id)
    command = f'{mode}{motor_id} {value} {duration_ms}'
    point = {'command': command, 'value': value, 'duration_ms': duration_ms,
             'response': None, 'samples': [], 'error': ''}
    try:
        baseline = board.read_state()[index]['encoder']
        fields = board.exchange(command, f'{mode}{motor_id}', 4 if mode == 'D' else 5)
        point['response'] = fields
        if mode == 'S' and int(fields[4]) != 0:
            raise ProtocolError(f'{command} 返回错误码 {fields[4]}')
        start = time.monotonic()
        while True:
            wheels = board.read_state()
            elapsed = time.monotonic() - start
            point['samples'].append({'time_s': round(elapsed, 4), 'wheels': wheels})
            if elapsed >= duration_ms / 1000:
                break
            time.sleep(min(sample_ms / 1000, max(0, duration_ms / 1000 - elapsed)))
        tail = point['samples'][len(point['samples']) // 2:]
        point['speed_rad_s'] = round(statistics.mean(
            item['wheels'][index]['tps'] * sign * 2 * math.pi / cpr for item in tail), 5)
        point['peak_current_ma'] = max(item['wheels'][index]['current_ma']
                                       for item in point['samples'])
        point['mean_current_ma'] = round(statistics.mean(
            item['wheels'][index]['current_ma'] for item in tail), 1)
        point['encoder_delta_ticks'] = signed_delta(
            point['samples'][-1]['wheels'][index]['encoder'], baseline)
    except (OSError, ValueError, ProtocolError, KeyboardInterrupt) as exc:
        point['error'] = str(exc) or type(exc).__name__
    finally:
        try:
            board.stop()
        except (OSError, ValueError, ProtocolError) as exc:
            point['error'] += f'；停车失败：{exc}'
    return point


def record(args):
    """仅测指定电机和测试点，不按轮速、电流或稳定性筛选结果。"""
    report = {'schema_version': 1, 'motor_id': args.motor, 'wheel': NAMES[IDS.index(args.motor)],
              'mode': args.mode, 'device': None, 'cpr': None, 'encoder_direction': None,
              'duration_ms': args.duration_ms, 'sample_ms': args.sample_ms,
              'pause_ms': args.pause_ms, 'points': [], 'error': ''}
    board = None
    try:
        candidates = [args.device] if args.device else [f'/dev/ttyACM{i}' for i in range(9)]
        board = Dmc1(candidates, args.response_timeout, args.idle_timeout,
                     args.stop_duration_ms)
        report['device'] = board.device
        fields = board.exchange(f'I{args.motor} ', f'I{args.motor}', 3)
        cpr, direction = int(fields[1]), int(fields[2])
        if cpr <= 0 or direction not in (-1, 1):
            raise ProtocolError(f'I{args.motor} 的 CPR 或方向无效')
        report['cpr'], report['encoder_direction'] = cpr, direction
        maximum_ms = int(board.exchange(f'J{args.motor}', f'J{args.motor}', 5)[4])
        if args.duration_ms > maximum_ms:
            raise ValueError(f'板端 J{args.motor} 单命令最长 {maximum_ms}ms')
        board.stop()
        for value in args.values:
            point = record_point(board, args.motor, args.mode, value,
                                 args.duration_ms, args.sample_ms, cpr, -direction)
            report['points'].append(point)
            save_report(report, args.output)
            print(f'{point["command"]}: 速度 {point.get("speed_rad_s", "无反馈")} rad/s，'
                  f'峰值电流 {point.get("peak_current_ma", "无反馈")} mA'
                  + (f'；{point["error"]}' if point['error'] else ''), flush=True)
            if point['error']:
                report['error'] = point['error']
                break
            time.sleep(args.pause_ms / 1000)
    except (OSError, ValueError, ProtocolError, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or type(exc).__name__
    finally:
        if board is not None:
            try:
                board.close()
            except (OSError, ValueError, ProtocolError) as exc:
                report['error'] += f'；最终停车失败：{exc}'
        save_report(report, args.output)
    if any('speed_rad_s' in point for point in report['points']):
        try:
            image = args.image or str(Path(args.output).with_suffix('.png'))
            plot(args.output, image)
            print(f'曲线图：{image}', flush=True)
        except (ImportError, OSError, ValueError) as exc:
            report['error'] += f'；绘图失败：{exc}'
            save_report(report, args.output)
    return 2 if report['error'] else 0


def plot(report_path, output_path):
    """以命令值为横轴，绘制实测轮速、峰值和后半段平均电流。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    report = json.loads(Path(report_path).read_text(encoding='utf-8'))
    points = [point for point in report['points'] if 'speed_rad_s' in point]
    if not points:
        raise ValueError('报告没有可绘制的采样点')
    x = [point['value'] for point in points]
    figure, (speed, current) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    speed.plot(x, [point['speed_rad_s'] for point in points], 'o-', label='Measured')
    if report['mode'] == 'S':
        speed.plot(x, [value * 2 * math.pi / report['cpr'] for value in x], '--',
                   label='Target')
    speed.set_ylabel('Wheel speed (rad/s)')
    speed.legend()
    current.plot(x, [point['peak_current_ma'] for point in points], 's-',
                 label='Observed peak')
    current.plot(x, [point['mean_current_ma'] for point in points], '^-',
                 label='Second-half mean')
    current.set_ylabel('Current (mA)')
    current.set_xlabel('D PWM (%)' if report['mode'] == 'D' else 'S target (tick/s)')
    current.legend()
    for axis in (speed, current):
        axis.grid(True, alpha=0.25)
    figure.suptitle(f'DMC1 {report["wheel"]} {report["mode"]} response')
    figure.tight_layout()
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=170)
    plt.close(figure)


def main():
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt('收到终止信号')

    signal.signal(signal.SIGTERM, interrupt)
    parser = argparse.ArgumentParser(description='DMC1 单电机 D/S 输入、轮速和电流记录')
    commands = parser.add_subparsers(dest='action', required=True)
    recording = commands.add_parser('record', help='逐点发送命令并记录 JSON')
    recording.add_argument('--motor', type=int, choices=IDS, required=True,
                           help='4 左前，5 右前，6 左后，7 右后')
    recording.add_argument('--mode', choices=('D', 'S'), required=True)
    recording.add_argument('--values', type=parse_values, required=True,
                           help='逗号分隔的 PWM 百分比或 tick/s，按填写顺序测试')
    recording.add_argument('--duration-ms', type=int, default=600)
    recording.add_argument('--sample-ms', type=int, default=50)
    recording.add_argument('--pause-ms', type=int, default=500)
    recording.add_argument('--device', help='指定 DMC1 串口；默认扫描 /dev/ttyACM0..8')
    recording.add_argument('--response-timeout', type=float, default=0.35,
                           help='等待首个响应字节的秒数')
    recording.add_argument('--idle-timeout', type=float, default=0.015,
                           help='响应字节间空闲超时秒数')
    recording.add_argument('--stop-duration-ms', type=int, default=500,
                           help='逐轴零速命令时长，必须在 1～1000ms')
    recording.add_argument('--output', required=True)
    recording.add_argument('--image', help='PNG 路径；默认与 JSON 同名')
    drawing = commands.add_parser('plot', help='从单轮 JSON 绘制速度、电流 PNG')
    drawing.add_argument('report')
    drawing.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.action == 'record':
        if args.mode == 'D' and any(abs(value) > 100 for value in args.values):
            parser.error('D 命令的协议范围是 -100～100%')
        if (not 0 < args.duration_ms <= 1000 or args.sample_ms <= 0 or args.pause_ms < 0
                or not 0 < args.stop_duration_ms <= 1000 or args.response_timeout <= 0
                or args.idle_timeout <= 0):
            parser.error('命令时长须在 1～1000ms，采样、响应超时须为正，暂停可以为零')
        return record(args)
    plot(args.report, args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

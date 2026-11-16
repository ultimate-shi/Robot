#!/usr/bin/env python3
"""使用方法：架空车轮并停止 ROS/vctrl 后，指定 --motor 和 --encoder-sign 采集单轴 D 模式曲线。"""

import argparse
import math
import statistics
import time
from pathlib import Path

from single_motor_curve import Dmc1, IDS, ProtocolError, parse_values, save_report, signed_delta


def measure(board, motor_id, value, duration_ms, hold_ms, renew_ms, sample_ms,
            cpr, encoder_sign, current_limit_ma):
    """记录指定轴的原始方向与速度；任一异常退出时都先停车。"""
    index = IDS.index(motor_id)
    prefix = f'D{motor_id}'
    point = {'command': f'{prefix} {value} {duration_ms}', 'value': value,
             'duration_ms': duration_ms, 'hold_ms': hold_ms,
             'renew_ms': renew_ms, 'samples': [], 'error': ''}
    try:
        baseline = board.read_state()[index]['encoder']
        point['response'] = board.exchange(point['command'], prefix, 4)
        start = time.monotonic()
        next_renew = start + renew_ms / 1000
        while time.monotonic() - start < hold_ms / 1000:
            if time.monotonic() >= next_renew and hold_ms > duration_ms:
                board.exchange(point['command'], prefix, 4)
                next_renew += renew_ms / 1000
            state = board.read_state()[index]
            elapsed = time.monotonic() - start
            point['samples'].append({'time_s': round(elapsed, 4), **state})
            if abs(state['current_ma']) >= current_limit_ma:
                raise ProtocolError(f'电流 {state["current_ma"]}mA 达到停车阈值')
            time.sleep(min(sample_ms / 1000, max(0, hold_ms / 1000 - elapsed)))
        if not point['samples']:
            raise ProtocolError('无轮速反馈')
        tail = point['samples'][len(point['samples']) // 2:]
        mean_tps = statistics.mean(sample['tps'] for sample in tail)
        point['mean_tps'] = round(mean_tps, 3)
        point['speed_rad_s'] = round(mean_tps * encoder_sign * 2 * math.pi / cpr, 5)
        point['encoder_delta_ticks'] = signed_delta(
            point['samples'][-1]['encoder'], baseline)
        point['peak_current_ma'] = max(sample['current_ma'] for sample in point['samples'])
        point['speed_span_rad_s'] = round(
            (max(sample['tps'] for sample in tail) - min(sample['tps'] for sample in tail))
            * 2 * math.pi / cpr, 5)
        if mean_tps and point['encoder_delta_ticks'] and (mean_tps > 0) != (
                point['encoder_delta_ticks'] > 0):
            raise ProtocolError('速度与编码器计数方向不一致')
    except (OSError, ValueError, ProtocolError, KeyboardInterrupt) as exc:
        point['error'] = str(exc) or type(exc).__name__
        if point['samples']:
            point['peak_current_ma'] = max(
                sample['current_ma'] for sample in point['samples'])
    finally:
        try:
            board.stop()
        except (OSError, ValueError, ProtocolError) as exc:
            point['error'] += f'；停车失败：{exc}'
    return point


def main():
    parser = argparse.ArgumentParser(description='DMC1 单轮 D 模式正反向限流曲线采样')
    parser.add_argument('--motor', required=True, type=int, choices=IDS,
                        help='DMC1 电机轴号 4～7；按轮位配置核对')
    parser.add_argument('--encoder-sign', required=True, type=int, choices=(-1, 1),
                        help='该轮位 ROS 编码器方向符号，来自硬件配置，不由 I 参数猜测')
    parser.add_argument('--values', required=True, type=parse_values,
                        help='按顺序测试的带符号 PWM 百分比，逗号分隔')
    parser.add_argument('--duration-ms', type=int, default=1000, help='单点时长，单位毫秒')
    parser.add_argument('--hold-ms', type=int, default=1000,
                        help='每点总采样时长；超过单条命令时周期续发，单位毫秒')
    parser.add_argument('--renew-ms', type=int, default=400,
                        help='续发 D 指令的周期，单位毫秒')
    parser.add_argument('--sample-ms', type=int, default=20, help='状态查询间隔，单位毫秒')
    parser.add_argument('--current-stop-ma', required=True, type=int,
                        help='现场确认的该轴软件停车阈值，达到时立即停车，单位 mA')
    parser.add_argument('--pause-ms', type=int, default=800, help='两点间停车间隔，单位毫秒')
    parser.add_argument('--output', required=True, type=Path, help='原始采样 JSON 路径')
    args = parser.parse_args()
    if any(value == 0 or abs(value) > 100 for value in args.values):
        parser.error('--values 必须是非零且绝对值不超过 100 的 PWM 百分比')
    if not 100 <= args.duration_ms <= 1000 or \
            not args.duration_ms <= args.hold_ms <= 3000 or \
            not 100 <= args.renew_ms <= args.duration_ms - 100 or \
            not 10 <= args.sample_ms <= 100 or \
            not 100 <= args.pause_ms or not 500 <= args.current_stop_ma <= 3200:
        parser.error('时长、采样间隔、停车间隔或电流阈值超出安全范围')
    report = {'schema_version': 1, 'motor_id': args.motor, 'mode': 'D',
              'direction_basis': 'raw tps, encoder delta, ROS encoder_sign',
              'encoder_sign': args.encoder_sign,
              'current_stop_ma': args.current_stop_ma, 'points': [], 'error': ''}
    board = None
    try:
        board = Dmc1([f'/dev/ttyACM{index}' for index in range(9)], 0.5, 0.03)
        report['device'] = board.device
        cpr_fields = board.exchange(f'I{args.motor} ', f'I{args.motor}', 3)
        report['cpr'] = int(cpr_fields[1])
        report['board_encoder_direction'] = int(cpr_fields[2])
        report['joint_config'] = board._exchange_fd(board.fd, f'J{args.motor} ')
        report['pid_config'] = board._exchange_fd(board.fd, f'P{args.motor} ')
        if report['cpr'] <= 0 or abs(report['board_encoder_direction']) != 1:
            raise ProtocolError(f'I{args.motor} 编码器配置无效')
        if int(report['joint_config'].split()[-1]) < args.duration_ms:
            raise ProtocolError(f'测试时长超过 J{args.motor} 板端上限')
        board.stop()
        for value in args.values:
            point = measure(board, args.motor, value, args.duration_ms, args.hold_ms,
                            args.renew_ms,
                            args.sample_ms,
                            report['cpr'], args.encoder_sign, args.current_stop_ma)
            report['points'].append(point)
            save_report(report, args.output)
            print(point['command'], 'ROS速度', point.get('speed_rad_s'),
                  '原始tps', point.get('mean_tps'),
                  '编码器增量', point.get('encoder_delta_ticks'),
                  '峰值电流', point.get('peak_current_ma'),
                  point['error'], flush=True)
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
    return 2 if report['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())

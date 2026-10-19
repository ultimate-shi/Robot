#!/usr/bin/env python3
"""用途：架空实机逐段验证三种底盘模式的轮速与转向反馈。

使用方法：启动实机 controllers、chassis_control、safety 后，在 Jazzy 容器内运行
python3 /workspace/src/robot_control/test/chassis_mode_hardware_probe.py --output /tmp/chassis_probe.json
脚本只发布 /cmd_vel_nav_smoothed，经现有门控和安全链控制底盘；异常及退出时发送零速。
"""

import argparse
import json
import statistics
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from robot_interfaces.msg import ChassisState


STEER = [f'{place}_steer_joint' for place in
         ('front_left', 'front_right', 'rear_left', 'rear_right')]
WHEEL = [f'{place}_wheel_joint' for place in
         ('front_left', 'front_right', 'rear_left', 'rear_right')]
CASES = [
    ('four_ws', 'forward', .08, 0., 0.),
    ('four_ws', 'reverse', -.08, 0., 0.),
    ('four_ws', 'left_arc', .08, 0., .10),
    ('four_ws', 'right_arc', .08, 0., -.10),
    ('four_ws', 's_left', .08, 0., .10),
    ('four_ws', 's_right', .08, 0., -.10),
    ('four_ws', 'reverse_left', -.08, 0., .10),
    ('four_ws', 'reverse_right', -.08, 0., -.10),
    ('crab', 'diagonal_left', .08, .03, 0.),
    ('crab', 'diagonal_right', .08, -.03, 0.),
    ('crab', 'diagonal_reverse_left', -.08, .03, 0.),
    ('crab', 'diagonal_reverse_right', -.08, -.03, 0.),
    ('ackermann', 'forward_left', .08, 0., .10),
    ('ackermann', 'forward_right', .08, 0., -.10),
    ('ackermann', 'reverse_left', -.08, 0., .10),
    ('ackermann', 'reverse_right', -.08, 0., -.10),
]


class Probe(Node):
    def __init__(self, current_limit):
        super().__init__('chassis_mode_hardware_probe')
        self.publisher = self.create_publisher(Twist, '/cmd_vel_nav_smoothed', 10)
        self.parameter_client = AsyncParameterClient(self, '/chassis_controller')
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.create_subscription(ChassisState, '/hardware/chassis_state', self.on_hardware, 10)
        self.create_subscription(Float64MultiArray, '/wheel_controller/commands_raw',
                                 self.on_wheel_command, 10)
        self.create_subscription(Float64MultiArray, '/steering_controller/commands',
                                 self.on_steer_command, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.on_safe, 10)
        self.current_limit = current_limit
        self.hardware = None
        self.joint = None
        self.wheel_command = None
        self.steer_command = None
        self.safe = None
        self.samples = []
        self.abort_reason = ''

    def on_joint(self, message):
        values = dict(zip(message.name, zip(message.position, message.velocity)))
        if all(name in values for name in STEER + WHEEL):
            self.joint = ([values[name][0] for name in STEER],
                          [values[name][1] for name in WHEEL])
            if self.wheel_command is not None and self.steer_command is not None:
                self.samples.append({
                    'time': time.monotonic(),
                    'steer': self.joint[0], 'wheel': self.joint[1],
                    'steer_target': self.steer_command,
                    'wheel_target': self.wheel_command,
                    'safe': self.safe,
                    'current_ma': list(self.hardware.wheel_current_ma)
                    if self.hardware is not None else None,
                })

    def on_hardware(self, message):
        self.hardware = message
        if (not message.outputs_enabled or message.manual_recovery_required or
                not message.watchdog_ok or message.fault):
            self.abort_reason = f'hardware fault: {message.fault}'
        if max(message.wheel_current_ma) >= self.current_limit:
            self.abort_reason = f'wheel current >= {self.current_limit} mA'

    def on_wheel_command(self, message):
        if len(message.data) == 4:
            self.wheel_command = list(message.data)

    def on_steer_command(self, message):
        if len(message.data) == 4:
            self.steer_command = list(message.data)

    def on_safe(self, message):
        self.safe = [message.linear.x, message.linear.y, message.angular.z]

    def publish(self, vx=0., vy=0., wz=0.):
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self.publisher.publish(message)

    def tick(self, duration, velocity=(0., 0., 0.)):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish(*velocity)
            rclpy.spin_once(self, timeout_sec=.08)
            if self.abort_reason:
                raise RuntimeError(self.abort_reason)

    def set_mode(self, mode):
        for _ in range(3):
            future = self.parameter_client.set_parameters([
                Parameter('motion_mode', value=mode)])
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.)
            if future.done() and future.result() and all(
                    item.successful for item in future.result().results):
                return
        raise RuntimeError(f'无法切换到 {mode} 模式: {future.result() if future.done() else "超时"}')


def summarize(samples, start, end):
    steady = [item for item in samples if start + 2. <= item['time'] <= end]
    if not steady:
        return {'pass': False, 'reason': '没有稳态反馈'}
    median = lambda key, index: statistics.median(item[key][index] for item in steady)
    target = [median('wheel_target', i) for i in range(4)]
    actual = [median('wheel', i) for i in range(4)]
    steer_target = [median('steer_target', i) for i in range(4)]
    steer = [median('steer', i) for i in range(4)]
    safe = [median('safe', i) if all(item['safe'] is not None for item in steady)
            else None for i in range(3)]
    matched = [abs(actual[i] - target[i]) <= max(.35, .5 * abs(target[i]))
               for i in range(4)]
    steering_matched = [abs(steer[i] - steer_target[i]) <= .12 for i in range(4)]
    active_seconds = sum(
        min(.25, steady[i]['time'] - steady[i - 1]['time'])
        for i in range(1, len(steady))
        if all(abs(value) > .3 for value in steady[i]['wheel']))
    command_active_fraction = sum(
        all(abs(value) > .3 for value in item['wheel_target'])
        for item in steady) / len(steady)
    maximum_steer_error = max(
        max(abs(actual - target) for actual, target in
            zip(item['steer'], item['steer_target'])) for item in steady)
    peak_current = max((max(item['current_ma']) for item in steady
                        if item['current_ma'] is not None), default=None)
    return {
        'pass': all(matched + steering_matched) and active_seconds >= 5.,
        'samples': len(steady), 'active_seconds': round(active_seconds, 2),
        'command_active_fraction': round(command_active_fraction, 3),
        'maximum_steer_error_rad': round(maximum_steer_error, 3),
        'peak_wheel_current_ma': peak_current,
        'wheel_target_rad_s': target, 'wheel_actual_rad_s': actual,
        'steer_target_rad': steer_target, 'steer_actual_rad': steer,
        'safe_twist': safe,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/chassis_probe.json')
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--current-limit-ma', type=float, default=3200.,
                        help='任一车轮电流达到此值即终止，单位 mA')
    parser.add_argument('--cases', default='', help='逗号分隔的案例名称；默认全部')
    parser.add_argument('--include-samples', action='store_true',
                        help='在报告中保留每帧目标、反馈和电流，便于排查瞬时停轮')
    args = parser.parse_args()
    selected = set(args.cases.split(',')) if args.cases else None
    rclpy.init()
    node = Probe(args.current_limit_ma)
    report = []
    try:
        discovery_deadline = time.monotonic() + 10.
        while (node.hardware is None or node.joint is None) and time.monotonic() < discovery_deadline:
            node.tick(.2)
        if node.hardware is None or node.joint is None:
            raise RuntimeError('缺少硬件或四轮关节反馈')
        previous_case = None
        for mode, name, vx, vy, wz in CASES:
            if selected is not None and name not in selected and f'{mode}:{name}' not in selected:
                continue
            continuous = previous_case in (
                ('four_ws', 's_left'), ('four_ws', 'reverse_left')) and (
                mode, name) in (('four_ws', 's_right'), ('four_ws', 'reverse_right'))
            if not continuous:
                node.tick(1.)
            if previous_case is None or previous_case[0] != mode:
                node.set_mode(mode)
            node.samples.clear()
            start = time.monotonic()
            node.tick(args.duration, (vx, vy, wz))
            end = time.monotonic()
            result = summarize(node.samples, start, end)
            result.update(mode=mode, name=name, command=[vx, vy, wz])
            if args.include_samples:
                result['trace'] = list(node.samples)
            report.append(result)
            print(json.dumps({key: value for key, value in result.items()
                              if key != 'trace'}, ensure_ascii=False), flush=True)
            previous_case = (mode, name)
    except Exception as error:
        report.append({'pass': False, 'reason': str(error)})
        print(f'中止: {error}', flush=True)
    finally:
        node.abort_reason = ''
        for _ in range(20):
            node.publish()
            rclpy.spin_once(node, timeout_sec=.05)
        with open(args.output, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report and all(item.get('pass') for item in report) else 1


if __name__ == '__main__':
    raise SystemExit(main())

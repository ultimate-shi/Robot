#!/usr/bin/env python3
"""使用方法：架空实机并启动硬件控制器后，在 Jazzy 容器运行本脚本逐腿验证软件端点。"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from robot_interfaces.msg import ChassisState
from robot_interfaces.srv import SetBodyBalance
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


NAMES = ('fl', 'fr', 'rl', 'rr')
JOINTS = tuple(f'lap_body_joint_{name}' for name in NAMES)
TOPICS = tuple(f'/lap_{name}_position_controller/commands' for name in NAMES)
WHEELS = ('front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint')


class LegProbe(Node):
    """只发布单腿位置命令，并在每个等待周期检查实机反馈。"""

    def __init__(self, settings):
        super().__init__('leg_range_hardware_probe')
        self.settings = settings
        self.joints = None
        self.hardware = None
        self.joint_time = 0.0
        self.hardware_time = 0.0
        self.wheel_target = None
        self.wheel_target_time = 0.0
        self.overcurrent_since = None
        self.command_publishers = [
            self.create_publisher(Float64MultiArray, topic, 10)
            for topic in TOPICS]
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.create_subscription(ChassisState, '/hardware/chassis_state',
                                 self.on_hardware, 20)
        self.create_subscription(Float64MultiArray, '/wheel_controller/commands',
                                 self.on_wheel_target, 20)
        self.balance_client = self.create_client(SetBodyBalance, '/body_balance/set')

    def on_joints(self, message):
        values = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.joints = {
                name: math.degrees(values[name]) for name in JOINTS}
            self.wheel_speed = [velocities.get(name, float('nan')) for name in WHEELS]
            self.joint_time = time.monotonic()

    def on_hardware(self, message):
        self.hardware = message
        self.hardware_time = time.monotonic()

    def on_wheel_target(self, message):
        self.wheel_target = list(message.data)
        self.wheel_target_time = time.monotonic()

    def poll(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        now = time.monotonic()
        if (self.joints is None or self.hardware is None or
                now - self.joint_time > self.settings['feedback_timeout_s'] or
                now - self.hardware_time > self.settings['feedback_timeout_s']):
            raise RuntimeError('四腿或硬件状态反馈缺失/超时')
        state = self.hardware
        if (not state.dmc0_connected or not state.dmc1_connected or
                not state.se2_connected or not state.outputs_enabled or
                state.manual_recovery_required or state.fault):
            raise RuntimeError(f'硬件未就绪：{state.fault}')
        if (len(state.leg_current_ma) != 4 or
                any(not math.isfinite(value)
                    for value in state.leg_current_ma)):
            raise RuntimeError('腿电流反馈无效')
        if any(not math.isfinite(speed) or
               abs(speed) > self.settings['wheel_speed_max_rad_s']
               for speed in self.wheel_speed):
            raise RuntimeError('车轮未静止，停止腿部测试')
        if (self.wheel_target is not None and
                now - self.wheel_target_time < self.settings['feedback_timeout_s'] and
                any(not math.isfinite(value) or
                    abs(value) > self.settings['wheel_speed_max_rad_s']
                    for value in self.wheel_target)):
            raise RuntimeError('轮速目标不为零，停止腿部测试')

    def command(self, index, degree):
        self.command_publishers[index].publish(
            Float64MultiArray(data=[math.radians(degree)]))

    def hold(self, index):
        if self.joints:
            degree = self.joints[JOINTS[index]]
            for _ in range(3):
                self.command(index, degree)
                rclpy.spin_once(self, timeout_sec=0.05)


def move(probe, index, target, current_limit, overload_hold, report):
    """每步等待目标到位；电流、反馈或其他腿异常时立即保持当前角度。"""
    name = JOINTS[index]
    initial = dict(probe.joints)
    start = time.monotonic()
    last_progress = start
    previous = initial[name]
    peak = 0.0
    probe.command(index, target)
    while time.monotonic() - start < probe.settings['step_timeout_s']:
        probe.poll()
        currents = [abs(value) for value in probe.hardware.leg_current_ma]
        peak = max(peak, max(currents))
        if max(currents) >= probe.settings['instant_current_limit_ma']:
            channel = max(range(4), key=lambda item: currents[item])
            raise RuntimeError(
                f'{NAMES[channel]} 腿电流达到瞬时停止门槛：{currents[channel]:.0f} mA')
        if max(currents) >= current_limit:
            if probe.overcurrent_since is None:
                probe.overcurrent_since = time.monotonic()
            elif time.monotonic() - probe.overcurrent_since >= overload_hold:
                raise RuntimeError(
                    f'腿电流超过 {current_limit:.0f} mA 持续 {overload_hold:.1f} 秒')
        else:
            probe.overcurrent_since = None
        if any(abs(probe.joints[other] - initial[other]) >
               probe.settings['other_leg_tolerance_deg']
               for other in JOINTS if other != name):
            raise RuntimeError('非目标腿出现明显运动')
        actual = probe.joints[name]
        if abs(actual - target) <= probe.settings['joint_tolerance_deg']:
            report.append({'leg': NAMES[index], 'target_deg': target,
                           'actual_deg': actual, 'peak_current_ma': peak})
            return
        if abs(actual - previous) >= probe.settings['progress_deg']:
            previous = actual
            last_progress = time.monotonic()
        if time.monotonic() - last_progress > probe.settings['stall_timeout_s']:
            raise RuntimeError(f'{NAMES[index]} 角度反馈停滞：{actual:.2f}°')
    raise RuntimeError(f'{NAMES[index]} 目标 {target:.2f}° 未在单步时限内到位')


def ramp(probe, index, target, current_limit, overload_hold, report):
    """按配置步距靠近目标，终点不越过配置限位。"""
    while abs(probe.joints[JOINTS[index]] - target) > probe.settings['joint_tolerance_deg']:
        current = probe.joints[JOINTS[index]]
        step = math.copysign(
            min(probe.settings['step_deg'], abs(target - current)), target - current)
        move(probe, index, current + step, current_limit, overload_hold, report)


def main():
    parser = argparse.ArgumentParser(description='架空四腿软件限位逐腿测试')
    parser.add_argument('--confirm-suspended', action='store_true',
                        help='确认机器人已固定架空且现场有人可立即断电')
    parser.add_argument('--report', default='/tmp/leg_range_probe.json')
    parser.add_argument('--legs', nargs='+', choices=NAMES, default=NAMES)
    parser.add_argument('--endpoint', choices=('lower', 'upper', 'both'),
                        default='both')
    parser.add_argument('--test-lower-deg', type=float,
                        help='本轮谨慎测试的负向端点，不改变机器人软件限位')
    parser.add_argument('--test-upper-deg', type=float,
                        help='本轮谨慎测试的正向端点，不改变机器人软件限位')
    parser.add_argument('--home-only', action='store_true',
                        help='仅将指定腿分步回到 0°')
    parser.add_argument('--finish-zero', action='store_true',
                        help='每条腿测完后回到 0°，而非本轮起始角度')
    args = parser.parse_args()
    if not args.confirm_suspended:
        parser.error('实机测试前必须确认固定架空和现场断电手段')
    config = Path(get_package_share_directory('robot_control')) / 'config/body_balance.yaml'
    parameters = yaml.safe_load(config.read_text())['body_balance']['ros__parameters']
    probe_config = Path(get_package_share_directory('robot_control')) / (
        'config/leg_range_hardware_probe.yaml')
    settings = yaml.safe_load(probe_config.read_text())['leg_range_hardware_probe']
    lower = parameters['leg_min_deg']
    upper = parameters['leg_max_deg']
    current_limit = float(parameters['overload_current_ma'])
    overload_hold = float(parameters['overload_hold'])
    report = []
    rclpy.init()
    probe = LegProbe(settings)
    active_index = None
    failed = False
    try:
        if probe.balance_client.wait_for_service(timeout_sec=1.0):
            request = SetBodyBalance.Request()
            request.enabled = False
            request.level = 'mid'
            future = probe.balance_client.call_async(request)
            rclpy.spin_until_future_complete(probe, future, timeout_sec=3.0)
            if not future.done() or not future.result().success:
                raise RuntimeError('无法关闭自动平衡')
        deadline = time.monotonic() + 8.0
        while (probe.joints is None or probe.hardware is None) and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)
        probe.poll()
        for name in args.legs:
            index = NAMES.index(name)
            active_index = index
            baseline = probe.joints[JOINTS[index]]
            report.append({'leg': NAMES[index], 'baseline_deg': baseline,
                           'baseline_current_ma':
                           float(probe.hardware.leg_current_ma[index])})
            if not lower[index] <= baseline <= upper[index]:
                raise RuntimeError(f'{NAMES[index]} 起始角度超出软件限位')
            if max(abs(value) for value in probe.hardware.leg_current_ma) >= current_limit:
                raise RuntimeError('起始腿电流已达到停止门槛')
            if args.home_only:
                ramp(probe, index, 0.0, current_limit, overload_hold, report)
                continue
            test_lower = (lower[index] if args.test_lower_deg is None else
                          max(lower[index], args.test_lower_deg))
            test_upper = (upper[index] if args.test_upper_deg is None else
                          min(upper[index], args.test_upper_deg))
            if not test_lower < test_upper or not test_lower <= baseline <= test_upper:
                raise RuntimeError('本轮测试端点未覆盖起始角度')
            print(f'{NAMES[index]} 起始 {baseline:.2f}°，开始小角度确认', flush=True)
            small = min(baseline + settings['initial_probe_deg'], test_upper)
            if abs(small - baseline) < settings['joint_tolerance_deg']:
                small = max(baseline - settings['initial_probe_deg'], test_lower)
            ramp(probe, index, small, current_limit, overload_hold, report)
            ramp(probe, index, baseline, current_limit, overload_hold, report)
            endpoints = []
            if args.endpoint in ('lower', 'both'):
                endpoints.append(test_lower)
            if args.endpoint in ('upper', 'both'):
                endpoints.append(test_upper)
            for endpoint in endpoints:
                print(f'{NAMES[index]} 前往软件端点 {endpoint:.1f}°', flush=True)
                ramp(probe, index, endpoint, current_limit, overload_hold, report)
                ramp(probe, index, baseline, current_limit, overload_hold, report)
            if args.finish_zero:
                ramp(probe, index, 0.0, current_limit, overload_hold, report)
        print('本轮指定腿测试完成', flush=True)
    except Exception as error:
        failed = True
        print(f'测试停止：{error}', flush=True)
        report.append({'error': str(error), 'active_leg':
                       NAMES[active_index] if active_index is not None else None,
                       'angles_deg': probe.joints,
                       'leg_current_ma': (list(probe.hardware.leg_current_ma)
                                          if probe.hardware else None)})
    finally:
        if active_index is not None:
            probe.hold(active_index)
        Path(args.report).write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(f'记录：{args.report}', flush=True)
        probe.destroy_node()
        rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()

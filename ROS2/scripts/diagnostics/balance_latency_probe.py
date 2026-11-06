#!/usr/bin/env python3
"""作用：只读采集平衡与通信延迟基线；用法：source install/setup.bash 后 python3 本文件 --duration 60 --phase idle --output calibration_reports/balance/idle.jsonl。"""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from diagnostic_msgs.msg import DiagnosticArray
from robot_interfaces.msg import BodyBalanceStatus, ChassisState
from sensor_msgs.msg import Imu, JointState


class Probe(Node):
    """只订阅，不创建控制发布器或服务客户端，不改变当前机器人状态。"""

    def __init__(self, stream):
        super().__init__('balance_latency_probe')
        self.stream = stream
        self.started = time.monotonic()
        self.counts = {}
        self.timings = {}
        self.first_timings = {}
        self.first_timing_at = {}
        self.last_timing_at = {}
        self.enabled_seen = False
        self.max_wheel_speed = 0.0
        for kind, topic in (
                (Imu, '/sensors/imu/data'), (JointState, '/joint_states'),
                (BodyBalanceStatus, '/body_balance/status'),
                (ChassisState, '/hardware/chassis_state'),
                (DiagnosticArray, '/diagnostics')):
            self.create_subscription(kind, topic, lambda msg, name=topic: self.record(name, msg),
                                     qos_profile_sensor_data)

    def record(self, topic, msg):
        now = time.monotonic()-self.started
        self.counts[topic] = self.counts.get(topic, 0)+1
        if isinstance(msg, BodyBalanceStatus):
            self.enabled_seen |= msg.enabled
        if isinstance(msg, JointState):
            for name, velocity in zip(msg.name, msg.velocity):
                if 'wheel_joint' in name and math.isfinite(velocity):
                    self.max_wheel_speed = max(self.max_wheel_speed, abs(velocity))
        if isinstance(msg, DiagnosticArray):
            for status in msg.status:
                if status.name.startswith('robot_hardware/timing/'):
                    self.timings[status.hardware_id] = {v.key: v.value for v in status.values}
                    if status.hardware_id not in self.first_timings:
                        self.first_timings[status.hardware_id] = dict(self.timings[status.hardware_id])
                        self.first_timing_at[status.hardware_id] = now
                    self.last_timing_at[status.hardware_id] = now
        self.stream.write(json.dumps({'elapsed_s': now, 'topic': topic,
                                      'message': message_to_ordereddict(msg)},
                                     ensure_ascii=False, allow_nan=True)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--phase', choices=('idle', 'balance', 'obstacle'), default='idle')
    parser.add_argument('--output', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration 必须为有限正数')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    with args.output.open('x', encoding='utf-8') as stream:
        node = Probe(stream)
        try:
            while rclpy.ok() and time.monotonic()-node.started < args.duration:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            elapsed = time.monotonic()-node.started
            summary = {'phase': args.phase, 'duration_s': elapsed, 'counts': node.counts,
                       'arrival_hz': {k: v/elapsed for k, v in node.counts.items()},
                       'balance_enabled_seen': node.enabled_seen,
                       'phase_matches_observation': args.phase != 'idle' or not node.enabled_seen,
                       'max_wheel_speed_rad_s': node.max_wheel_speed,
                       'board_timing': node.timings,
                       'timing_available': bool(node.timings)}
            summary['board_window'] = {}
            for board, last in node.timings.items():
                first = node.first_timings[board]
                seconds = node.last_timing_at[board]-node.first_timing_at[board]
                delta = {key: int(last.get(key, '0'))-int(first.get(key, '0'))
                         for key in ('feedback_sequence', 'transaction_failures',
                                     'cycle_failures', 'watchdog_overruns', 'cycle_overruns')}
                summary['board_window'][board] = {
                    'observed_seconds': seconds, 'counter_deltas': delta,
                    'actual_feedback_hz': delta['feedback_sequence']/seconds if seconds > 0 else 0.0}
            args.output.with_suffix('.summary.json').write_text(
                json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
            print(json.dumps(summary, ensure_ascii=False))
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

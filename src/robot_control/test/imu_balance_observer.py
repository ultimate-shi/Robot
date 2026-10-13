#!/usr/bin/env python3
"""使用方法：在 Jazzy 容器运行本脚本，分段记录平衡节点读取的真实 IMU 姿态；本脚本不发运动命令。"""

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import BodyBalanceStatus


class Observer(Node):
    """采集 /body_balance/status 的横滚、俯仰与使能状态。"""

    def __init__(self):
        super().__init__('imu_balance_observer')
        self.samples = []
        self.start = time.monotonic()
        self.create_subscription(BodyBalanceStatus, '/body_balance/status',
                                 self.on_status, 20)

    def on_status(self, message):
        self.samples.append({
            'elapsed_s': round(time.monotonic() - self.start, 3),
            'roll_deg': round(message.actual_roll_deg, 3),
            'pitch_deg': round(message.actual_pitch_deg, 3),
            'enabled': message.enabled,
            'state': message.state,
        })


def main():
    parser = argparse.ArgumentParser(description='只读采集四腿平衡的 IMU 姿态')
    parser.add_argument('--duration-s', type=float, default=20.0)
    parser.add_argument('--report', default='/tmp/imu_balance_observer.json')
    args = parser.parse_args()
    rclpy.init()
    node = Observer()
    try:
        while time.monotonic() - node.start < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        Path(args.report).write_text(json.dumps(
            node.samples, ensure_ascii=False, indent=2))
        print(f'采集 {len(node.samples)} 帧，记录：{args.report}')
        for start in range(0, int(args.duration_s), 4):
            block = [item for item in node.samples
                     if start <= item['elapsed_s'] < start + 4]
            if block:
                print(f'{start:02d}–{start + 4:02d}s '
                      f'roll {min(item["roll_deg"] for item in block):+.1f}～'
                      f'{max(item["roll_deg"] for item in block):+.1f}° '
                      f'pitch {min(item["pitch_deg"] for item in block):+.1f}～'
                      f'{max(item["pitch_deg"] for item in block):+.1f}° '
                      f'enabled={any(item["enabled"] for item in block)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

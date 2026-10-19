#!/usr/bin/env python3
"""使用方法：ros2 run robot_perception stereo_pipeline_benchmark 统计双目频率和延迟。"""

from collections import OrderedDict
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String
from stereo_msgs.msg import DisparityImage


TOPICS = {
    'combined': (Image, '/stereo/image_raw'),
    'left_raw': (Image, '/stereo/left/image_raw'),
    'right_raw': (Image, '/stereo/right/image_raw'),
    'left_rect': (Image, '/stereo/left/image_rect'),
    'right_rect': (Image, '/stereo/right/image_rect'),
    'nav_left_rect': (Image, '/stereo/navigation/left/image_rect'),
    'nav_right_rect': (Image, '/stereo/navigation/right/image_rect'),
    'disparity': (DisparityImage, '/stereo/disparity'),
    'depth': (Image, '/stereo/depth/image'),
    'points': (PointCloud2, '/stereo/points2'),
    'nav_points': (PointCloud2, '/nav/stereo_obstacle_points'),
    'scan': (LaserScan, '/stereo/scan'),
}

RATE_TOPICS = (
    'left_rect',
    'right_rect',
    'nav_left_rect',
    'disparity',
    'depth',
    'nav_points',
    'scan',
)

STAGES = (
    ('combined', 'left_raw'),
    ('left_raw', 'left_rect'),
    ('left_rect', 'nav_left_rect'),
    ('nav_left_rect', 'disparity'),
    ('disparity', 'depth'),
    ('points', 'nav_points'),
    ('nav_points', 'scan'),
    ('combined', 'depth'),
    ('combined', 'scan'),
)


def percentile(values, ratio):
    """返回小样本也可复现的最近秩百分位数."""
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * ratio)))
    return round(float(ordered[index]), 2)


def message_stamp_ns(msg):
    """读取消息头；视差消息异常时退回其内嵌图像头."""
    header = getattr(msg, 'header', None)
    if header is None and hasattr(msg, 'image'):
        header = msg.image.header
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class StereoPipelineBenchmark(Node):
    """短时订阅整条链路并输出 JSON；仅在验收时运行以免增加常驻负载."""

    def __init__(self):
        super().__init__('stereo_pipeline_benchmark')
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('output_file', '')
        self.duration = max(30.0, float(self.get_parameter('duration').value))
        self.output_file = str(self.get_parameter('output_file').value)
        self.started = time.monotonic()
        self.done = False
        self.phases = (
            [('rate', name) for name in RATE_TOPICS]
            + [('stage', start, end) for start, end in STAGES]
        )
        self.phase_window = self.duration / len(self.phases)
        self.phase_index = -1
        self.phase_started = None
        self.phase_subscriptions = []
        self.phase_arrivals = {}
        self.phase_ages_ms = {}
        self.phase_matches = []
        self.matched_stamps = set()
        self.topic_results = {}
        self.stage_results = {}
        self.status_pub = self.create_publisher(
            String, '/stereo/pipeline/status', 10)

        # 每次只订阅一个话题或一对相邻阶段，避免 Python 同时反序列化多路大图。
        self.create_timer(0.1, self._advance_if_due)
        self._start_next_phase()
        self.get_logger().warn(
            f'低扰动性能验收开始，共 {len(self.phases)} 个串行阶段，'
            f'预计 {self.duration:.1f} 秒；测量期间请关闭 Foxglove')

    def _callback(self, name, msg):
        arrival = time.monotonic()
        stamp = message_stamp_ns(msg)
        self.phase_arrivals.setdefault(name, OrderedDict())[stamp] = arrival
        ros_now_ns = self.get_clock().now().nanoseconds
        if stamp > 0:
            self.phase_ages_ms.setdefault(name, []).append(
                max(0.0, (ros_now_ns - stamp) / 1e6))
        cache = self.phase_arrivals[name]
        while len(cache) > 300:
            cache.popitem(last=False)

        phase = self.phases[self.phase_index]
        if phase[0] != 'stage' or stamp in self.matched_stamps:
            return
        _, start, end = phase
        if stamp in self.phase_arrivals.get(start, {}) \
                and stamp in self.phase_arrivals.get(end, {}):
            delta = (
                self.phase_arrivals[end][stamp]
                - self.phase_arrivals[start][stamp]
            ) * 1000.0
            self.phase_matches.append(max(0.0, delta))
            self.matched_stamps.add(stamp)

    def _advance_if_due(self):
        if self.done or self.phase_started is None:
            return
        if time.monotonic() - self.phase_started < self.phase_window:
            return
        self._finish_phase()
        self._start_next_phase()

    def _start_next_phase(self):
        self.phase_index += 1
        if self.phase_index >= len(self.phases):
            self._finish_benchmark()
            return
        self.phase_arrivals = {}
        self.phase_ages_ms = {}
        self.phase_matches = []
        self.matched_stamps = set()
        phase = self.phases[self.phase_index]
        names = [phase[1]] if phase[0] == 'rate' else [phase[1], phase[2]]
        for name in names:
            message_type, topic = TOPICS[name]
            subscription = self.create_subscription(
                message_type,
                topic,
                lambda msg, key=name: self._callback(key, msg),
                qos_profile_sensor_data,
            )
            self.phase_subscriptions.append(subscription)
        self.phase_started = time.monotonic()

    def _finish_phase(self):
        phase = self.phases[self.phase_index]
        if phase[0] == 'rate':
            name = phase[1]
            arrivals = list(self.phase_arrivals.get(name, {}).values())
            intervals = [
                (current - previous) * 1000.0
                for previous, current in zip(arrivals, arrivals[1:])
            ]
            ages = self.phase_ages_ms.get(name, [])
            self.topic_results[name] = {
                'samples': len(arrivals),
                'rate_hz': round(1000.0 / (sum(intervals) / len(intervals)), 2)
                if intervals else 0.0,
                'interval_median_ms': percentile(intervals, 0.5),
                'interval_p95_ms': percentile(intervals, 0.95),
                'age_median_ms': percentile(ages, 0.5),
                'age_p95_ms': percentile(ages, 0.95),
            }
        else:
            _, start, end = phase
            self.stage_results[f'{start}->{end}'] = {
                'matched_frames': len(self.phase_matches),
                'median_ms': percentile(self.phase_matches, 0.5),
                'p95_ms': percentile(self.phase_matches, 0.95),
            }
        for subscription in self.phase_subscriptions:
            self.destroy_subscription(subscription)
        self.phase_subscriptions = []

    def _finish_benchmark(self):
        elapsed = time.monotonic() - self.started
        result = {
            'duration_sec': round(elapsed, 2),
            'measurement_mode': 'sequential_low_impact',
            'phase_window_sec': round(self.phase_window, 2),
            'topics': self.topic_results,
            'same_stamp_stages': self.stage_results,
            'note': '各话题和相邻阶段串行测量，避免同时订阅多路大消息造成明显反压。',
        }
        encoded = json.dumps(result, ensure_ascii=False, indent=2)
        self.status_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        print(encoded, flush=True)
        if self.output_file:
            path = os.path.abspath(os.path.expanduser(self.output_file))
            with open(path, 'w', encoding='utf-8') as stream:
                stream.write(encoded + '\n')
            self.get_logger().info(f'性能结果已写入 {path}')
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = StereoPipelineBenchmark()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

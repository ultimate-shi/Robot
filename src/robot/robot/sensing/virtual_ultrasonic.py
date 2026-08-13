#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot.launch 使用说明：
本节点由 robot.launch.py 启动，用统一点云输入模拟 8 路超声波。
它不再直接读取 PLY 文件，而是订阅 /perception/points：
- 仿真时 /perception/points 来自 publish_ply 读取的 studyroom.ply。
- 现实中 /perception/points 可以来自双目摄像头或深度相机。

输入：
- /perception/points：PointCloud2 点云。
- TF map -> radar-*：由机器人模型、关节状态和 odom 共同提供。

输出：
- /ultrasonic/front_fl、front_fr、front_rl、front_rr。
- /ultrasonic/side_fl、side_fr、side_rl、side_rr。
"""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, Range
import tf2_ros

from robot.mapping.snapshot_manager import cloud_to_xyz


SENSORS = [
    {'link': 'radar-front_fl', 'topic': '/ultrasonic/front_fl'},
    {'link': 'radar-front_fr', 'topic': '/ultrasonic/front_fr'},
    {'link': 'radar-front_rl', 'topic': '/ultrasonic/front_rl'},
    {'link': 'radar-front_rr', 'topic': '/ultrasonic/front_rr'},
    {'link': 'radar-side_fl', 'topic': '/ultrasonic/side_fl'},
    {'link': 'radar-side_fr', 'topic': '/ultrasonic/side_fr'},
    {'link': 'radar-side_rl', 'topic': '/ultrasonic/side_rl'},
    {'link': 'radar-side_rr', 'topic': '/ultrasonic/side_rr'},
]


class VirtualUltrasonic(Node):
    """Compute virtual ultrasonic ranges from /perception/points and TF."""

    def __init__(self):
        super().__init__('virtual_ultrasonic')

        self.declare_parameter('input_topic', '/perception/points')
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('fov_half_deg', 15.0)
        self.declare_parameter('min_height', 0.03)
        self.declare_parameter('max_height', 0.60)
        self.declare_parameter('voxel_size', 0.03)
        self.declare_parameter('publish_period', 0.2)
        self.declare_parameter('source_timeout', 2.0)
        self.declare_parameter('cache_static_source', False)

        input_topic = self.get_parameter('input_topic').value
        self.MAX_RANGE = self.get_parameter('max_range').value
        self.MIN_RANGE = self.get_parameter('min_range').value
        self.FOV_HALF = math.radians(self.get_parameter('fov_half_deg').value)
        self.MIN_HEIGHT = self.get_parameter('min_height').value
        self.MAX_HEIGHT = self.get_parameter('max_height').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.source_timeout = float(
            self.get_parameter('source_timeout').value)
        self.cache_static_source = bool(
            self.get_parameter('cache_static_source').value)

        self.points = np.empty((0, 3), dtype=np.float32)
        self.kdtree = None
        self.last_cloud_monotonic = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, input_topic, self.cloud_callback, 10)
        self.sensor_publishers = {
            sensor['link']: self.create_publisher(Range, sensor['topic'], 10)
            for sensor in SENSORS
        }
        self.create_timer(self.get_parameter('publish_period').value, self.publish_all)
        self.get_logger().info(f'VirtualUltrasonic listening to {input_topic}')

    def cloud_callback(self, msg: PointCloud2):
        """Update KDTree from the latest shared perception cloud."""
        # 地图预演使用静态点云，只需构建一次 KDTree；真实传感器保持更新。
        if self.cache_static_source and self.kdtree is not None:
            return
        try:
            points = cloud_to_xyz(msg)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f'点云解析失败: {exc}', throttle_duration_sec=2.0)
            return
        if len(points) == 0:
            return
        if self.voxel_size > 0.0:
            grid = np.floor(points / self.voxel_size).astype(np.int32)
            _, idx = np.unique(grid, axis=0, return_index=True)
            points = points[idx]
        self.points = points.astype(np.float32)
        self.kdtree = cKDTree(self.points)
        self.last_cloud_monotonic = time.monotonic()

    def get_sensor_distance(self, link_name):
        """Return nearest point distance in the sensor cone."""
        if self.kdtree is None or self.last_cloud_monotonic is None:
            return float('nan')
        if (self.source_timeout > 0.0
                and time.monotonic() - self.last_cloud_monotonic
                > self.source_timeout):
            return float('nan')

        try:
            tf = self.tf_buffer.lookup_transform('map', link_name, rclpy.time.Time())
        except Exception:
            return float('nan')

        sx = tf.transform.translation.x
        sy = tf.transform.translation.y
        sz = tf.transform.translation.z
        sensor_pos = np.array([sx, sy, sz])

        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        rot = self.quat_to_rotmat(qw, qx, qy, qz)
        front_2d = rot[:, 0][:2]
        norm = np.linalg.norm(front_2d)
        if norm < 1e-6:
            return self.MAX_RANGE
        front_2d /= norm

        ids = self.kdtree.query_ball_point(sensor_pos, self.MAX_RANGE)
        if not ids:
            return self.MAX_RANGE

        min_dist = self.MAX_RANGE
        for p in self.points[ids]:
            px, py, pz = p
            if pz < self.MIN_HEIGHT or pz > self.MAX_HEIGHT:
                continue
            dx = px - sx
            dy = py - sy
            horizontal_dist = math.sqrt(dx * dx + dy * dy)
            if horizontal_dist < self.MIN_RANGE or horizontal_dist > self.MAX_RANGE:
                continue
            vec = np.array([dx, dy])
            vnorm = np.linalg.norm(vec)
            if vnorm < 1e-6:
                continue
            vec /= vnorm
            angle = math.acos(float(np.clip(np.dot(front_2d, vec), -1.0, 1.0)))
            if angle <= self.FOV_HALF:
                min_dist = min(min_dist, horizontal_dist)

        return round(min_dist, 3)

    def quat_to_rotmat(self, w, x, y, z):
        """Convert quaternion to rotation matrix."""
        return np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
        ], dtype=np.float64)

    def publish_all(self):
        """Publish all eight Range messages."""
        stamp = self.get_clock().now().to_msg()
        for sensor in SENSORS:
            link = sensor['link']
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = link
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.FOV_HALF * 2.0
            msg.min_range = self.MIN_RANGE
            msg.max_range = self.MAX_RANGE
            msg.range = self.get_sensor_distance(link)
            self.sensor_publishers[link].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualUltrasonic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

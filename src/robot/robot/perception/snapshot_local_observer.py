#!/usr/bin/env python3
"""从已保存的全局点云生成随虚拟机器人移动的局部障碍观测。"""

import json
import math
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String

from robot.mapping.snapshot_manager import cloud_to_xyz


def quaternion_matrix(quaternion):
    """将 geometry_msgs Quaternion 转换为三维旋转矩阵。"""
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


class SnapshotLocalObserver(Node):
    """裁剪静态环境点云，模拟虚拟机器人当前位置的双目可见障碍。"""

    def __init__(self):
        super().__init__('snapshot_local_observer')
        defaults = {
            'input_topic': '/perception/points',
            'cloud_output_topic': '/nav/stereo_obstacle_points',
            'scan_output_topic': '/stereo/scan',
            'target_frame': 'base_link',
            'publish_rate': 4.0,
            'min_range': 0.25,
            'max_range': 4.0,
            'min_height': 0.05,
            'max_height': 0.80,
            'horizontal_fov_deg': 120.0,
            'scan_angle_increment_deg': 2.0,
            'source_voxel_size': 0.05,
            'voxel_size': 0.05,
            'max_points': 20000,
            'cache_static_source': True,
            'source_timeout': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)

        self.points = None
        self.source_frame = ''
        self.last_source_time = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cloud_pub = self.create_publisher(
            PointCloud2, str(self.cloud_output_topic), qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(
            LaserScan, str(self.scan_output_topic), qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            String, '/preview/local_observer/status', 10)
        self.create_subscription(
            PointCloud2, str(self.input_topic), self._cloud_callback,
            qos_profile_sensor_data)
        period = 1.0 / max(float(self.publish_rate), 0.1)
        self.create_timer(period, self._publish_observation)

    def _cloud_callback(self, msg):
        # 导航预演输入是静态地图，重复解析几十万点会阻塞安全扫描定时器。
        if bool(self.cache_static_source) and self.points is not None:
            return
        try:
            points = cloud_to_xyz(msg)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                f'预演点云解析失败: {exc}', throttle_duration_sec=2.0)
            return
        if len(points) == 0:
            return
        source_voxel = float(self.source_voxel_size)
        if source_voxel > 0.0:
            keys = np.floor(points / source_voxel).astype(np.int32)
            _, indices = np.unique(keys, axis=0, return_index=True)
            points = points[np.sort(indices)]
        self.points = np.ascontiguousarray(points, dtype=np.float32)
        self.source_frame = msg.header.frame_id or 'map'
        self.last_source_time = time.monotonic()

    def _publish_observation(self):
        if self.points is None or self.last_source_time is None:
            self._publish_status('waiting', 0, '尚未收到环境点云')
            return
        source_timeout = float(self.source_timeout)
        if (source_timeout > 0.0
                and time.monotonic() - self.last_source_time > source_timeout):
            self._publish_status('stale', 0, '环境点云已超时')
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.target_frame), self.source_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception as exc:
            self._publish_status('tf_error', 0, str(exc))
            return

        rotation = quaternion_matrix(transform.transform.rotation)
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float32)
        local = self.points @ rotation.T + translation
        local = self._filter(local)
        stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(self._make_cloud(local, stamp))
        self.scan_pub.publish(self._make_scan(local, stamp))
        self._publish_status('ok', len(local), '')

    def _filter(self, points):
        distances = np.hypot(points[:, 0], points[:, 1])
        angles = np.arctan2(points[:, 1], points[:, 0])
        half_fov = math.radians(float(self.horizontal_fov_deg)) / 2.0
        mask = (
            (points[:, 2] >= float(self.min_height))
            & (points[:, 2] <= float(self.max_height))
            & (distances >= float(self.min_range))
            & (distances <= float(self.max_range))
            & (np.abs(angles) <= half_fov)
        )
        result = points[mask]
        voxel = float(self.voxel_size)
        if voxel > 0.0 and len(result):
            keys = np.floor(result / voxel).astype(np.int32)
            _, indices = np.unique(keys, axis=0, return_index=True)
            result = result[np.sort(indices)]
        maximum = int(self.max_points)
        if maximum > 0 and len(result) > maximum:
            step = int(math.ceil(len(result) / maximum))
            result = result[::step]
        return np.ascontiguousarray(result, dtype=np.float32)

    def _make_cloud(self, points, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.target_frame)
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32,
                       count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.width * msg.point_step
        msg.data = points.tobytes()
        msg.is_dense = True
        return msg

    def _make_scan(self, points, stamp):
        half_fov = math.radians(float(self.horizontal_fov_deg)) / 2.0
        increment = math.radians(float(self.scan_angle_increment_deg))
        count = int(round(2.0 * half_fov / increment)) + 1
        ranges = np.full(count, np.inf, dtype=np.float32)
        if len(points):
            angles = np.arctan2(points[:, 1], points[:, 0])
            distances = np.hypot(points[:, 0], points[:, 1])
            indices = np.rint((angles + half_fov) / increment).astype(int)
            valid = (indices >= 0) & (indices < count)
            for index, distance in zip(indices[valid], distances[valid]):
                ranges[index] = min(ranges[index], float(distance))
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.target_frame)
        msg.angle_min = -half_fov
        msg.angle_max = half_fov
        msg.angle_increment = increment
        msg.scan_time = 1.0 / max(float(self.publish_rate), 0.1)
        msg.range_min = float(self.min_range)
        msg.range_max = float(self.max_range)
        msg.ranges = ranges.tolist()
        return msg

    def _publish_status(self, state, count, message):
        payload = {'state': state, 'point_count': int(count),
                   'message': message}
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotLocalObserver()
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

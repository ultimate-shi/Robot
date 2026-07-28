#!/usr/bin/env python3
"""独立过滤真实双目点云，输出给 Nav2 使用的 base_link 障碍点。"""

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
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class StereoPointCloudFilter(Node):
    """在采集时间对应的 TF 下完成变换、裁剪、体素过滤和限点。"""

    def __init__(self):
        super().__init__('stereo_pointcloud_filter')
        defaults = {
            'input_topic': '/stereo/points2',
            'output_topic': '/nav/stereo_obstacle_points',
            'status_topic': '/stereo/pointcloud_filter/status',
            'target_frame': 'base_link',
            'min_height': 0.05,
            'max_height': 0.80,
            'min_range': 0.25,
            'max_range': 4.0,
            'horizontal_fov_deg': 120.0,
            'voxel_size': 0.05,
            'max_points': 20000,
            'max_input_rate': 20.0,
            'tf_timeout': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.output_pub = self.create_publisher(
            PointCloud2, str(self.output_topic), qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            String, str(self.status_topic), 10)
        self.create_subscription(
            PointCloud2, str(self.input_topic), self.cloud_callback,
            qos_profile_sensor_data)

        self.last_accept_monotonic = 0.0
        self.last_message_monotonic = None
        self.filtered_frames = 0
        self.dropped_frames = 0
        self.tf_errors = 0
        self.processing = False

    def cloud_callback(self, msg):
        now_monotonic = time.monotonic()
        minimum_period = (
            1.0 / float(self.max_input_rate)
            if float(self.max_input_rate) > 0.0 else 0.0
        )
        if (self.processing
                or now_monotonic - self.last_accept_monotonic < minimum_period):
            self.dropped_frames += 1
            return
        self.processing = True
        self.last_accept_monotonic = now_monotonic
        try:
            points = self._cloud_to_xyz(msg)
            input_count = len(points)
            points = self._transform_to_base(points, msg)
            if points is None:
                self.tf_errors += 1
                self.dropped_frames += 1
                self._publish_status(msg, input_count, 0, 0.0, 'tf_error')
                return

            points = self._filter_points(points)
            points = self._voxel_downsample(points)
            if len(points) > int(self.max_points):
                step = int(math.ceil(len(points) / int(self.max_points)))
                points = points[::step]
            points = np.ascontiguousarray(points, dtype=np.float32)
            self._publish_cloud(points, msg)

            processing_ms = (time.monotonic() - now_monotonic) * 1000.0
            self.filtered_frames += 1
            self._publish_status(
                msg, input_count, len(points), processing_ms, 'ok')
        except (TypeError, ValueError) as exc:
            self.dropped_frames += 1
            self.get_logger().error(
                f'双目点云解析失败: {exc}', throttle_duration_sec=2.0)
            self._publish_status(msg, 0, 0, 0.0, 'invalid_cloud')
        finally:
            self.processing = False

    @staticmethod
    def _cloud_to_xyz(msg):
        """使用带 offset/stride 的 NumPy 结构数组解析，避免逐点 unpack。"""
        fields = {field.name: field for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(fields):
            raise ValueError('PointCloud2 缺少 x/y/z 字段')
        type_map = {
            PointField.INT8: 'i1',
            PointField.UINT8: 'u1',
            PointField.INT16: 'i2',
            PointField.UINT16: 'u2',
            PointField.INT32: 'i4',
            PointField.UINT32: 'u4',
            PointField.FLOAT32: 'f4',
            PointField.FLOAT64: 'f8',
        }
        endian = '>' if msg.is_bigendian else '<'
        names, formats, offsets = [], [], []
        for name in ('x', 'y', 'z'):
            field = fields[name]
            if field.datatype not in type_map or field.count != 1:
                raise ValueError(f'字段 {name} 的 datatype/count 不受支持')
            names.append(name)
            formats.append(np.dtype(endian + type_map[field.datatype]))
            offsets.append(field.offset)
        dtype = np.dtype({
            'names': names,
            'formats': formats,
            'offsets': offsets,
            'itemsize': msg.point_step,
        })
        structured = np.ndarray(
            shape=(msg.height, msg.width),
            dtype=dtype,
            buffer=msg.data,
            strides=(msg.row_step, msg.point_step),
        )
        points = np.column_stack((
            structured['x'].reshape(-1),
            structured['y'].reshape(-1),
            structured['z'].reshape(-1),
        )).astype(np.float32, copy=False)
        return points[np.isfinite(points).all(axis=1)]

    def _transform_to_base(self, points, msg):
        if msg.header.frame_id == self.target_frame:
            return points
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.target_frame),
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=float(self.tf_timeout)),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'TF {self.target_frame} <- {msg.header.frame_id} 查询失败: '
                f'{exc}', throttle_duration_sec=2.0)
            return None

        q = transform.transform.rotation
        rotation = self._quaternion_matrix(q.x, q.y, q.z, q.w)
        translation = transform.transform.translation
        offset = np.array(
            [translation.x, translation.y, translation.z], dtype=np.float32)
        return points @ rotation.T + offset

    @staticmethod
    def _quaternion_matrix(x, y, z, w):
        norm = x * x + y * y + z * z + w * w
        if norm < 1e-12:
            return np.eye(3, dtype=np.float32)
        scale = 2.0 / norm
        return np.array([
            [1.0 - scale * (y * y + z * z),
             scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w),
             1.0 - scale * (x * x + z * z),
             scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w),
             1.0 - scale * (x * x + y * y)],
        ], dtype=np.float32)

    def _filter_points(self, points):
        if points.size == 0:
            return points
        planar_range = np.hypot(points[:, 0], points[:, 1])
        half_fov = math.radians(float(self.horizontal_fov_deg)) * 0.5
        angles = np.abs(np.arctan2(points[:, 1], points[:, 0]))
        mask = (
            (points[:, 2] >= float(self.min_height))
            & (points[:, 2] <= float(self.max_height))
            & (planar_range >= float(self.min_range))
            & (planar_range <= float(self.max_range))
            & (points[:, 0] > 0.0)
            & (angles <= half_fov)
        )
        return points[mask]

    def _voxel_downsample(self, points):
        if points.size == 0 or float(self.voxel_size) <= 0.0:
            return points
        grid = np.floor(points / float(self.voxel_size)).astype(np.int32)
        _, indices = np.unique(grid, axis=0, return_index=True)
        return points[np.sort(indices)]

    def _publish_cloud(self, points, source):
        cloud = PointCloud2()
        cloud.header.stamp = source.header.stamp
        cloud.header.frame_id = str(self.target_frame)
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(
                name=name, offset=index * 4,
                datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(('x', 'y', 'z'))
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.output_pub.publish(cloud)

    def _publish_status(
            self, source, input_count, output_count, processing_ms, state):
        now = self.get_clock().now()
        source_time = rclpy.time.Time.from_msg(source.header.stamp)
        latency_ms = max(0.0, (now - source_time).nanoseconds / 1e6)
        monotonic_now = time.monotonic()
        fps = 0.0
        if self.last_message_monotonic is not None:
            period = monotonic_now - self.last_message_monotonic
            if period > 0.0:
                fps = 1.0 / period
        self.last_message_monotonic = monotonic_now
        status = {
            'state': state,
            'fps': round(fps, 2),
            'latency_ms': round(latency_ms, 2),
            'processing_ms': round(processing_ms, 2),
            'input_points': input_count,
            'output_points': output_count,
            'tf_errors': self.tf_errors,
            'dropped_frames': self.dropped_frames,
            'filtered_frames': self.filtered_frames,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = StereoPointCloudFilter()
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

"""作用：把可选相机点云投影为定位专用扫描；使用方法：localization_scan.launch.py，Astra 需外部驱动和准确安装 TF。"""

import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from robot_perception.stereo.stereo_pointcloud_filter import StereoPointCloudFilter


def project_scan(points, origin, angle_min, angle_max, increment, range_min, range_max):
    """按真实光心投影最近端点，未知角度使用 NaN 而非无障碍/最大量程。"""
    ranges = np.full(int(math.floor((angle_max-angle_min)/increment))+1, np.inf)
    relative = np.asarray(points)[:, :2]-np.asarray(origin)[:2]
    distances = np.linalg.norm(relative, axis=1)
    angles = np.arctan2(relative[:, 1], relative[:, 0])
    valid = (np.isfinite(distances) & (distances >= range_min) & (distances <= range_max) &
             (angles >= angle_min) & (angles <= angle_max))
    indices = np.floor((angles[valid]-angle_min)/increment).astype(int)
    np.minimum.at(ranges, indices, distances[valid])
    ranges[~np.isfinite(ranges)] = np.nan
    return ranges


class LocalizationScan(Node):
    def __init__(self):
        super().__init__('localization_scan')
        defaults = {
            'input_topic': '/stereo/points2', 'sensor_frame': 'stereo_left_camera_frame',
            'output_topic': '/localization/scan', 'status_topic': '/localization/scan_status',
            'target_frame': 'base_link', 'odom_frame': 'odom', 'scan_frame': 'localization_scan',
            'min_height': .05, 'max_height': .8, 'min_range': .25, 'max_range': 4.,
            'horizontal_fov_deg': 120., 'angle_increment': .00872664626,
            'voxel_size': .05, 'min_points_per_voxel': 3, 'min_neighbor_voxels': 1,
            'max_input_rate': 4., 'max_message_age': 1.5, 'tf_timeout': .02,
            'max_tilt': .174533, 'mount_tolerance': .005,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        numeric = [getattr(self, name) for name, default in defaults.items()
                   if isinstance(default, (int, float))]
        if (not all(math.isfinite(value) and value > 0 for value in numeric) or
                self.min_height >= self.max_height or self.min_range >= self.max_range or
                self.horizontal_fov_deg > 360 or self.angle_increment > math.pi):
            raise ValueError('定位扫描范围、频率、容差及过滤参数无效')
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.origin = None
        self.last_processed = -math.inf
        self.last_stamp = -1
        self.publisher = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)
        self.status = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(PointCloud2, self.input_topic, self.on_cloud, qos_profile_sensor_data)

    def report(self, state, stamp, points=0):
        self.status.publish(String(data=json.dumps({
            'state': state, 'input_topic': self.input_topic, 'sensor_frame': self.sensor_frame,
            'stamp': stamp, 'points': points}, ensure_ascii=False)))

    def lookup(self, target, source, stamp):
        return self.buffer.lookup_transform(target, source, rclpy.time.Time.from_msg(stamp),
                                            timeout=Duration(seconds=self.tf_timeout)).transform

    def on_cloud(self, msg):
        stamp_ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp_ns)/1e9
        if stamp_ns <= self.last_stamp or stamp_ns <= 0 or not 0 <= age <= self.max_message_age:
            self.report('invalid_timestamp', stamp_ns/1e9)
            return
        if time.monotonic()-self.last_processed < 1/self.max_input_rate:
            return
        self.last_processed = time.monotonic()
        try:
            body = self.lookup(self.odom_frame, self.target_frame, msg.header.stamp)
            q = body.rotation
            if math.acos(np.clip(1-2*(q.x*q.x+q.y*q.y), -1., 1.)) > self.max_tilt:
                self.report('tilted', stamp_ns/1e9)
                return
            camera = self.lookup(self.target_frame, self.sensor_frame, msg.header.stamp)
            position = camera.translation
            origin = np.array([position.x, position.y, position.z])
            if self.origin is not None and np.linalg.norm(origin-self.origin) > self.mount_tolerance:
                self.report('mount_moved', stamp_ns/1e9)
                return
            transform = self.lookup(self.target_frame, msg.header.frame_id, msg.header.stamp)
            q, t = transform.rotation, transform.translation
            rotation = StereoPointCloudFilter._quaternion_matrix(q.x, q.y, q.z, q.w)
            points = StereoPointCloudFilter._cloud_to_xyz(msg) @ rotation.T + [t.x, t.y, t.z]
            points = StereoPointCloudFilter._filter_points(self, points)
            points = StereoPointCloudFilter._voxel_downsample(self, points)
            half = math.radians(self.horizontal_fov_deg)/2
            ranges = project_scan(points, origin, -half, half, self.angle_increment,
                                  self.min_range, self.max_range)
            if self.origin is None:
                self.origin = origin
                frame = TransformStamped()
                frame.header.frame_id, frame.child_frame_id = self.target_frame, self.scan_frame
                frame.header.stamp = msg.header.stamp
                frame.transform.translation.x, frame.transform.translation.y, frame.transform.translation.z = map(float, origin)
                frame.transform.rotation.w = 1.
                self.broadcaster.sendTransform(frame)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = msg.header.stamp, self.scan_frame
            scan.angle_min, scan.angle_increment = -half, self.angle_increment
            scan.angle_max = -half+(len(ranges)-1)*self.angle_increment
            scan.range_min, scan.range_max = self.min_range, self.max_range
            scan.scan_time, scan.time_increment = 1/self.max_input_rate, 0.
            scan.ranges = ranges.astype(np.float32).tolist()
            self.publisher.publish(scan)
            self.last_stamp = stamp_ns
            self.report('ok', stamp_ns/1e9, int(np.isfinite(ranges).sum()))
        except Exception as exc:
            self.report('invalid_cloud_or_tf', stamp_ns/1e9)
            self.get_logger().warning(f'定位扫描拒绝本帧：{exc}', throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

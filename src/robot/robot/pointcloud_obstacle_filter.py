#!/usr/bin/env python3
"""
robot.launch 使用说明：
本节点由 robot.launch.py 以 executable='pointcloud_obstacle_filter' 启动。
它是仿真 PLY 点云和现实双目相机点云复用 Nav2 的关键转换层。

输入：
- /perception/points：统一点云输入，仿真来自 publish_ply，现实可来自双目摄像头。
- TF：读取 robot_base_frame 在点云坐标系中的位置，只保留机器人附近 local_radius 范围内的点。

处理：
- 解析 PointCloud2 的 x/y/z 字段。
- 按高度 min_obstacle_height/max_obstacle_height 去掉地面或过高点。
- 按 local_radius 裁剪局部范围，降低 Nav2 costmap 压力。
- 按 voxel_size 体素降采样，限制 max_points 数量。

输出：
- /nav/obstacle_points：过滤后的障碍物点云，供 Nav2 VoxelLayer 标记/清除局部障碍。
- /pointcloud_obstacle_status：输出过滤后的点数，用于调试点云是否进入 Nav2。

为什么不能删除：
nav2_params.yaml 的 local_costmap 直接订阅 /nav/obstacle_points；删除会导致 Nav2 点云避障失效。
"""

import math
import struct

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
import tf2_ros


FLOAT32 = PointField.FLOAT32


class PointCloudObstacleFilter(Node):
    """Convert a reusable perception point cloud into Nav2 obstacle points."""

    def __init__(self):
        super().__init__('pointcloud_obstacle_filter')

        self.declare_parameter('input_topic', '/perception/points')
        self.declare_parameter('output_topic', '/nav/obstacle_points')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('min_obstacle_height', 0.05)
        self.declare_parameter('max_obstacle_height', 0.80)
        self.declare_parameter('local_radius', 4.0)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('max_points', 20000)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').value
        self.local_radius = self.get_parameter('local_radius').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, '/pointcloud_obstacle_status', 10)

        self.get_logger().info(
            f'PointCloudObstacleFilter: {self.input_topic} -> {self.output_topic}, '
            f'height=[{self.min_obstacle_height}, {self.max_obstacle_height}]m'
        )

    def cloud_callback(self, msg: PointCloud2):
        points = self._cloud_to_xyz_array(msg)
        if points.size == 0:
            self._publish_points(np.empty((0, 3), dtype=np.float32), msg.header)
            return

        points = self._filter_local_area(points, msg.header.frame_id)
        points = self._filter_obstacle_height(points)
        points = self._voxel_downsample(points)

        if len(points) > self.max_points:
            step = max(1, int(math.ceil(len(points) / self.max_points)))
            points = points[::step]

        self._publish_points(points, msg.header)
        self._publish_status(points)

    def _cloud_to_xyz_array(self, msg: PointCloud2) -> np.ndarray:
        offsets = {field.name: field.offset for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(offsets):
            self.get_logger().warn('Ignoring point cloud without x/y/z fields')
            return np.empty((0, 3), dtype=np.float32)

        point_count = msg.width * msg.height
        if point_count == 0:
            return np.empty((0, 3), dtype=np.float32)

        data = memoryview(msg.data)
        points = np.empty((point_count, 3), dtype=np.float32)
        endian = '>' if msg.is_bigendian else '<'
        fmt = endian + 'f'

        for i in range(point_count):
            base = i * msg.point_step
            points[i, 0] = struct.unpack_from(fmt, data, base + offsets['x'])[0]
            points[i, 1] = struct.unpack_from(fmt, data, base + offsets['y'])[0]
            points[i, 2] = struct.unpack_from(fmt, data, base + offsets['z'])[0]

        return points[np.isfinite(points).all(axis=1)]

    def _filter_local_area(self, points: np.ndarray, cloud_frame: str) -> np.ndarray:
        if self.local_radius <= 0.0 or points.size == 0:
            return points

        try:
            tf = self.tf_buffer.lookup_transform(
                cloud_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except Exception:
            return points

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        dist_sq = (points[:, 0] - robot_x) ** 2 + (points[:, 1] - robot_y) ** 2
        return points[dist_sq <= self.local_radius ** 2]

    def _filter_obstacle_height(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        mask = ((points[:, 2] >= self.min_obstacle_height) &
                (points[:, 2] <= self.max_obstacle_height))
        return points[mask]

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0 or self.voxel_size <= 0.0:
            return points.astype(np.float32)
        grid = np.floor(points / self.voxel_size).astype(np.int32)
        _, idx = np.unique(grid, axis=0, return_index=True)
        return points[idx].astype(np.float32)

    def _publish_points(self, points: np.ndarray, source_header: Header):
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = source_header.frame_id or self.target_frame
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.astype(np.float32).tobytes()
        self.obstacle_pub.publish(cloud)

    def _publish_status(self, obstacle_points: np.ndarray):
        msg = String()
        msg.data = '{"pointcloud_obstacle_count": %d}' % len(obstacle_points)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudObstacleFilter()
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

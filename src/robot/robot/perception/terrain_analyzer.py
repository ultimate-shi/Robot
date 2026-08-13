#!/usr/bin/env python3
"""
robot.launch 使用说明：
本节点由 robot.launch.py 启动，把统一点云输入转换为统一地形状态输出。
它让仿真 PLY 点云和现实双目点云都走同一个接口：
/perception/points -> /terrain_status。

输入：
- /perception/points：PointCloud2，仿真来自 publish_ply，现实可来自双目摄像头。
- /odom：机器人当前 x/y/yaw 和速度，用于在当前位置评估地形。

输出：
- /terrain_status：JSON 字符串，包含坡度、台阶、坑洼、打滑、body_z、roll、pitch 等信息。
"""

import json
import math
import struct

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from robot.perception.terrain_heightmap import TerrainHeightmap
from robot.perception.terrain_physics import TerrainPhysics


class TerrainAnalyzerNode(Node):
    """Analyze terrain from /perception/points and publish /terrain_status."""

    def __init__(self):
        super().__init__('terrain_analyzer')

        self.declare_parameter('input_topic', '/perception/points')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('grid_resolution', 0.02)
        self.declare_parameter('ground_tolerance', 0.05)
        self.declare_parameter('terrain_voxel_size', 0.03)
        self.declare_parameter('max_grade_deg', 35.0)
        self.declare_parameter('step_threshold', 0.03)
        self.declare_parameter('dropoff_threshold', 0.05)
        self.declare_parameter('look_ahead_distance', 0.10)
        self.declare_parameter('look_ahead_samples', 5)
        self.declare_parameter('ground_to_base_height', 0.15)
        self.declare_parameter('wheelbase', 0.4)
        self.declare_parameter('track', 0.2)

        input_topic = self.get_parameter('input_topic').value
        publish_rate = self.get_parameter('publish_rate').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.ground_tolerance = self.get_parameter('ground_tolerance').value
        self.terrain_voxel_size = self.get_parameter('terrain_voxel_size').value

        self.physics = TerrainPhysics(
            max_grade_deg=self.get_parameter('max_grade_deg').value,
            step_threshold=self.get_parameter('step_threshold').value,
            dropoff_threshold=self.get_parameter('dropoff_threshold').value,
            look_ahead_distance=self.get_parameter('look_ahead_distance').value,
            look_ahead_samples=self.get_parameter('look_ahead_samples').value,
            ground_to_base_height=self.get_parameter('ground_to_base_height').value,
            wheelbase=self.get_parameter('wheelbase').value,
            track=self.get_parameter('track').value,
        )

        self.heightmap = None
        self.latest_odom = None
        self.create_subscription(PointCloud2, input_topic, self.cloud_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_pub = self.create_publisher(String, '/terrain_status', 10)
        self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(f'TerrainAnalyzer listening to {input_topic}')

    def cloud_callback(self, msg: PointCloud2):
        """Rebuild terrain heightmap from the latest perception cloud."""
        points = self._cloud_to_xyz_array(msg)
        if len(points) < 10:
            self.get_logger().warn('Ignoring terrain cloud with too few valid points')
            return
        try:
            self.heightmap = TerrainHeightmap(
                points=points,
                resolution=self.grid_resolution,
                ground_tolerance=self.ground_tolerance,
                voxel_size=self.terrain_voxel_size,
            )
            self.get_logger().info(
                f'Terrain heightmap updated: {self.heightmap.grid_w}x{self.heightmap.grid_h}, '
                f'ground_z={self.heightmap.ground_z_base:.3f}'
            )
        except Exception as exc:
            self.get_logger().warn(f'Failed to update terrain heightmap: {exc}')

    def odom_callback(self, msg: Odometry):
        """Cache latest odometry for terrain evaluation."""
        self.latest_odom = msg

    def timer_callback(self):
        """Evaluate terrain at current odometry pose and publish JSON status."""
        if self.heightmap is None or self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose
        twist = self.latest_odom.twist.twist
        yaw = self._yaw_from_quaternion(pose.orientation)
        constraint = self.physics.evaluate(
            self.heightmap,
            pose.position.x,
            pose.position.y,
            yaw,
            twist.linear.x,
        )

        status = {
            'is_blocked': constraint.is_blocked,
            'block_reason': constraint.block_reason,
            'slip_factor': round(constraint.slip_factor, 3),
            'traversability': round(constraint.traversability, 3),
            'body_z': round(constraint.body_z, 4),
            'roll': round(constraint.roll, 6),
            'pitch': round(constraint.pitch, 6),
            'roll_deg': round(math.degrees(constraint.roll), 2),
            'pitch_deg': round(math.degrees(constraint.pitch), 2),
            'step_blocked': constraint.block_reason == 'step',
            'dropoff_blocked': constraint.block_reason == 'dropoff',
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _cloud_to_xyz_array(self, msg: PointCloud2) -> np.ndarray:
        offsets = {field.name: field.offset for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(offsets):
            return np.empty((0, 3), dtype=np.float32)

        point_count = msg.width * msg.height
        points = np.empty((point_count, 3), dtype=np.float32)
        data = memoryview(msg.data)
        endian = '>' if msg.is_bigendian else '<'
        fmt = endian + 'f'
        for i in range(point_count):
            base = i * msg.point_step
            points[i, 0] = struct.unpack_from(fmt, data, base + offsets['x'])[0]
            points[i, 1] = struct.unpack_from(fmt, data, base + offsets['y'])[0]
            points[i, 2] = struct.unpack_from(fmt, data, base + offsets['z'])[0]
        return points[np.isfinite(points).all(axis=1)]

    def _yaw_from_quaternion(self, q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalyzerNode()
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

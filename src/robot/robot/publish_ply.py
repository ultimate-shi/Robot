#!/usr/bin/env python3
"""
robot.launch 使用说明：
本节点由 robot.launch.py 以 executable='publish_ply' 启动。
作用是把离线 iPhone 扫描得到的 PLY 点云文件转换成 ROS 2 的 PointCloud2 话题。

输入：
- 参数 ply_file，默认指向 robot 包 share 目录下的 map/studyroom.ply。
- 参数 frame_id，默认 map，表示点云坐标已经在地图坐标系下。

输出：
- /pointcloud：给 Foxglove/RViz 显示完整房间点云。
- /perception/points：仿真和现实复用的统一点云输入接口；现实双目相机也应输出到这个话题。

为什么不能删除：
Nav2 点云避障链路依赖 /perception/points，前端显示依赖 /pointcloud。
如果换成真实双目相机，可以保留接口但用相机驱动替代本节点的数据来源。
"""

import os

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

try:
    from plyfile import PlyData
except ImportError:
    PlyData = None


class PLYPublisher(Node):
    """Publish an offline PLY map as reusable PointCloud2 topics."""

    def __init__(self):
        super().__init__('ply_publisher')

        pkg_share = get_package_share_directory('robot')
        default_ply = os.path.join(pkg_share, 'map', 'studyroom.ply')

        self.declare_parameter('ply_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('display_topic', '/pointcloud')
        self.declare_parameter('perception_topic', '/perception/points')

        requested_ply = str(self.get_parameter('ply_file').value).strip()
        self.ply_file = requested_ply if requested_ply else default_ply
        self.frame_id = self.get_parameter('frame_id').value
        period = self.get_parameter('publish_period').value
        display_topic = self.get_parameter('display_topic').value
        perception_topic = self.get_parameter('perception_topic').value

        self.display_pub = self.create_publisher(PointCloud2, display_topic, 10)
        self.perception_pub = self.create_publisher(PointCloud2, perception_topic, 10)
        self.points = self._load_points()

        self.timer = self.create_timer(period, self.publish_cloud)
        self.get_logger().info(
            f'PLYPublisher loaded {len(self.points)} points from {self.ply_file}; '
            f'publishing {display_topic} and {perception_topic}'
        )

    def _load_points(self):
        if PlyData is None:
            self.get_logger().error('plyfile is not installed; publishing fallback test cloud')
            return np.array([[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]], dtype=np.float32)

        try:
            plydata = PlyData.read(self.ply_file)
            vertex = plydata['vertex']
            return np.column_stack((vertex['x'], vertex['y'], vertex['z'])).astype(np.float32)
        except Exception as exc:
            self.get_logger().error(f'Failed to load PLY: {exc}')
            return np.array([[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]], dtype=np.float32)

    def publish_cloud(self):
        cloud = self._make_cloud()
        self.display_pub.publish(cloud)
        self.perception_pub.publish(cloud)

    def _make_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(self.points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.data = self.points.tobytes()
        cloud.is_dense = True
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = PLYPublisher()
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

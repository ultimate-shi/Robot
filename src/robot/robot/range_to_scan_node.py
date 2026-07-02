#!/usr/bin/env python3
"""
foxglove3d 使用说明：
本节点由 foxglove3d.launch.py 以 executable='range_to_scan' 启动。
它把 8 路超声波 Range 数据合成为一个稀疏 LaserScan，主要用于 Foxglove/RViz 调试和兼容。

输入：
- /ultrasonic/front_fl、front_fr、front_rl、front_rr。
- /ultrasonic/side_fl、side_fr、side_rl、side_rr。

输出：
- /scan：sensor_msgs/LaserScan，frame_id 为 base_link，角度范围 -pi 到 pi，5 度一个采样。

注意：
/scan 只是 8 路超声波展开后的稀疏扫描，不等价于真实 360 度激光雷达。
当前 Nav2 local_costmap 主要使用 /nav/obstacle_points，/scan 保留用于调试。
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan


class RangeToScan(Node):

    def __init__(self):

        super().__init__('range_to_scan')

        self.angle_min = -math.pi
        self.angle_max = math.pi

        self.angle_increment = math.radians(5)

        self.num_ranges = int(
            (self.angle_max - self.angle_min)
            / self.angle_increment
        )

        self.range_min = 0.02
        self.range_max = 4.0

        self.latest = {}

        self.sensor_angles = {

            "front_fl": 0.0,
            "front_fr": 0.0,

            "side_fl": math.pi / 2,
            "side_rl": math.pi / 2,

            "side_fr": -math.pi / 2,
            "side_rr": -math.pi / 2,

            "front_rl": math.pi,
            "front_rr": math.pi,
        }

        sensors = [
            "front_fl",
            "front_fr",
            "front_rl",
            "front_rr",
            "side_fl",
            "side_fr",
            "side_rl",
            "side_rr",
        ]

        for sensor in sensors:

            self.create_subscription(
                Range,
                f"/ultrasonic/{sensor}",
                lambda msg, s=sensor:
                    self.range_callback(msg, s),
                10
            )

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan",
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_scan
        )

        self.get_logger().info(
            "range_to_scan started"
        )

    def range_callback(self, msg, sensor):

        self.latest[sensor] = (
            msg.range,
            msg.field_of_view
        )

    def publish_scan(self):

        ranges = [float('inf')] * self.num_ranges

        for sensor, data in self.latest.items():

            distance = data[0]
            fov = data[1]

            center = self.sensor_angles[sensor]

            start_angle = center - fov / 2.0
            end_angle = center + fov / 2.0

            angle = start_angle

            while angle <= end_angle:

                index = int(
                    (angle - self.angle_min)
                    / self.angle_increment
                )

                if 0 <= index < self.num_ranges:

                    if ranges[index] == float('inf'):
                        ranges[index] = distance
                    else:
                        ranges[index] = min(
                            ranges[index],
                            distance
                        )

                angle += self.angle_increment

        scan = LaserScan()

        scan.header.stamp = \
            self.get_clock().now().to_msg()

        scan.header.frame_id = "base_link"

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment

        scan.time_increment = 0.0
        scan.scan_time = 0.1

        scan.range_min = self.range_min
        scan.range_max = self.range_max

        scan.ranges = ranges

        self.scan_pub.publish(scan)


def main(args=None):

    rclpy.init(args=args)

    node = RangeToScan()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
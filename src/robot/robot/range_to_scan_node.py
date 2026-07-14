#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot.launch 使用说明：
本节点由 robot.launch.py 以 executable='range_to_scan' 启动。
它把 8 路超声波 Range 数据合成为一个稀疏 LaserScan，主要用于 Foxglove/RViz 调试和兼容。

输入：
- /ultrasonic/front_fl、front_fr、front_rl、front_rr：安装在前/后侧，随对应轮子转向关节转动。
- /ultrasonic/side_fl、side_fr、side_rl、side_rr：安装在四条腿外侧，方向相对底盘固定。
- /joint_states：读取四个转向关节角，用于更新前/后侧超声波方向。

输出：
- /scan：sensor_msgs/LaserScan，frame_id 为 base_link，角度范围 -pi 到 pi。

注意：
/scan 只是 8 路超声波展开后的稀疏扫描，不等价于真实 360 度激光雷达。
外侧超声波不跟随轮子转向；前/后侧超声波会叠加对应轮子的当前转向角。
"""

import math
from typing import Dict, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan, Range


SENSORS = [
    "front_fl",
    "front_fr",
    "front_rl",
    "front_rr",
    "side_fl",
    "side_fr",
    "side_rl",
    "side_rr",
]

# 传感器在 base_link 下的安装基准角。
# ROS 约定：0 为车头 +x，+pi/2 为左侧，-pi/2 为右侧。
FIXED_SENSOR_ANGLES = {
    "front_fl": 0.0,
    "front_fr": 0.0,
    "front_rl": math.pi,
    "front_rr": math.pi,
    "side_fl": math.pi / 2.0,
    "side_rl": math.pi / 2.0,
    "side_fr": -math.pi / 2.0,
    "side_rr": -math.pi / 2.0,
}

# 只有前/后侧超声波随轮子转向；外侧超声波固定不动。
STEERING_JOINT_BY_SENSOR = {
    "front_fl": "front_left_steer_joint",
    "front_fr": "front_right_steer_joint",
    "front_rl": "rear_left_steer_joint",
    "front_rr": "rear_right_steer_joint",
}


class RangeToScan(Node):
    """将 8 路超声波按实际安装逻辑展开成稀疏 LaserScan。"""

    def __init__(self):
        super().__init__('range_to_scan')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('angle_increment_deg', 5.0)
        self.declare_parameter('range_timeout', 0.25)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 4.0)
        self.declare_parameter('stamp_backdate', 0.05)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav_smoothed')
        self.declare_parameter('cmd_vel_timeout', 0.3)
        self.declare_parameter('ignore_steering_ultrasonic_on_pure_rotation', True)
        self.declare_parameter('rotation_linear_threshold', 0.01)
        self.declare_parameter('rotation_angular_threshold', 0.05)

        self.target_frame = str(self.get_parameter('target_frame').value)
        joint_state_topic = str(self.get_parameter('joint_state_topic').value)
        publish_period = float(self.get_parameter('publish_period').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(
            float(self.get_parameter('angle_increment_deg').value)
        )
        self.num_ranges = int(
            round((self.angle_max - self.angle_min) / self.angle_increment)
        ) + 1
        self.range_timeout = float(self.get_parameter('range_timeout').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.stamp_backdate = float(self.get_parameter('stamp_backdate').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.ignore_steering_ultrasonic_on_pure_rotation = bool(
            self.get_parameter('ignore_steering_ultrasonic_on_pure_rotation').value
        )
        self.rotation_linear_threshold = float(
            self.get_parameter('rotation_linear_threshold').value
        )
        self.rotation_angular_threshold = float(
            self.get_parameter('rotation_angular_threshold').value
        )

        # latest 保存 sensor -> (range, fov, receive_time)。
        self.latest: Dict[str, Tuple[float, float, Time]] = {}
        self.steering_angles = {
            joint_name: 0.0 for joint_name in STEERING_JOINT_BY_SENSOR.values()
        }
        self.latest_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()

        for sensor in SENSORS:
            self.create_subscription(
                Range,
                f"/ultrasonic/{sensor}",
                lambda msg, s=sensor: self.range_callback(msg, s),
                10,
            )

        self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
        )
        self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.timer = self.create_timer(publish_period, self.publish_scan)
        self.get_logger().info(
            f"range_to_scan started: period={publish_period:.3f}s, "
            f"target_frame={self.target_frame}, joint_state_topic={joint_state_topic}, "
            f"cmd_vel_topic={cmd_vel_topic}"
        )

    def range_callback(self, msg: Range, sensor: str):
        distance = msg.range if msg.min_range <= msg.range <= msg.max_range else msg.max_range
        self.latest[sensor] = (
            float(distance),
            float(msg.field_of_view),
            self.get_clock().now(),
        )

    def joint_state_callback(self, msg: JointState):
        joint_index = {name: i for i, name in enumerate(msg.name)}
        for joint_name in self.steering_angles:
            index = joint_index.get(joint_name)
            if index is not None and index < len(msg.position):
                self.steering_angles[joint_name] = float(msg.position[index])

    def cmd_vel_callback(self, msg: Twist):
        """缓存 Nav2 平滑后的速度，用于判断是否为纯原地旋转。"""
        self.latest_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def sensor_center_angle(self, sensor: str) -> float:
        base_angle = FIXED_SENSOR_ANGLES[sensor]
        joint_name = STEERING_JOINT_BY_SENSOR.get(sensor)
        if joint_name is None:
            return base_angle
        return self.normalize_angle(base_angle + self.steering_angles.get(joint_name, 0.0))

    def publish_scan(self):
        ranges = [float('inf')] * self.num_ranges
        now = self.get_clock().now()

        pure_rotation = self.is_pure_rotation(now)
        for sensor, data in self.latest.items():
            if pure_rotation and sensor in STEERING_JOINT_BY_SENSOR:
                continue
            distance, fov, receive_time = data
            if (now - receive_time).nanoseconds * 1e-9 > self.range_timeout:
                continue

            center = self.sensor_center_angle(sensor)
            start_angle = center - fov / 2.0
            end_angle = center + fov / 2.0

            angle = start_angle
            while angle <= end_angle + 1e-9:
                self.write_range_bin(ranges, angle, distance)
                angle += self.angle_increment

        scan = LaserScan()
        # 使用略早于当前时刻的时间戳，避免可视化端查询 TF 时请求到未来。
        scan.header.stamp = (
            now - Duration(seconds=self.stamp_backdate)
        ).to_msg()
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = float(self.get_parameter('publish_period').value)
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def is_pure_rotation(self, now: Time) -> bool:
        """Nav2 纯原地旋转时只屏蔽随轮转向的前/后超声波。"""
        if not self.ignore_steering_ultrasonic_on_pure_rotation:
            return False
        elapsed = (now - self.last_cmd_vel_time).nanoseconds * 1e-9
        if elapsed > self.cmd_vel_timeout:
            return False
        return (
            abs(self.latest_cmd_vel.linear.x) <= self.rotation_linear_threshold and
            abs(self.latest_cmd_vel.linear.y) <= self.rotation_linear_threshold and
            abs(self.latest_cmd_vel.angular.z) >= self.rotation_angular_threshold
        )

    def write_range_bin(self, ranges, angle: float, distance: float):
        normalized = self.normalize_angle(angle)
        index = int(round((normalized - self.angle_min) / self.angle_increment))
        if 0 <= index < self.num_ranges:
            ranges[index] = min(ranges[index], distance)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = RangeToScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

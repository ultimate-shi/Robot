#!/usr/bin/env python3
"""作用：拒绝过时/无效视觉并保护协方差；使用方法：由 state_estimation.launch.py 启动，发布 /visual_odom。"""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


POSE_VARIANCE_INDICES = {'x': 0, 'y': 7, 'z': 14,
                         'roll': 21, 'pitch': 28, 'yaw': 35}
DEFAULT_VARIANCE_FLOORS = {'x': 0.02, 'y': 0.02, 'z': 0.1,
                           'roll': 0.1, 'pitch': 0.1, 'yaw': 0.03}
LEGACY_PLANAR_AXES = frozenset(('x', 'y', 'yaw'))


def invalid_visual_reason(message, now_ns, max_age, future_tolerance,
                          quaternion_norm_tolerance=0.01):
    """检查真实采集时间与有限单位姿态；不把迟到消息重打为当前时间。"""
    stamp = message.header.stamp
    stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    age = (now_ns - stamp_ns) / 1e9
    if stamp_ns <= 0 or not -future_tolerance <= age <= max_age:
        return '视觉时间戳无效、超龄或超前'
    pose = message.pose.pose
    q = pose.orientation
    values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in values):
        return '视觉位姿含非有限数'
    if abs(sum(value * value for value in (q.x, q.y, q.z, q.w)) - 1.0) > quaternion_norm_tolerance:
        return '视觉姿态四元数无效'
    return ''


def protect_pose_covariance(covariance, floors, invalid_variance):
    """保留平面零方差兼容行为；未知三维方差标为高不确定度。"""
    invalid_axes = []
    for axis, index in POSE_VARIANCE_INDICES.items():
        value = float(covariance[index])
        if not math.isfinite(value) or value < 0.0 or (
                value == 0.0 and axis not in LEGACY_PLANAR_AXES):
            covariance[index] = invalid_variance
            invalid_axes.append(axis)
        else:
            covariance[index] = max(value, floors[axis])
    return invalid_axes


class OdometryCovarianceGuard(Node):
    """转发视觉里程计，避免未知位姿分量被赋予虚假的高可信度。"""

    def __init__(self):
        super().__init__('odometry_covariance_guard')
        self.declare_parameter('input_topic', '/visual_odom_raw')
        self.declare_parameter('output_topic', '/visual_odom')
        for axis, default in DEFAULT_VARIANCE_FLOORS.items():
            self.declare_parameter(f'{axis}_variance_floor', default)
        self.declare_parameter('invalid_variance', 1e6)
        self.declare_parameter('max_message_age', 1.5)
        self.declare_parameter('future_tolerance', 0.1)
        self.declare_parameter('quaternion_norm_tolerance', 0.01)
        self.max_message_age = float(self.get_parameter('max_message_age').value)
        self.future_tolerance = float(self.get_parameter('future_tolerance').value)
        self.quaternion_norm_tolerance = float(
            self.get_parameter('quaternion_norm_tolerance').value)
        if (not math.isfinite(self.max_message_age) or self.max_message_age <= 0.0 or
                not math.isfinite(self.future_tolerance) or self.future_tolerance < 0.0):
            raise ValueError('视觉最大年龄须为有限正数，未来容差须为有限非负数')
        if (not math.isfinite(self.quaternion_norm_tolerance) or
                not 0.0 < self.quaternion_norm_tolerance < 1.0):
            raise ValueError('四元数模平方容差必须在 0 与 1 之间')
        self.variance_floors = {
            axis: float(self.get_parameter(f'{axis}_variance_floor').value)
            for axis in POSE_VARIANCE_INDICES}
        self.invalid_variance = float(self.get_parameter('invalid_variance').value)
        if any(not math.isfinite(value) or value <= 0.0
               for value in self.variance_floors.values()):
            raise ValueError('视觉位姿协方差下限必须是有限正数')
        if (not math.isfinite(self.invalid_variance) or
                self.invalid_variance < max(self.variance_floors.values())):
            raise ValueError('无效方差替代值必须是有限数且不小于所有方差下限')
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 20)
        self.create_subscription(
            Odometry, str(self.get_parameter('input_topic').value),
            self._callback, 20)

    def _callback(self, message):
        reason = invalid_visual_reason(
            message, self.get_clock().now().nanoseconds,
            self.max_message_age, self.future_tolerance, self.quaternion_norm_tolerance)
        if reason:
            self.get_logger().warning(reason, throttle_duration_sec=5.0)
            return
        invalid_axes = protect_pose_covariance(
            message.pose.covariance, self.variance_floors, self.invalid_variance)
        if invalid_axes:
            self.get_logger().warning(
                f'视觉位姿方差无效，按高不确定度处理：{", ".join(invalid_axes)}',
                throttle_duration_sec=5.0)
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryCovarianceGuard()
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

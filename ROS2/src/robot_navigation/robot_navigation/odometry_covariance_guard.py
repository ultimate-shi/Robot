#!/usr/bin/env python3
"""使用方法：由 state_estimation.launch.py 启动，约束视觉位姿协方差后发布 /visual_odom。"""

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

#!/usr/bin/env python3
"""使用方法：把 /visual_odom_raw 的过小或空协方差提升到 EKF 安全下限。."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class OdometryCovarianceGuard(Node):
    """不改变视觉位姿，只保证 x、y、yaw 不会被 EKF 赋予无限权重。."""

    def __init__(self):
        super().__init__('odometry_covariance_guard')
        self.declare_parameter('input_topic', '/visual_odom_raw')
        self.declare_parameter('output_topic', '/visual_odom')
        self.declare_parameter('x_variance_floor', 0.02)
        self.declare_parameter('y_variance_floor', 0.02)
        self.declare_parameter('yaw_variance_floor', 0.03)
        self.x_floor = float(self.get_parameter('x_variance_floor').value)
        self.y_floor = float(self.get_parameter('y_variance_floor').value)
        self.yaw_floor = float(self.get_parameter('yaw_variance_floor').value)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 20)
        self.create_subscription(
            Odometry, str(self.get_parameter('input_topic').value),
            self._callback, 20)

    def _callback(self, message):
        message.pose.covariance[0] = max(
            float(message.pose.covariance[0]), self.x_floor)
        message.pose.covariance[7] = max(
            float(message.pose.covariance[7]), self.y_floor)
        message.pose.covariance[35] = max(
            float(message.pose.covariance[35]), self.yaw_floor)
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

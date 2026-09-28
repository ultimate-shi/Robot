#!/usr/bin/env python3
"""使用方法：置于底盘运动学与轮速控制器之间；可加载实测轮速曲线并保持超时停车。"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray

from robot_control.wheel_curve_model import WHEEL_NAMES, DIRECTIONS, WheelCurveModel


class WheelCommandGuard(Node):
    """让普通 JointGroupVelocityController 具备独立于上游进程的命令超时."""

    def __init__(self):
        super().__init__('wheel_command_guard')
        self.declare_parameter('input_topic', '/wheel_controller/commands_raw')
        self.declare_parameter('output_topic', '/wheel_controller/commands')
        self.declare_parameter('heartbeat_topic', '/hardware/command_heartbeat')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_wheel_command_rad_s', 12.0)
        self.declare_parameter('wheel_calibration.enabled', False)
        self.declare_parameter('wheel_calibration.max_compensation_ratio', 2.0)
        for wheel in WHEEL_NAMES:
            for direction in DIRECTIONS:
                prefix = f'wheel_calibration.{wheel}.{direction}'
                self.declare_parameter(f'{prefix}.commands', [-1.0])
                self.declare_parameter(f'{prefix}.measured', [-1.0])
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.timeout = float(self.get_parameter('command_timeout').value)
        rate = float(self.get_parameter('publish_rate').value)
        self.maximum = float(self.get_parameter('max_wheel_command_rad_s').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('命令超时必须为正且有限')
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('发布频率必须为正且有限')
        if not math.isfinite(self.maximum) or self.maximum <= 0:
            raise ValueError('最大轮速命令必须为正且有限')
        self.calibration = None
        if bool(self.get_parameter('wheel_calibration.enabled').value):
            curves = {}
            for wheel in WHEEL_NAMES:
                curves[wheel] = {}
                for direction in DIRECTIONS:
                    prefix = f'wheel_calibration.{wheel}.{direction}'
                    curves[wheel][direction] = (
                        self.get_parameter(f'{prefix}.commands').value,
                        self.get_parameter(f'{prefix}.measured').value)
            self.calibration = WheelCurveModel(
                curves, self.maximum,
                self.get_parameter('wheel_calibration.max_compensation_ratio').value)
        self.latest = [0.0] * 4
        self.last_message = None
        self.publisher = self.create_publisher(
            Float64MultiArray, self.output_topic, 10)
        self.heartbeat = self.create_publisher(
            Empty, str(self.get_parameter('heartbeat_topic').value), 10)
        self.create_subscription(
            Float64MultiArray, self.input_topic, self._command_callback, 10)
        self.create_timer(1.0 / rate, self._timer_callback)

    def _command_callback(self, message):
        if len(message.data) != 4:
            self.latest = [0.0] * 4
            self.last_message = None
            self.get_logger().error('轮速命令必须恰好包含四个元素')
            return
        if not all(math.isfinite(value) for value in message.data):
            self.latest = [0.0] * 4
            self.last_message = None
            self.get_logger().error('轮速命令含非有限值，已停车')
            return
        self.latest = [float(value) for value in message.data]
        self.last_message = time.monotonic()

    def _timer_callback(self):
        fresh = (self.last_message is not None and
                 time.monotonic() - self.last_message <= self.timeout)
        output = [0.0] * 4
        if fresh:
            for index, target in enumerate(self.latest):
                if self.calibration is not None:
                    target = self.calibration.compensate(WHEEL_NAMES[index], target)
                output[index] = max(-self.maximum, min(self.maximum, target))
        self.publisher.publish(Float64MultiArray(data=output))
        self.heartbeat.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = WheelCommandGuard()
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

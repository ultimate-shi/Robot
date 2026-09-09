#!/usr/bin/env python3
"""使用方法：未来 MCU 发布四轮 JointState 后，本节点输出不带 TF 的 /wheel/odom。"""

import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class FourWheelOdometry(Node):
    """由四轮实际转角和轮速计算四轮独立转向底盘的瞬时速度。"""

    def __init__(self):
        super().__init__('four_wheel_odometry')
        defaults = {
            'enabled': False,
            'joint_state_topic': '/joint_states',
            'output_topic': '/wheel/odom',
            'status_topic': '/wheel/odometry_status',
            'wheelbase': 0.4,
            'track': 0.2,
            'wheel_radius': 0.05,
            'feedback_timeout': 0.25,
            'max_wheel_speed': 40.0,
            'velocity_variance': 0.02,
            'yaw_rate_variance': 0.03,
            'wheel_velocity_sign': [1.0, 1.0, 1.0, 1.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)
        self.steer_joints = [
            'front_left_steer_joint', 'front_right_steer_joint',
            'rear_left_steer_joint', 'rear_right_steer_joint']
        self.wheel_joints = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint']
        signs = np.asarray(self.wheel_velocity_sign, dtype=float)
        if signs.shape != (4,) or np.any(np.abs(np.abs(signs) - 1.0) > 1e-6):
            raise ValueError('wheel_velocity_sign 必须包含四个 +1 或 -1')
        self.wheel_velocity_sign = signs
        self.odom_pub = self.create_publisher(Odometry, str(self.output_topic), 20)
        self.status_pub = self.create_publisher(String, str(self.status_topic), 10)
        self.create_subscription(
            JointState, str(self.joint_state_topic), self._joint_callback, 20)
        self.last_valid_monotonic = None
        self.last_state = 'disabled' if not self.enabled else 'waiting'
        self.create_timer(1.0, self._publish_status)

    def _joint_callback(self, message):
        if not bool(self.enabled):
            return
        index = {name: position for position, name in enumerate(message.name)}
        required = self.steer_joints + self.wheel_joints
        if any(name not in index for name in required):
            self.last_state = 'missing_joint'
            return
        if any(index[name] >= len(message.position) for name in self.steer_joints):
            self.last_state = 'missing_steering_position'
            return
        if any(index[name] >= len(message.velocity) for name in self.wheel_joints):
            self.last_state = 'missing_wheel_velocity'
            return
        steering = np.array(
            [message.position[index[name]] for name in self.steer_joints], dtype=float)
        wheel_speed = np.array(
            [message.velocity[index[name]] for name in self.wheel_joints], dtype=float)
        wheel_speed *= self.wheel_velocity_sign
        if not np.isfinite(steering).all() or not np.isfinite(wheel_speed).all():
            self.last_state = 'non_finite'
            return
        if np.any(np.abs(wheel_speed) > float(self.max_wheel_speed)):
            self.last_state = 'wheel_speed_out_of_range'
            return
        vx, vy, yaw_rate = self.compute_twist(
            steering, wheel_speed, float(self.wheel_radius),
            float(self.wheelbase), float(self.track))
        self._publish_odom(message, vx, vy, yaw_rate)
        self.last_valid_monotonic = time.monotonic()
        self.last_state = 'ok'

    @staticmethod
    def compute_twist(steering, wheel_speed, radius, wheelbase, track):
        """按四个轮心的刚体速度场最小二乘求解 vx、vy、yaw_rate。"""
        positions = np.array([
            [wheelbase / 2.0, track / 2.0],
            [wheelbase / 2.0, -track / 2.0],
            [-wheelbase / 2.0, track / 2.0],
            [-wheelbase / 2.0, -track / 2.0],
        ])
        linear_speed = np.asarray(wheel_speed) * radius
        measured = np.column_stack((
            linear_speed * np.cos(steering),
            linear_speed * np.sin(steering))).reshape(-1)
        matrix = np.zeros((8, 3), dtype=float)
        for wheel, (x_pos, y_pos) in enumerate(positions):
            matrix[2 * wheel] = [1.0, 0.0, -y_pos]
            matrix[2 * wheel + 1] = [0.0, 1.0, x_pos]
        solution, _, _, _ = np.linalg.lstsq(matrix, measured, rcond=None)
        return tuple(float(value) for value in solution)

    def _publish_odom(self, source, vx, vy, yaw_rate):
        message = Odometry()
        message.header.stamp = source.header.stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 1e6
        message.pose.covariance[7] = 1e6
        message.pose.covariance[35] = 1e6
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = yaw_rate
        message.twist.covariance[0] = float(self.velocity_variance)
        message.twist.covariance[7] = float(self.velocity_variance)
        message.twist.covariance[35] = float(self.yaw_rate_variance)
        self.odom_pub.publish(message)

    def _publish_status(self):
        age = (
            None if self.last_valid_monotonic is None
            else time.monotonic() - self.last_valid_monotonic)
        state = self.last_state
        if bool(self.enabled) and age is not None and age > float(self.feedback_timeout):
            state = 'stale'
        self.status_pub.publish(String(data=json.dumps({
            'state': state,
            'enabled': bool(self.enabled),
            'data_age': None if age is None else round(age, 3),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = FourWheelOdometry()
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


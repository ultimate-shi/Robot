#!/usr/bin/env python3
"""使用方法：订阅 /joint_states，发布 /wheel/odom 和轮间一致性状态."""

import math
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_interfaces.msg import WheelOdometryStatus
from sensor_msgs.msg import JointState


class FourWheelOdometry(Node):
    """由实际转角、轮速、连续编码器位置和腿角估计底盘瞬时速度."""

    def __init__(self):
        super().__init__('four_wheel_odometry')
        defaults = {
            'enabled': True,
            'joint_state_topic': '/joint_states',
            'output_topic': '/wheel/odom',
            'status_topic': '/wheel/odometry_status',
            'wheelbase': 0.312,
            'track': 0.280,
            'wheel_radius': 0.055,
            'leg_length': 0.060,
            'feedback_timeout': 0.25,
            'max_sync_skew': 0.10,
            'max_wheel_speed': 40.0,
            'soft_residual': 0.05,
            'hard_residual': 0.15,
            'minimum_effective_wheels': 3,
            'irls_iterations': 5,
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
        self.leg_joints = [
            'lap_body_joint_fl', 'lap_body_joint_fr',
            'lap_body_joint_rl', 'lap_body_joint_rr']
        signs = np.asarray(self.wheel_velocity_sign, dtype=float)
        if signs.shape != (4,) or np.any(np.abs(np.abs(signs) - 1.0) > 1e-6):
            raise ValueError('wheel_velocity_sign 必须包含四个 +1 或 -1')
        self.wheel_velocity_sign = signs
        self.odom_pub = self.create_publisher(Odometry, str(self.output_topic), 20)
        self.status_pub = self.create_publisher(
            WheelOdometryStatus, str(self.status_topic), 10)
        self.create_subscription(
            JointState, str(self.joint_state_topic), self._joint_callback, 20)
        self.last_valid_monotonic = None
        self.last_state = 'disabled' if not self.enabled else 'waiting'
        self.last_residual = np.zeros(4)
        self.last_weight = np.zeros(4)
        self.last_fault = np.zeros(4, dtype=bool)
        self.last_covariance_scale = 1.0
        self.last_sync_skew = 0.0
        self.last_wheel_position = None
        self.last_stamp_seconds = None
        self.create_timer(0.2, self._publish_status)

    def _joint_callback(self, message):
        if not bool(self.enabled):
            return
        index = {name: position for position, name in enumerate(message.name)}
        required = self.steer_joints + self.wheel_joints + self.leg_joints
        if any(name not in index for name in required):
            self.last_state = 'missing_joint'
            return
        if any(index[name] >= len(message.position)
               for name in self.steer_joints + self.leg_joints):
            self.last_state = 'missing_position'
            return
        if any(index[name] >= len(message.velocity) for name in self.wheel_joints):
            self.last_state = 'missing_wheel_velocity'
            return

        steering = np.asarray(
            [message.position[index[name]] for name in self.steer_joints],
            dtype=float)
        leg_angles = np.asarray(
            [message.position[index[name]] for name in self.leg_joints],
            dtype=float)
        wheel_speed = np.asarray(
            [message.velocity[index[name]] for name in self.wheel_joints],
            dtype=float) * self.wheel_velocity_sign
        wheel_position = np.asarray(
            [message.position[index[name]] for name in self.wheel_joints],
            dtype=float) * self.wheel_velocity_sign
        values = np.concatenate((steering, leg_angles, wheel_speed, wheel_position))
        if not np.isfinite(values).all():
            self.last_state = 'non_finite'
            return
        if np.any(np.abs(wheel_speed) > float(self.max_wheel_speed)):
            self.last_state = 'wheel_speed_out_of_range'
            return

        stamp_seconds = (
            float(message.header.stamp.sec) +
            float(message.header.stamp.nanosec) * 1e-9)
        if stamp_seconds <= 0.0:
            stamp_seconds = self.get_clock().now().nanoseconds * 1e-9
        encoder_fault = np.zeros(4, dtype=bool)
        if self.last_wheel_position is not None and self.last_stamp_seconds is not None:
            dt = stamp_seconds - self.last_stamp_seconds
            if 1e-3 <= dt <= float(self.feedback_timeout):
                position_velocity = (
                    wheel_position - self.last_wheel_position) / dt
                mismatch_mps = np.abs(position_velocity - wheel_speed) * float(
                    self.wheel_radius)
                encoder_fault = mismatch_mps >= float(self.hard_residual)
        self.last_wheel_position = wheel_position
        self.last_stamp_seconds = stamp_seconds

        positions = self.wheel_positions(
            leg_angles, float(self.wheelbase), float(self.track),
            float(self.leg_length))
        solution, residual, weight, suspected = self.robust_twist(
            steering, wheel_speed, float(self.wheel_radius), positions,
            float(self.soft_residual), float(self.hard_residual),
            int(self.irls_iterations))
        suspected = np.logical_or(suspected, encoder_fault)
        weight[suspected] = 0.0
        effective = int(np.count_nonzero(weight > 0.05))
        self.last_residual = residual
        self.last_weight = weight
        self.last_fault = suspected
        self.last_sync_skew = 0.0
        residual_scale = max(1.0, float(np.mean(residual)) /
                             max(float(self.soft_residual), 1e-6))
        wheel_scale = 4.0 / max(effective, 1)
        self.last_covariance_scale = min(100.0, residual_scale * wheel_scale)
        if effective < int(self.minimum_effective_wheels):
            self.last_state = 'insufficient_wheels'
            return
        if np.any(suspected):
            solution, _, _, _ = self.robust_twist(
                steering, wheel_speed, float(self.wheel_radius), positions,
                float(self.soft_residual), float(self.hard_residual),
                int(self.irls_iterations), initial_weight=weight)
        self._publish_odom(message, *solution)
        self.last_valid_monotonic = time.monotonic()
        self.last_state = 'degraded' if np.any(suspected) else 'ok'

    @staticmethod
    def wheel_positions(leg_angles, wheelbase, track, leg_length):
        """复现旧 chassis.c 的腿角动态轴距，顺序为左前、右前、左后、右后."""
        leg_angles = np.asarray(leg_angles, dtype=float)
        x_pos = np.empty(4, dtype=float)
        x_pos[:2] = (wheelbase / 2.0 + leg_length *
                     np.cos(leg_angles[:2]) - leg_length)
        x_pos[2:] = (-wheelbase / 2.0 + leg_length - leg_length *
                     np.cos(leg_angles[2:]))
        return np.column_stack((
            x_pos, np.asarray([track / 2.0, -track / 2.0,
                               track / 2.0, -track / 2.0])))

    @staticmethod
    def _system(steering, wheel_speed, radius, positions):
        linear_speed = np.asarray(wheel_speed, dtype=float) * radius
        measured = np.column_stack((
            linear_speed * np.cos(steering),
            linear_speed * np.sin(steering))).reshape(-1)
        matrix = np.zeros((8, 3), dtype=float)
        for wheel, (x_pos, y_pos) in enumerate(positions):
            matrix[2 * wheel] = [1.0, 0.0, -y_pos]
            matrix[2 * wheel + 1] = [0.0, 1.0, x_pos]
        return matrix, measured

    @staticmethod
    def robust_twist(steering, wheel_speed, radius, positions,
                     soft_residual, hard_residual, iterations,
                     initial_weight=None):
        """用按轮 Huber IRLS 降低单轮堵转、打滑和编码器异常的影响."""
        matrix, measured = FourWheelOdometry._system(
            steering, wheel_speed, radius, positions)
        weight = (np.ones(4, dtype=float) if initial_weight is None else
                  np.asarray(initial_weight, dtype=float).copy())
        solution = np.zeros(3, dtype=float)
        residual = np.full(4, math.inf)
        for _ in range(max(1, iterations)):
            row_weight = np.repeat(np.sqrt(np.clip(weight, 0.0, 1.0)), 2)
            if np.count_nonzero(row_weight) < 6:
                break
            solution, _, _, _ = np.linalg.lstsq(
                matrix * row_weight[:, None], measured * row_weight,
                rcond=None)
            error = (matrix @ solution - measured).reshape(4, 2)
            residual = np.linalg.norm(error, axis=1)
            huber = np.ones(4, dtype=float)
            above = residual > soft_residual
            huber[above] = soft_residual / np.maximum(residual[above], 1e-9)
            if initial_weight is not None:
                huber *= np.asarray(initial_weight, dtype=float)
            if np.allclose(huber, weight, atol=1e-3):
                weight = huber
                break
            weight = huber
        suspected = residual >= hard_residual
        weight[suspected] = 0.0
        return solution, residual, weight, suspected

    @staticmethod
    def compute_twist(steering, wheel_speed, radius, wheelbase, track):
        """兼容旧测试接口的普通最小二乘计算."""
        positions = FourWheelOdometry.wheel_positions(
            np.zeros(4), wheelbase, track, 0.0)
        matrix, measured = FourWheelOdometry._system(
            steering, wheel_speed, radius, positions)
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
        message.twist.twist.linear.x = float(vx)
        message.twist.twist.linear.y = float(vy)
        message.twist.twist.angular.z = float(yaw_rate)
        scale = float(self.last_covariance_scale)
        message.twist.covariance[0] = float(self.velocity_variance) * scale
        message.twist.covariance[7] = float(self.velocity_variance) * scale
        message.twist.covariance[35] = float(self.yaw_rate_variance) * scale
        self.odom_pub.publish(message)

    def _publish_status(self):
        age = (-1.0 if self.last_valid_monotonic is None else
               time.monotonic() - self.last_valid_monotonic)
        state = self.last_state
        if bool(self.enabled) and age >= 0.0 and age > float(self.feedback_timeout):
            state = 'stale'
        message = WheelOdometryStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.state = state
        message.residual_mps = self.last_residual.tolist()
        message.weight = self.last_weight.tolist()
        message.suspected_fault = self.last_fault.tolist()
        message.effective_wheels = int(np.count_nonzero(self.last_weight > 0.05))
        message.covariance_scale = float(self.last_covariance_scale)
        message.data_age = age
        message.sync_skew = float(self.last_sync_skew)
        self.status_pub.publish(message)


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

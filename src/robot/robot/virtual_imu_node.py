"""
Virtual IMU Node - Simulates IMU sensor data from odometry.

Subscribes to /odom (6DOF) and generates realistic sensor_msgs/Imu
messages at 20Hz with GY25T-matching noise characteristics.

Computes:
- Orientation: directly from odom quaternion + noise
- Angular velocity: numerical differentiation of orientation + noise
- Linear acceleration: velocity differentiation + gravity rotation + noise
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class VirtualIMUNode(Node):

    def __init__(self):
        super().__init__('virtual_imu')

        # Parameters
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("orientation_noise_std", 0.0087)  # ~0.5 deg
        self.declare_parameter("angular_vel_noise_std", 0.005)   # rad/s
        self.declare_parameter("linear_accel_noise_std", 0.02)   # m/s^2
        self.declare_parameter("gravity", 9.81)

        self.publish_rate = self.get_parameter("publish_rate").value
        self.orientation_noise = self.get_parameter("orientation_noise_std").value
        self.angular_vel_noise = self.get_parameter("angular_vel_noise_std").value
        self.linear_accel_noise = self.get_parameter("linear_accel_noise_std").value
        self.gravity = self.get_parameter("gravity").value

        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer at publish_rate Hz
        period = 1.0 / self.publish_rate
        self.create_timer(period, self.timer_callback)

        # State
        self.latest_odom = None
        self.prev_odom = None
        self.prev_time = None
        self.prev_linear_vel = np.zeros(3)  # body frame velocity

        # Covariance matrices (diagonal)
        self.orientation_cov = [self.orientation_noise ** 2] * 3
        self.angular_vel_cov = [self.angular_vel_noise ** 2] * 3
        self.linear_accel_cov = [self.linear_accel_noise ** 2] * 3

        self.get_logger().info(
            f"VirtualIMU started: {self.publish_rate}Hz, "
            f"noise: orient={self.orientation_noise:.4f}rad, "
            f"gyro={self.angular_vel_noise:.4f}rad/s, "
            f"accel={self.linear_accel_noise:.3f}m/s^2"
        )

    def odom_callback(self, msg: Odometry):
        """Cache latest odometry message."""
        self.prev_odom = self.latest_odom
        self.latest_odom = msg

    def timer_callback(self):
        """Publish IMU data at configured rate."""
        if self.latest_odom is None:
            return

        now = self.get_clock().now()
        odom = self.latest_odom

        # Extract orientation quaternion from odom
        q = odom.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Extract velocities from odom twist (in body frame)
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        vz = odom.twist.twist.linear.z
        wx = odom.twist.twist.angular.x
        wy = odom.twist.twist.angular.y
        wz = odom.twist.twist.angular.z

        # Compute dt for differentiation
        dt = 1.0 / self.publish_rate
        if self.prev_time is not None:
            elapsed = (now - self.prev_time).nanoseconds * 1e-9
            if elapsed > 0.001:
                dt = elapsed

        # --- Angular velocity ---
        # Use odom twist angular values + noise
        angular_velocity = np.array([wx, wy, wz])
        angular_velocity += np.random.normal(0, self.angular_vel_noise, 3)

        # --- Linear acceleration ---
        # Differentiate velocity
        current_vel = np.array([vx, vy, vz])
        accel = (current_vel - self.prev_linear_vel) / dt

        # Add gravity component in body frame
        # g_body = R^T * [0, 0, g] where R is body-to-world rotation
        rot_matrix = self._quat_to_rotmat(qw, qx, qy, qz)
        g_world = np.array([0.0, 0.0, self.gravity])
        g_body = rot_matrix.T @ g_world  # Rotate gravity into body frame

        linear_acceleration = accel + g_body
        linear_acceleration += np.random.normal(0, self.linear_accel_noise, 3)

        # Update previous state
        self.prev_linear_vel = current_vel.copy()
        self.prev_time = now

        # --- Orientation with noise ---
        # Add small noise to quaternion (via euler perturbation)
        noise_r = np.random.normal(0, self.orientation_noise)
        noise_p = np.random.normal(0, self.orientation_noise)
        noise_y = np.random.normal(0, self.orientation_noise)
        # Convert noise to quaternion delta and multiply
        dqx, dqy, dqz, dqw = self._euler_to_quaternion(noise_r, noise_p, noise_y)
        # Quaternion multiplication: q_noisy = q * dq
        nqw = qw * dqw - qx * dqx - qy * dqy - qz * dqz
        nqx = qw * dqx + qx * dqw + qy * dqz - qz * dqy
        nqy = qw * dqy - qx * dqz + qy * dqw + qz * dqx
        nqz = qw * dqz + qx * dqy - qy * dqx + qz * dqw
        # Normalize
        norm = math.sqrt(nqx ** 2 + nqy ** 2 + nqz ** 2 + nqw ** 2)
        if norm > 1e-8:
            nqx /= norm
            nqy /= norm
            nqz /= norm
            nqw /= norm

        # --- Build IMU message ---
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Orientation
        imu_msg.orientation.x = nqx
        imu_msg.orientation.y = nqy
        imu_msg.orientation.z = nqz
        imu_msg.orientation.w = nqw
        imu_msg.orientation_covariance = self._build_covariance(self.orientation_cov)

        # Angular velocity
        imu_msg.angular_velocity.x = float(angular_velocity[0])
        imu_msg.angular_velocity.y = float(angular_velocity[1])
        imu_msg.angular_velocity.z = float(angular_velocity[2])
        imu_msg.angular_velocity_covariance = self._build_covariance(self.angular_vel_cov)

        # Linear acceleration
        imu_msg.linear_acceleration.x = float(linear_acceleration[0])
        imu_msg.linear_acceleration.y = float(linear_acceleration[1])
        imu_msg.linear_acceleration.z = float(linear_acceleration[2])
        imu_msg.linear_acceleration_covariance = self._build_covariance(self.linear_accel_cov)

        self.imu_pub.publish(imu_msg)

    def _build_covariance(self, diag_values):
        """Build 9-element covariance array (3x3 diagonal)."""
        cov = [0.0] * 9
        cov[0] = diag_values[0]
        cov[4] = diag_values[1]
        cov[8] = diag_values[2]
        return cov

    def _quat_to_rotmat(self, w, x, y, z):
        """Quaternion to 3x3 rotation matrix."""
        return np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
        ], dtype=np.float64)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion (x, y, z, w)."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = VirtualIMUNode()
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

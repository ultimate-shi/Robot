"""
Chassis Controller 3D - Enhanced 6DOF chassis controller with terrain awareness.

Replicates all functionality from chassis_controller_node.py (crab/four_ws/ackermann
modes, minimum steering optimization, low-pass filtering, odometry integration)
and adds:
- Terrain height map query for Z/roll/pitch
- Physical constraints (slope slip, step blocking, drop-off blocking)
- 6DOF odometry and TF publishing
- Terrain status diagnostics
"""

import math
import json
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory

from robot.terrain_heightmap import TerrainHeightmap
from robot.terrain_physics import TerrainPhysics


class ChassisController3D(Node):

    def __init__(self):
        super().__init__('chassis_controller')

        # ==================== Parameters ====================
        self.declare_parameter("wheelbase", 0.4)
        self.declare_parameter("track", 0.2)
        self.declare_parameter("radius", 0.05)
        self.declare_parameter("motion_mode", "crab")
        # Terrain parameters
        self.declare_parameter("terrain_check_enabled", True)
        self.declare_parameter("grid_resolution", 0.02)
        self.declare_parameter("ground_tolerance", 0.05)
        self.declare_parameter("ground_to_base_height", 0.15)
        self.declare_parameter("max_grade_deg", 35.0)
        self.declare_parameter("step_threshold", 0.03)
        self.declare_parameter("dropoff_threshold", 0.05)
        self.declare_parameter("look_ahead_distance", 0.10)
        self.declare_parameter("look_ahead_samples", 5)
        self.declare_parameter("ply_path", "")

        self.wheel_base = self.get_parameter("wheelbase").value
        self.wheel_track = self.get_parameter("track").value
        self.wheel_radius = self.get_parameter("radius").value
        self.motion_mode = self.get_parameter("motion_mode").value
        self.terrain_enabled = self.get_parameter("terrain_check_enabled").value

        self.Lx = self.wheel_base / 2.0
        self.Ly = self.wheel_track / 2.0

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Control rate
        self.control_rate = 0.1  # 10Hz
        self.latest_cmd_vel = Twist()

        # ==================== ROS interfaces ====================
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/wheel_states', self.wheel_state_callback, 10)

        self.steer_pub = self.create_publisher(Float64MultiArray, '/steering_controller/commands', 10)
        self.speed_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)
        self.mode_pub = self.create_publisher(String, '/chassis_mode', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.terrain_status_pub = self.create_publisher(String, '/terrain_status', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 10Hz control loop
        self.control_timer = self.create_timer(self.control_rate, self.control_loop)

        # ==================== State variables ====================
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.vx_filtered = 0.0
        self.vy_filtered = 0.0
        self.w_filtered = 0.0
        self.filter_alpha = 0.2

        self.prev_angles = [0.0, 0.0, 0.0, 0.0]

        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timeout = 0.5
        self.has_received_cmd = False

        # Terrain state
        self.slip_factor = 1.0
        self.slip_factor_filtered = 1.0
        self.terrain_blocked = False
        self.block_reason = ""

        # Previous position for rollback
        self.prev_x = 0.0
        self.prev_y = 0.0

        # ==================== Initialize terrain ====================
        self.heightmap = None
        self.physics = None

        if self.terrain_enabled:
            self._init_terrain()

        # Publish initial mode
        self.mode_pub.publish(String(data=self.motion_mode))
        self.get_logger().info(
            f"ChassisController3D started | mode: {self.motion_mode} | "
            f"terrain: {'ON' if self.heightmap else 'OFF'}"
        )

    def _init_terrain(self):
        """Initialize terrain heightmap and physics engine."""
        ply_path = self.get_parameter("ply_path").value

        if not ply_path:
            # Auto-detect from robot package
            try:
                pkg_path = get_package_share_directory("robot")
                ply_path = os.path.join(pkg_path, "map", "studyroom.ply")
            except Exception:
                # Fallback to source path
                ply_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "map", "studyroom.ply"
                )

        if not os.path.exists(ply_path):
            self.get_logger().warn(f"PLY not found: {ply_path}, terrain disabled")
            self.terrain_enabled = False
            return

        try:
            self.get_logger().info(f"Loading terrain from: {ply_path}")
            self.heightmap = TerrainHeightmap(
                ply_path=ply_path,
                resolution=self.get_parameter("grid_resolution").value,
                ground_tolerance=self.get_parameter("ground_tolerance").value
            )
            self.physics = TerrainPhysics(
                max_grade_deg=self.get_parameter("max_grade_deg").value,
                step_threshold=self.get_parameter("step_threshold").value,
                dropoff_threshold=self.get_parameter("dropoff_threshold").value,
                look_ahead_distance=self.get_parameter("look_ahead_distance").value,
                look_ahead_samples=self.get_parameter("look_ahead_samples").value,
                ground_to_base_height=self.get_parameter("ground_to_base_height").value,
                wheelbase=self.wheel_base,
                track=self.wheel_track
            )
            self.get_logger().info(
                f"Terrain loaded: grid {self.heightmap.grid_w}x{self.heightmap.grid_h}, "
                f"ground_z={self.heightmap.ground_z_base:.3f}"
            )
        except Exception as e:
            self.get_logger().error(f"Terrain init failed: {e}")
            self.heightmap = None
            self.physics = None
            self.terrain_enabled = False

    # ==================== Parameter callback ====================
    def parameter_callback(self, params):
        for param in params:
            if param.name == "motion_mode":
                self.motion_mode = param.value
                self.get_logger().info(f"Mode changed: {self.motion_mode}")
                self.mode_pub.publish(String(data=self.motion_mode))
        return SetParametersResult(successful=True)

    # ==================== cmd_vel callback ====================
    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg
        self.has_received_cmd = True
        self.last_cmd_vel_time = self.get_clock().now()

    # ==================== 10Hz control loop ====================
    def control_loop(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_vel_time).nanoseconds * 1e-9 > self.cmd_vel_timeout:
            self.send_stop()
            return
        if not self.has_received_cmd:
            self.send_stop()
            return

        vx = self.latest_cmd_vel.linear.x
        vy = self.latest_cmd_vel.linear.y
        w = self.latest_cmd_vel.angular.z

        # Mode kinematics
        if self.motion_mode == "crab":
            angles, speeds = self.mode_crab(vx, vy)
        elif self.motion_mode == "four_ws":
            angles, speeds = self.mode_four_ws(vx, w)
        elif self.motion_mode == "ackermann":
            angles, speeds = self.mode_ackermann(vx, w)
        else:
            angles, speeds = self.stop_command()

        # Minimum steering angle optimization
        opt_angles = []
        opt_speeds = []
        for i in range(4):
            a, s = self.optimize_steering(angles[i], speeds[i], self.prev_angles[i])
            opt_angles.append(a)
            opt_speeds.append(s)
        self.prev_angles = opt_angles

        # Apply terrain slip to wheel speeds
        if self.terrain_enabled and self.slip_factor_filtered < 1.0:
            opt_speeds = [s * self.slip_factor_filtered for s in opt_speeds]

        # If terrain blocked, stop
        if self.terrain_blocked:
            opt_speeds = [0.0, 0.0, 0.0, 0.0]

        # Publish
        self.steer_pub.publish(Float64MultiArray(data=opt_angles))
        self.speed_pub.publish(Float64MultiArray(data=opt_speeds))

    # ==================== Motion modes ====================
    def mode_crab(self, vx, vy):
        if abs(vx) < 1e-3 and abs(vy) < 1e-3:
            return self.stop_command()
        angle = math.atan2(vy, vx)
        speed = math.hypot(vx, vy) / self.wheel_radius
        return [angle] * 4, [speed] * 4

    def mode_four_ws(self, vx, w):
        if abs(vx) < 1e-3 and abs(w) < 1e-3:
            return self.stop_command()
        if abs(w) < 1e-5:
            return [0.0] * 4, [vx / self.wheel_radius] * 4

        R = vx / w
        delta_fl = math.atan2(self.wheel_base / 2, R - self.wheel_track / 2)
        delta_fr = math.atan2(self.wheel_base / 2, R + self.wheel_track / 2)
        delta_rl = -math.atan2(self.wheel_base / 2, R - self.wheel_track / 2)
        delta_rr = -math.atan2(self.wheel_base / 2, R + self.wheel_track / 2)

        v_fl = abs(w) * math.hypot(R - self.wheel_track / 2, self.wheel_base / 2)
        v_fr = abs(w) * math.hypot(R + self.wheel_track / 2, self.wheel_base / 2)
        v_rl = abs(w) * math.hypot(R - self.wheel_track / 2, self.wheel_base / 2)
        v_rr = abs(w) * math.hypot(R + self.wheel_track / 2, self.wheel_base / 2)

        sign = 1.0 if vx >= 0 else -1.0
        angles = [delta_fl, delta_fr, delta_rl, delta_rr]
        speeds = [
            (v_fl * sign) / self.wheel_radius,
            (v_fr * sign) / self.wheel_radius,
            (v_rl * sign) / self.wheel_radius,
            (v_rr * sign) / self.wheel_radius
        ]
        return angles, speeds

    def mode_ackermann(self, vx, w):
        if abs(vx) < 1e-3 and abs(w) < 1e-3:
            return self.stop_command()
        if abs(w) < 1e-3:
            return [0, 0, 0, 0], [vx / self.wheel_radius] * 4

        R_icr = vx / w
        delta_l = math.atan(self.wheel_base / (R_icr - self.wheel_track / 2))
        delta_r = math.atan(self.wheel_base / (R_icr + self.wheel_track / 2))

        v_fl = w * math.hypot(R_icr - self.wheel_track / 2, self.wheel_base)
        v_fr = w * math.hypot(R_icr + self.wheel_track / 2, self.wheel_base)
        v_rl = w * (R_icr - self.wheel_track / 2)
        v_rr = w * (R_icr + self.wheel_track / 2)

        angles = [delta_l, delta_r, 0.0, 0.0]
        speeds = [
            v_fl / self.wheel_radius,
            v_fr / self.wheel_radius,
            v_rl / self.wheel_radius,
            v_rr / self.wheel_radius
        ]
        return angles, speeds

    def optimize_steering(self, target_angle, target_speed, prev_angle):
        a1 = math.atan2(math.sin(target_angle), math.cos(target_angle))
        s1 = target_speed
        a2 = math.atan2(math.sin(target_angle + math.pi), math.cos(target_angle + math.pi))
        s2 = -target_speed

        if abs(a1) <= math.pi / 2:
            return a1, s1
        else:
            return a2, s2

    def stop_command(self):
        return [0.0] * 4, [0.0] * 4

    def send_stop(self):
        ang, spd = self.stop_command()
        self.steer_pub.publish(Float64MultiArray(data=ang))
        self.speed_pub.publish(Float64MultiArray(data=spd))

    # ==================== Odometry with terrain ====================
    def wheel_state_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        if dt <= 0.0:
            return

        fl_s, fr_s, rl_s, rr_s = msg.data[0:4]
        fl_v, fr_v, rl_v, rr_v = np.array(msg.data[4:8]) * self.wheel_radius

        # Chassis velocity decomposition
        vx = np.mean([
            fl_v * math.cos(fl_s), fr_v * math.cos(fr_s),
            rl_v * math.cos(rl_s), rr_v * math.cos(rr_s)
        ])
        vy = np.mean([
            fl_v * math.sin(fl_s), fr_v * math.sin(fr_s),
            rl_v * math.sin(rl_s), rr_v * math.sin(rr_s)
        ])

        # Angular velocity (4WIS kinematics)
        w = ((-fl_v * math.cos(fl_s) * self.Ly + fl_v * math.sin(fl_s) * self.Lx) +
             (-fr_v * math.cos(fr_s) * (-self.Ly) + fr_v * math.sin(fr_s) * self.Lx) +
             (-rl_v * math.cos(rl_s) * self.Ly + rl_v * math.sin(rl_s) * (-self.Lx)) +
             (-rr_v * math.cos(rr_s) * (-self.Ly) + rr_v * math.sin(rr_s) * (-self.Lx))) / \
            (4.0 * (self.Lx ** 2 + self.Ly ** 2))

        # Low-pass filter
        self.vx_filtered = self.filter_alpha * vx + (1 - self.filter_alpha) * self.vx_filtered
        self.vy_filtered = self.filter_alpha * vy + (1 - self.filter_alpha) * self.vy_filtered
        self.w_filtered = self.filter_alpha * w + (1 - self.filter_alpha) * self.w_filtered

        # Save previous position for rollback
        self.prev_x = self.x
        self.prev_y = self.y

        # 2D pose integration
        self.yaw += self.w_filtered * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        dx = (self.vx_filtered * math.cos(self.yaw) - self.vy_filtered * math.sin(self.yaw)) * dt
        dy = (self.vx_filtered * math.sin(self.yaw) + self.vy_filtered * math.cos(self.yaw)) * dt

        # Apply slip factor
        dx *= self.slip_factor_filtered
        dy *= self.slip_factor_filtered

        self.x += dx
        self.y += dy

        # ==================== Terrain query ====================
        if self.terrain_enabled and self.heightmap and self.physics:
            constraint = self.physics.evaluate(
                self.heightmap, self.x, self.y, self.yaw, self.vx_filtered
            )

            # Update terrain state
            self.terrain_blocked = constraint.is_blocked
            self.block_reason = constraint.block_reason
            self.slip_factor = constraint.slip_factor

            # Smooth slip factor
            slip_alpha = 0.3
            self.slip_factor_filtered = (slip_alpha * self.slip_factor +
                                         (1 - slip_alpha) * self.slip_factor_filtered)

            # If blocked, rollback position
            if constraint.is_blocked:
                self.x = self.prev_x
                self.y = self.prev_y

            # Update 3D pose from terrain
            self.z = constraint.body_z
            self.roll = constraint.roll
            self.pitch = constraint.pitch

            # Publish terrain status
            self._publish_terrain_status(constraint)
        else:
            self.z = 0.0
            self.roll = 0.0
            self.pitch = 0.0

        # Publish 6DOF odometry and TF
        self.publish_odom(current_time)

    def _publish_terrain_status(self, constraint):
        """Publish terrain status as JSON string."""
        status = {
            "is_blocked": constraint.is_blocked,
            "block_reason": constraint.block_reason,
            "slip_factor": round(constraint.slip_factor, 3),
            "traversability": round(constraint.traversability, 3),
            "body_z": round(constraint.body_z, 4),
            "roll_deg": round(math.degrees(constraint.roll), 2),
            "pitch_deg": round(math.degrees(constraint.pitch), 2),
            "step_blocked": constraint.block_reason == "step",
            "dropoff_blocked": constraint.block_reason == "dropoff",
        }
        msg = String()
        msg.data = json.dumps(status)
        self.terrain_status_pub.publish(msg)

    # ==================== 6DOF Odometry publishing ====================
    def publish_odom(self, time):
        # Convert roll, pitch, yaw to quaternion
        qx, qy, qz, qw = self._euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx_filtered
        odom.twist.twist.linear.y = self.vy_filtered
        odom.twist.twist.angular.z = self.w_filtered
        self.odom_pub.publish(odom)

        # TF broadcast
        tf = TransformStamped()
        tf.header.stamp = time.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = self.z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)."""
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
    node = ChassisController3D()
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

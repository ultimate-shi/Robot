"""
Obstacle Avoidance Node - Filters velocity commands for safe traversal.

Sits between teleop (/cmd_vel_raw) and chassis controller (/cmd_vel).
Uses ultrasonic sensor data and terrain status to prevent collisions.

Logic:
- Front wall: ultrasonic < stop_distance → stop
- Front approach: ultrasonic < warn_distance → decelerate
- Side wall: limit turning toward wall
- Terrain blocked (step/dropoff/slope): stop forward motion
- Terrain slip: reduce speed proportionally
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter("front_stop_distance", 0.15)
        self.declare_parameter("front_warn_distance", 0.40)
        self.declare_parameter("side_stop_distance", 0.10)
        self.declare_parameter("side_warn_distance", 0.25)
        self.declare_parameter("terrain_traversability_min", 0.3)
        self.declare_parameter("update_rate", 20.0)

        self.front_stop = self.get_parameter("front_stop_distance").value
        self.front_warn = self.get_parameter("front_warn_distance").value
        self.side_stop = self.get_parameter("side_stop_distance").value
        self.side_warn = self.get_parameter("side_warn_distance").value
        self.traversability_min = self.get_parameter("terrain_traversability_min").value
        update_rate = self.get_parameter("update_rate").value

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_vel_raw_callback, 10)
        self.create_subscription(String, '/terrain_status', self.terrain_status_callback, 10)

        # Subscribe to 8 ultrasonic sensors
        self.ultrasonic_data = {}
        ultrasonic_topics = [
            '/ultrasonic/front_fl', '/ultrasonic/front_fr',
            '/ultrasonic/front_rl', '/ultrasonic/front_rr',
            '/ultrasonic/side_fl', '/ultrasonic/side_fr',
            '/ultrasonic/side_rl', '/ultrasonic/side_rr',
        ]
        for topic in ultrasonic_topics:
            self.create_subscription(
                Range, topic,
                lambda msg, t=topic: self.ultrasonic_callback(msg, t),
                10
            )
            self.ultrasonic_data[topic] = float('inf')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.warning_pub = self.create_publisher(String, '/obstacle_warning', 10)

        # Timer
        period = 1.0 / update_rate
        self.create_timer(period, self.timer_callback)

        # State
        self.latest_cmd_vel_raw = Twist()
        self.terrain_status = {}

        self.get_logger().info(
            f"ObstacleAvoidance started: front_stop={self.front_stop}m, "
            f"front_warn={self.front_warn}m, side_stop={self.side_stop}m"
        )

    def cmd_vel_raw_callback(self, msg: Twist):
        """Cache raw velocity command from teleop."""
        self.latest_cmd_vel_raw = msg

    def terrain_status_callback(self, msg: String):
        """Parse terrain status JSON."""
        try:
            self.terrain_status = json.loads(msg.data)
        except (json.JSONDecodeError, Exception):
            self.terrain_status = {}

    def ultrasonic_callback(self, msg: Range, topic: str):
        """Cache ultrasonic range reading."""
        if msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.ultrasonic_data[topic] = msg.range
        else:
            self.ultrasonic_data[topic] = msg.max_range

    def timer_callback(self):
        """Main avoidance logic - filter cmd_vel_raw and publish safe cmd_vel."""
        cmd = Twist()
        cmd.linear.x = self.latest_cmd_vel_raw.linear.x
        cmd.linear.y = self.latest_cmd_vel_raw.linear.y
        cmd.angular.z = self.latest_cmd_vel_raw.angular.z

        warnings = []

        # === Front obstacle check (only when moving forward) ===
        if cmd.linear.x > 0.001:
            front_min = self._get_front_min_distance()

            if front_min < self.front_stop:
                cmd.linear.x = 0.0
                warnings.append(f"FRONT_WALL:{front_min:.2f}m")
            elif front_min < self.front_warn:
                # Linear deceleration
                scale = (front_min - self.front_stop) / (self.front_warn - self.front_stop)
                scale = max(0.0, min(1.0, scale))
                cmd.linear.x *= scale
                warnings.append(f"FRONT_APPROACH:{front_min:.2f}m")

        # === Rear obstacle check (only when moving backward) ===
        if cmd.linear.x < -0.001:
            rear_min = self._get_rear_min_distance()

            if rear_min < self.front_stop:
                cmd.linear.x = 0.0
                warnings.append(f"REAR_WALL:{rear_min:.2f}m")
            elif rear_min < self.front_warn:
                scale = (rear_min - self.front_stop) / (self.front_warn - self.front_stop)
                scale = max(0.0, min(1.0, scale))
                cmd.linear.x *= scale

        # === Side obstacle check (limit turning) ===
        left_min = self._get_left_min_distance()
        right_min = self._get_right_min_distance()

        # Positive angular.z = turn left (CCW in ROS convention)
        if cmd.angular.z > 0.001 and left_min < self.side_stop:
            cmd.angular.z = 0.0
            warnings.append(f"LEFT_WALL:{left_min:.2f}m")
        elif cmd.angular.z > 0.001 and left_min < self.side_warn:
            scale = (left_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.angular.z *= max(0.0, min(1.0, scale))

        # Negative angular.z = turn right (CW)
        if cmd.angular.z < -0.001 and right_min < self.side_stop:
            cmd.angular.z = 0.0
            warnings.append(f"RIGHT_WALL:{right_min:.2f}m")
        elif cmd.angular.z < -0.001 and right_min < self.side_warn:
            scale = (right_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.angular.z *= max(0.0, min(1.0, scale))

        # === Terrain constraint check ===
        if self.terrain_status:
            # Step blocked
            if self.terrain_status.get("step_blocked", False) and cmd.linear.x > 0:
                cmd.linear.x = 0.0
                warnings.append("STEP_BLOCKED")

            # Drop-off blocked
            if self.terrain_status.get("dropoff_blocked", False) and cmd.linear.x > 0:
                cmd.linear.x = 0.0
                warnings.append("DROPOFF_BLOCKED")

            # General blockage
            if self.terrain_status.get("is_blocked", False) and cmd.linear.x > 0:
                reason = self.terrain_status.get("block_reason", "unknown")
                if reason == "slope":
                    cmd.linear.x = 0.0
                    warnings.append("SLOPE_BLOCKED")

            # Slip factor reduces speed
            slip = self.terrain_status.get("slip_factor", 1.0)
            if slip < 1.0:
                cmd.linear.x *= slip
                if slip < 0.5:
                    warnings.append(f"SLIPPING:{slip:.2f}")

        # Publish safe velocity
        self.cmd_vel_pub.publish(cmd)

        # Publish warnings
        if warnings:
            warn_msg = String()
            warn_msg.data = "|".join(warnings)
            self.warning_pub.publish(warn_msg)

    def _get_front_min_distance(self) -> float:
        """Get minimum distance from front-facing sensors."""
        front_topics = ['/ultrasonic/front_fl', '/ultrasonic/front_fr']
        distances = [self.ultrasonic_data.get(t, float('inf')) for t in front_topics]
        return min(distances) if distances else float('inf')

    def _get_rear_min_distance(self) -> float:
        """Get minimum distance from rear-facing sensors."""
        rear_topics = ['/ultrasonic/front_rl', '/ultrasonic/front_rr']
        distances = [self.ultrasonic_data.get(t, float('inf')) for t in rear_topics]
        return min(distances) if distances else float('inf')

    def _get_left_min_distance(self) -> float:
        """Get minimum distance from left-side sensors."""
        left_topics = ['/ultrasonic/side_fl', '/ultrasonic/side_rl']
        distances = [self.ultrasonic_data.get(t, float('inf')) for t in left_topics]
        return min(distances) if distances else float('inf')

    def _get_right_min_distance(self) -> float:
        """Get minimum distance from right-side sensors."""
        right_topics = ['/ultrasonic/side_fr', '/ultrasonic/side_rr']
        distances = [self.ultrasonic_data.get(t, float('inf')) for t in right_topics]
        return min(distances) if distances else float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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

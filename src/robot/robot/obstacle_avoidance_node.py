"""
robot.launch 使用说明：
本节点由 robot.launch.py 以 executable='obstacle_avoidance' 启动。
它是底盘控制前的安全过滤层，不负责路径规划，只负责把危险速度降速或置零。

输入：
- /cmd_vel_raw：在 launch 中被 remap 到 /cmd_vel，因此 Foxglove、Nav2 或命令行发布的 /cmd_vel 都会进入这里。
- /ultrasonic/front_fl、/ultrasonic/front_fr：前向避障。
- /ultrasonic/front_rl、/ultrasonic/front_rr：后向避障。
- /ultrasonic/side_fl、/ultrasonic/side_rl：左侧避障。
- /ultrasonic/side_fr、/ultrasonic/side_rr：右侧避障。
- /terrain_status：chassis_controller_node 发布的地形阻挡/打滑状态。

输出：
- /cmd_vel：在 launch 中被 remap 到 /cmd_vel_safe，底盘控制器实际消费这个安全速度。
- /obstacle_warning：发布 FRONT_WALL、LEFT_SIDE_WALL 等告警字符串，便于 Foxglove 调试。

避障策略：
- 不根据 crab/four_ws/ackermann 模式切换；只看 cmd_vel 中是否有 x/y/z 速度分量。
- linear.x 负责前后，linear.y 负责左右，angular.z 负责朝墙方向转向限制。
- cmd_vel_timeout 防止停止发布 /cmd_vel 后继续沿用最后一条非零速度。

为什么不能删除：
这是最后一层安全保护。即使 Nav2 local_costmap 正常工作，也需要它在近距离时拦截危险速度。
"""

import json

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
        self.declare_parameter("cmd_vel_timeout", 0.3)

        self.front_stop = self.get_parameter("front_stop_distance").value
        self.front_warn = self.get_parameter("front_warn_distance").value
        self.side_stop = self.get_parameter("side_stop_distance").value
        self.side_warn = self.get_parameter("side_warn_distance").value
        self.traversability_min = self.get_parameter("terrain_traversability_min").value
        update_rate = self.get_parameter("update_rate").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value

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
        self.last_cmd_vel_raw_time = self.get_clock().now()
        self.terrain_status = {}

        self.get_logger().info(
            f"ObstacleAvoidance started: front_stop={self.front_stop}m, "
            f"front_warn={self.front_warn}m, side_stop={self.side_stop}m"
        )

    def cmd_vel_raw_callback(self, msg: Twist):
        """Cache raw velocity command from teleop."""
        self.latest_cmd_vel_raw = msg
        self.last_cmd_vel_raw_time = self.get_clock().now()

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
        elapsed = (self.get_clock().now() - self.last_cmd_vel_raw_time).nanoseconds * 1e-9
        if elapsed <= self.cmd_vel_timeout:
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

        # === Side obstacle check (limit lateral motion and turning) ===
        left_min = self._get_left_min_distance()
        right_min = self._get_right_min_distance()

        # Positive linear.y = move left in ROS base_link convention.
        if cmd.linear.y > 0.001 and left_min < self.side_stop:
            cmd.linear.y = 0.0
            warnings.append(f"LEFT_SIDE_WALL:{left_min:.2f}m")
        elif cmd.linear.y > 0.001 and left_min < self.side_warn:
            scale = (left_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.linear.y *= max(0.0, min(1.0, scale))
            warnings.append(f"LEFT_SIDE_APPROACH:{left_min:.2f}m")

        # Negative linear.y = move right.
        if cmd.linear.y < -0.001 and right_min < self.side_stop:
            cmd.linear.y = 0.0
            warnings.append(f"RIGHT_SIDE_WALL:{right_min:.2f}m")
        elif cmd.linear.y < -0.001 and right_min < self.side_warn:
            scale = (right_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.linear.y *= max(0.0, min(1.0, scale))
            warnings.append(f"RIGHT_SIDE_APPROACH:{right_min:.2f}m")

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

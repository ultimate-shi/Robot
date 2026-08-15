"""
使用方法：由 robot_control/safety.launch.py 启动，过滤底盘最终速度。
本节点由 robot.launch.py 以 executable='obstacle_avoidance' 启动。
它是底盘控制前的最终安全过滤层，不负责路径规划，只负责把危险速度降速或置零。

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
- /obstacle_avoidance/status：发布具体触发的超声波、距离和安全速度，便于排查。

避障策略：
- 不根据 crab/four_ws/ackermann 模式切换；只看 cmd_vel 中是否有 x/y/z 速度分量。
- linear.x 负责前后，linear.y 负责左右，angular.z 负责朝墙方向转向限制。
- cmd_vel_timeout 防止停止发布 /cmd_vel 后继续沿用最后一条非零速度。
- range_timeout 防止超声波数据延迟或中断时继续沿用旧距离。
- 前方持续被挡且后方安全时，短暂低速后退，避免小车一直顶在障碍前。

为什么不能删除：
这是最后一层安全保护。即使 Nav2 local_costmap 正常工作，也需要它在近距离时拦截危险速度。
"""

import json
import math

from action_msgs.msg import GoalStatus, GoalStatusArray
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
try:
    from rclpy.qos import qos_profile_action_status
except ImportError:
    from rclpy.qos import qos_profile_action_status_default as qos_profile_action_status

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String
from robot_interfaces.msg import TerrainState


SENSOR_LABELS = {
    '/ultrasonic/front_fl': 'front_fl',
    '/ultrasonic/front_fr': 'front_fr',
    '/ultrasonic/front_rl': 'front_rl',
    '/ultrasonic/front_rr': 'front_rr',
    '/ultrasonic/side_fl': 'side_fl',
    '/ultrasonic/side_fr': 'side_fr',
    '/ultrasonic/side_rl': 'side_rl',
    '/ultrasonic/side_rr': 'side_rr',
}


class ObstacleAvoidanceNode(Node):
    ACTIVE_GOAL_STATUSES = {
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    }
    ESCAPE_GOAL_STATUSES = ACTIVE_GOAL_STATUSES | {GoalStatus.STATUS_ABORTED}

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Parameters
        self.declare_parameter("front_stop_distance", 0.10)
        self.declare_parameter("front_warn_distance", 0.20)
        self.declare_parameter("side_stop_distance", 0.10)
        self.declare_parameter("side_warn_distance", 0.20)
        self.declare_parameter("terrain_traversability_min", 0.3)
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("cmd_vel_timeout", 0.3)
        self.declare_parameter("range_timeout", 0.15)
        self.declare_parameter("require_valid_ranges", False)
        self.declare_parameter("obstacle_log_period", 0.5)
        self.declare_parameter("escape_reverse_enabled", True)
        self.declare_parameter("escape_trigger_time", 1.0)
        self.declare_parameter("escape_reverse_duration", 0.8)
        self.declare_parameter("escape_reverse_speed", -0.06)
        self.declare_parameter("escape_cooldown", 1.5)
        self.declare_parameter("rear_escape_clearance", 0.25)
        self.declare_parameter("recent_forward_timeout", 3.0)
        self.declare_parameter("nav_goal_active_timeout", 3.0)
        self.declare_parameter("nav_goal_escape_timeout", 30.0)

        self.front_stop = self.get_parameter("front_stop_distance").value
        self.front_warn = self.get_parameter("front_warn_distance").value
        self.side_stop = self.get_parameter("side_stop_distance").value
        self.side_warn = self.get_parameter("side_warn_distance").value
        self.traversability_min = self.get_parameter("terrain_traversability_min").value
        update_rate = self.get_parameter("update_rate").value
        self.cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self.range_timeout = self.get_parameter("range_timeout").value
        self.require_valid_ranges = bool(
            self.get_parameter("require_valid_ranges").value)
        self.obstacle_log_period = self.get_parameter("obstacle_log_period").value
        self.escape_reverse_enabled = bool(
            self.get_parameter("escape_reverse_enabled").value
        )
        self.escape_trigger_time = float(self.get_parameter("escape_trigger_time").value)
        self.escape_reverse_duration = float(
            self.get_parameter("escape_reverse_duration").value
        )
        self.escape_reverse_speed = float(self.get_parameter("escape_reverse_speed").value)
        self.escape_cooldown = float(self.get_parameter("escape_cooldown").value)
        self.rear_escape_clearance = float(
            self.get_parameter("rear_escape_clearance").value
        )
        self.recent_forward_timeout = float(
            self.get_parameter("recent_forward_timeout").value
        )
        self.nav_goal_active_timeout = float(
            self.get_parameter("nav_goal_active_timeout").value
        )
        self.nav_goal_escape_timeout = float(
            self.get_parameter("nav_goal_escape_timeout").value
        )

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_vel_raw_callback, 10)
        self.create_subscription(
            TerrainState, '/perception/terrain_state',
            self.terrain_status_callback, 10)
        self.nav_goal_active = {}
        self.nav_goal_status_time = {}
        self.nav_goal_last_active_time = {}
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            lambda msg: self.nav_goal_status_callback(msg, 'navigate_to_pose'),
            qos_profile_action_status
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_through_poses/_action/status',
            lambda msg: self.nav_goal_status_callback(msg, 'navigate_through_poses'),
            qos_profile_action_status
        )

        # Subscribe to 8 ultrasonic sensors
        self.ultrasonic_data = {}
        self.ultrasonic_stamp = {}
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
            self.ultrasonic_stamp[topic] = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.warning_pub = self.create_publisher(String, '/obstacle_warning', 10)
        self.status_pub = self.create_publisher(String, '/obstacle_avoidance/status', 10)

        # Timer
        period = 1.0 / update_rate
        self.create_timer(period, self.timer_callback)

        # State
        self.latest_cmd_vel_raw = Twist()
        self.last_cmd_vel_raw_time = self.get_clock().now()
        self.terrain_status = {}
        self.last_obstacle_log_time = self.get_clock().now()
        self.front_blocked_since = None
        self.escape_reverse_until = None
        self.escape_cooldown_until = None
        self.last_forward_cmd_time = None

        self.get_logger().info(
            f"ObstacleAvoidance started: front_stop={self.front_stop}m, "
            f"front_warn={self.front_warn}m, side_stop={self.side_stop}m, "
            f"range_timeout={self.range_timeout}s, "
            f"obstacle_log_period={self.obstacle_log_period}s, "
            f"escape_reverse_enabled={self.escape_reverse_enabled}, "
            f"nav_goal_active_timeout={self.nav_goal_active_timeout}s, "
            f"nav_goal_escape_timeout={self.nav_goal_escape_timeout}s"
        )

    def cmd_vel_raw_callback(self, msg: Twist):
        """Cache raw velocity command from teleop."""
        self.latest_cmd_vel_raw = msg
        now = self.get_clock().now()
        self.last_cmd_vel_raw_time = now
        if msg.linear.x > 0.001:
            self.last_forward_cmd_time = now

    def terrain_status_callback(self, msg: TerrainState):
        """缓存类型化地形状态，安全层不再解析跨包 JSON。"""
        self.terrain_status = {
            'is_blocked': msg.is_blocked,
            'block_reason': msg.block_reason,
            'traversability': msg.traversability,
            'slip_factor': msg.slip_factor,
            'body_z': msg.body_z,
            'roll': msg.roll,
            'pitch': msg.pitch,
            'step_blocked': msg.step_blocked,
            'dropoff_blocked': msg.dropoff_blocked,
        }
        self.last_obstacle_log_time = self.get_clock().now()

    def nav_goal_status_callback(self, msg: GoalStatusArray, action_name: str):
        """缓存 Nav2 目标是否仍活跃，只用于自动后退脱困触发。"""
        if not msg.status_list:
            return
        active = any(
            status.status in self.ACTIVE_GOAL_STATUSES
            for status in msg.status_list
        )
        now = self.get_clock().now()
        self.nav_goal_active[action_name] = active
        self.nav_goal_status_time[action_name] = now
        escape_candidate = any(
            status.status in self.ESCAPE_GOAL_STATUSES
            for status in msg.status_list
        )
        if escape_candidate:
            self.nav_goal_last_active_time[action_name] = now

    def ultrasonic_callback(self, msg: Range, topic: str):
        """缓存超声波距离和接收时间，避免主循环使用过期测距。"""
        if (math.isfinite(msg.range)
                and msg.range >= msg.min_range
                and msg.range <= msg.max_range):
            self.ultrasonic_data[topic] = msg.range
            self.ultrasonic_stamp[topic] = self.get_clock().now()
        else:
            self.ultrasonic_data[topic] = float('inf')
            self.ultrasonic_stamp[topic] = None

    def timer_callback(self):
        """Main avoidance logic - filter cmd_vel_raw and publish safe cmd_vel."""
        cmd = Twist()
        elapsed = (self.get_clock().now() - self.last_cmd_vel_raw_time).nanoseconds * 1e-9
        if elapsed <= self.cmd_vel_timeout:
            cmd.linear.x = self.latest_cmd_vel_raw.linear.x
            cmd.linear.y = self.latest_cmd_vel_raw.linear.y
            cmd.angular.z = self.latest_cmd_vel_raw.angular.z

        warnings = []

        front_min = self._get_front_min_distance()

        # === Front obstacle check (only when moving forward) ===
        if cmd.linear.x > 0.001:
            if self.require_valid_ranges and not self._has_valid_readings([
                    '/ultrasonic/front_fl', '/ultrasonic/front_fr']):
                cmd.linear.x = 0.0
                warnings.append("FRONT_RANGE_STALE")
            elif front_min < self.front_stop:
                cmd.linear.x = 0.0
                warnings.append(f"FRONT_WALL:{front_min:.2f}m")
            elif front_min < self.front_warn:
                # Linear deceleration
                scale = (front_min - self.front_stop) / (self.front_warn - self.front_stop)
                scale = max(0.0, min(1.0, scale))
                cmd.linear.x *= scale
                warnings.append(f"FRONT_APPROACH:{front_min:.2f}m")

        escape_active = self._update_escape_reverse(cmd, warnings, front_min)
        if escape_active:
            detected_ultrasonic = self._detected_ultrasonic()
            self._log_detected_ultrasonic(detected_ultrasonic, warnings)
            self.cmd_vel_pub.publish(cmd)
            self._publish_status(cmd, warnings, detected_ultrasonic, escape_active)
            warn_msg = String()
            warn_msg.data = "|".join(warnings)
            self.warning_pub.publish(warn_msg)
            return

        # === Rear obstacle check (only when moving backward) ===
        if cmd.linear.x < -0.001:
            rear_min = self._get_rear_min_distance()

            if self.require_valid_ranges and not self._has_valid_readings([
                    '/ultrasonic/front_rl', '/ultrasonic/front_rr']):
                cmd.linear.x = 0.0
                warnings.append("REAR_RANGE_STALE")
            elif rear_min < self.front_stop:
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
        left_topics = ['/ultrasonic/side_fl', '/ultrasonic/side_rl']
        right_topics = ['/ultrasonic/side_fr', '/ultrasonic/side_rr']
        if (cmd.linear.y > 0.001 and self.require_valid_ranges
                and not self._has_valid_readings(left_topics)):
            cmd.linear.y = 0.0
            warnings.append("LEFT_RANGE_STALE")
        elif cmd.linear.y > 0.001 and left_min < self.side_stop:
            cmd.linear.y = 0.0
            warnings.append(f"LEFT_SIDE_WALL:{left_min:.2f}m")
        elif cmd.linear.y > 0.001 and left_min < self.side_warn:
            scale = (left_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.linear.y *= max(0.0, min(1.0, scale))
            warnings.append(f"LEFT_SIDE_APPROACH:{left_min:.2f}m")

        # Negative linear.y = move right.
        if (cmd.linear.y < -0.001 and self.require_valid_ranges
                and not self._has_valid_readings(right_topics)):
            cmd.linear.y = 0.0
            warnings.append("RIGHT_RANGE_STALE")
        elif cmd.linear.y < -0.001 and right_min < self.side_stop:
            cmd.linear.y = 0.0
            warnings.append(f"RIGHT_SIDE_WALL:{right_min:.2f}m")
        elif cmd.linear.y < -0.001 and right_min < self.side_warn:
            scale = (right_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.linear.y *= max(0.0, min(1.0, scale))
            warnings.append(f"RIGHT_SIDE_APPROACH:{right_min:.2f}m")

        # Positive angular.z = turn left (CCW in ROS convention)
        if (cmd.angular.z > 0.001 and self.require_valid_ranges
                and not self._has_valid_readings(left_topics)):
            cmd.angular.z = 0.0
            warnings.append("LEFT_TURN_RANGE_STALE")
        elif cmd.angular.z > 0.001 and left_min < self.side_stop:
            cmd.angular.z = 0.0
            warnings.append(f"LEFT_WALL:{left_min:.2f}m")
        elif cmd.angular.z > 0.001 and left_min < self.side_warn:
            scale = (left_min - self.side_stop) / (self.side_warn - self.side_stop)
            cmd.angular.z *= max(0.0, min(1.0, scale))

        # Negative angular.z = turn right (CW)
        if (cmd.angular.z < -0.001 and self.require_valid_ranges
                and not self._has_valid_readings(right_topics)):
            cmd.angular.z = 0.0
            warnings.append("RIGHT_TURN_RANGE_STALE")
        elif cmd.angular.z < -0.001 and right_min < self.side_stop:
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

        detected_ultrasonic = self._detected_ultrasonic()
        self._log_detected_ultrasonic(detected_ultrasonic, warnings)

        # Publish safe velocity
        self.cmd_vel_pub.publish(cmd)
        self._publish_status(cmd, warnings, detected_ultrasonic, escape_active)

        # Publish warnings
        if warnings:
            warn_msg = String()
            warn_msg.data = "|".join(warnings)
            self.warning_pub.publish(warn_msg)

    def _update_escape_reverse(self, cmd: Twist, warnings, front_min: float) -> bool:
        """前方持续被挡时短暂后退，给 Nav2 重新规划留出空间。"""
        now = self.get_clock().now()
        if not self.escape_reverse_enabled:
            self._reset_escape_state()
            return False

        if self._time_active(self.escape_reverse_until, now):
            cmd.linear.x = self.escape_reverse_speed
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            warnings.append("ESCAPE_REVERSE")
            return True

        if self.escape_reverse_until is not None:
            self.escape_reverse_until = None
            self.escape_cooldown_until = now + Duration(seconds=self.escape_cooldown)

        front_blocking = front_min < self.front_stop and (
            self._recent_forward_command(now) or self._recent_active_nav_goal(now)
        )
        if not front_blocking:
            self.front_blocked_since = None
            return False

        if self._time_active(self.escape_cooldown_until, now):
            return False

        rear_min = self._get_rear_min_distance()
        if rear_min < self.rear_escape_clearance:
            warnings.append(f"REAR_NOT_CLEAR:{rear_min:.2f}m")
            self.front_blocked_since = None
            return False

        if self.front_blocked_since is None:
            self.front_blocked_since = now
            return False

        blocked_time = (now - self.front_blocked_since).nanoseconds * 1e-9
        if blocked_time < self.escape_trigger_time:
            return False

        self.escape_reverse_until = now + Duration(seconds=self.escape_reverse_duration)
        self.front_blocked_since = None
        cmd.linear.x = self.escape_reverse_speed
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        warnings.append("ESCAPE_REVERSE_START")
        self.get_logger().warning(
            "Front obstacle persisted; backing up briefly "
            f"at {self.escape_reverse_speed:.2f}m/s for "
            f"{self.escape_reverse_duration:.2f}s"
        )
        return True

    def _recent_forward_command(self, now) -> bool:
        """最近收到过前进命令时，才允许自动后退脱困。"""
        if self.last_forward_cmd_time is None:
            return False
        age = (now - self.last_forward_cmd_time).nanoseconds * 1e-9
        return age <= self.recent_forward_timeout

    def _recent_active_nav_goal(self, now) -> bool:
        """Nav2 目标刚活跃或失败过时，也允许近距离安全层主动后退脱困。"""
        for action_name, active in self.nav_goal_active.items():
            stamp = self.nav_goal_status_time.get(action_name)
            if active and stamp is not None:
                age = (now - stamp).nanoseconds * 1e-9
                if age <= self.nav_goal_active_timeout:
                    return True

        return self._recent_nav_goal_escape_candidate(now)

    def _recent_nav_goal_escape_candidate(self, now) -> bool:
        """Nav2 失败后超声波可能滞后更新，因此脱困窗口要比 action 活跃窗口更长。"""
        for stamp in self.nav_goal_last_active_time.values():
            age = (now - stamp).nanoseconds * 1e-9
            if age <= self.nav_goal_escape_timeout:
                return True
        return False

    @staticmethod
    def _time_active(deadline, now) -> bool:
        """判断当前时间是否仍在某个截止时间之前。"""
        return deadline is not None and (deadline - now).nanoseconds > 0

    def _reset_escape_state(self):
        """关闭脱困后退时清理内部状态。"""
        self.front_blocked_since = None
        self.escape_reverse_until = None
        self.escape_cooldown_until = None

    def _valid_readings(self, topics):
        """返回未超时的测距，包含传感器名称和距离。"""
        now = self.get_clock().now()
        readings = []
        for topic in topics:
            stamp = self.ultrasonic_stamp.get(topic)
            if stamp is None:
                continue
            age = (now - stamp).nanoseconds * 1e-9
            if age <= self.range_timeout:
                readings.append((topic, self.ultrasonic_data.get(topic, float('inf'))))
        return readings

    def _valid_distances(self, topics):
        """返回未超时的测距；过期数据不参与避障判断。"""
        return [distance for _, distance in self._valid_readings(topics)]

    def _has_valid_readings(self, topics):
        """安全模式要求运动方向至少有一路未超时测距。"""
        return bool(self._valid_readings(topics))

    def _get_front_min_distance(self) -> float:
        """Get minimum distance from front-facing sensors."""
        distances = self._valid_distances(['/ultrasonic/front_fl', '/ultrasonic/front_fr'])
        return min(distances) if distances else float('inf')

    def _get_rear_min_distance(self) -> float:
        """Get minimum distance from rear-facing sensors."""
        distances = self._valid_distances(['/ultrasonic/front_rl', '/ultrasonic/front_rr'])
        return min(distances) if distances else float('inf')

    def _get_left_min_distance(self) -> float:
        """Get minimum distance from left-side sensors."""
        distances = self._valid_distances(['/ultrasonic/side_fl', '/ultrasonic/side_rl'])
        return min(distances) if distances else float('inf')

    def _get_right_min_distance(self) -> float:
        """Get minimum distance from right-side sensors."""
        distances = self._valid_distances(['/ultrasonic/side_fr', '/ultrasonic/side_rr'])
        return min(distances) if distances else float('inf')

    def _detected_ultrasonic(self):
        """返回所有低于告警距离的超声波读数，用于日志和状态输出。"""
        groups = [
            ('front', self.front_warn, ['/ultrasonic/front_fl', '/ultrasonic/front_fr']),
            ('rear', self.front_warn, ['/ultrasonic/front_rl', '/ultrasonic/front_rr']),
            ('left', self.side_warn, ['/ultrasonic/side_fl', '/ultrasonic/side_rl']),
            ('right', self.side_warn, ['/ultrasonic/side_fr', '/ultrasonic/side_rr']),
        ]
        detected = []
        for direction, threshold, topics in groups:
            for topic, distance in self._valid_readings(topics):
                if distance < threshold:
                    detected.append({
                        'sensor': SENSOR_LABELS.get(topic, topic),
                        'topic': topic,
                        'direction': direction,
                        'distance': distance,
                        'threshold': threshold,
                    })
        return detected

    def _log_detected_ultrasonic(self, detected_ultrasonic, warnings):
        """检测到障碍物时在后台打印具体超声波和距离，日志节流避免刷屏。"""
        if not detected_ultrasonic:
            return
        now = self.get_clock().now()
        elapsed = (now - self.last_obstacle_log_time).nanoseconds * 1e-9
        if elapsed < self.obstacle_log_period:
            return
        self.last_obstacle_log_time = now
        readings = ', '.join(
            f"{item['sensor']}={item['distance']:.2f}m"
            for item in detected_ultrasonic
        )
        warn_text = '|'.join(warnings) if warnings else 'DETECTED_ONLY'
        self.get_logger().warning(f"Ultrasonic obstacle detected: {readings}; warnings={warn_text}")

    def _publish_status(self, cmd: Twist, warnings, detected_ultrasonic, escape_active=False):
        """发布避障层状态，便于确认超声波是否及时影响安全速度。"""
        front_min = self._get_front_min_distance()
        rear_min = self._get_rear_min_distance()
        left_min = self._get_left_min_distance()
        right_min = self._get_right_min_distance()
        status = {
            'front_min': self._json_distance(front_min),
            'rear_min': self._json_distance(rear_min),
            'left_min': self._json_distance(left_min),
            'right_min': self._json_distance(right_min),
            'cmd_safe': {
                'linear_x': cmd.linear.x,
                'linear_y': cmd.linear.y,
                'angular_z': cmd.angular.z,
            },
            'warnings': warnings,
            'escape_active': escape_active,
            'nav_goal_active': self._recent_active_nav_goal(self.get_clock().now()),
            'nav_goal_escape_recent': self._recent_nav_goal_escape_candidate(
                self.get_clock().now()
            ),
            'detected_ultrasonic': [
                {
                    'sensor': item['sensor'],
                    'topic': item['topic'],
                    'direction': item['direction'],
                    'distance': item['distance'],
                    'threshold': item['threshold'],
                }
                for item in detected_ultrasonic
            ],
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    @staticmethod
    def _json_distance(distance: float):
        """把无有效测距转换成 JSON null，避免发布非标准 Infinity。"""
        if distance == float('inf'):
            return None
        return distance


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

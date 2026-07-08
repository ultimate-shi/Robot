"""
nav_controller_node 使用说明：
本节点位于 Nav2 和底盘安全层之间，负责速度仲裁、卡住检测、回退请求和 Nav2 目标取消。

输入：
- /cmd_vel_nav：Nav2 collision_monitor 的输出速度。
- /cmd_vel_reverse：reverse_node 回退速度。
- /cmd_vel_safe：点云模式下 obstacle_avoidance 输出的安全速度，用于判断安全层是否长期拦停。
- /odom：检测 Nav2 发速度时车体是否真的移动。
- /obstacle_warning：安全层告警。
- /reverse_control/status：reverse_node 状态。

输出：
- /cmd_vel：正常转发 Nav2 速度；回退期间转发 /cmd_vel_reverse。
- /reverse_control/request：请求 reverse_node START/CANCEL。
- /nav_controller/status：JSON 状态，便于 Foxglove 和命令行排查。

服务：
- /nav_controller/start_reverse：手动请求回退。
- /nav_controller/cancel_reverse：取消回退。
- /nav_controller/reset_attempts：重置当前目标的恢复次数。
"""

import json
import math
from collections import deque

import rclpy
from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def twist_norm(msg: Twist) -> float:
    """用线速度和角速度合成一个简单运动强度。"""
    return abs(msg.linear.x) + abs(msg.linear.y) + 0.25 * abs(msg.angular.z)


class NavControllerNode(Node):
    """Nav2 速度仲裁和卡住恢复控制。"""

    def __init__(self):
        super().__init__('nav_controller_node')

        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('stuck_timeout', 2.5)
        self.declare_parameter('stuck_distance', 0.03)
        self.declare_parameter('nav_cmd_active_threshold', 0.02)
        self.declare_parameter('safe_cmd_zero_threshold', 0.01)
        self.declare_parameter('safe_block_timeout', 1.5)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('reverse_distance', 0.8)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('goal_idle_reset_time', 3.0)
        self.declare_parameter('reverse_retry_cooldown', 3.0)

        self.update_rate = float(self.get_parameter('update_rate').value)
        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.stuck_distance = float(self.get_parameter('stuck_distance').value)
        self.nav_cmd_active_threshold = float(
            self.get_parameter('nav_cmd_active_threshold').value
        )
        self.safe_cmd_zero_threshold = float(
            self.get_parameter('safe_cmd_zero_threshold').value
        )
        self.safe_block_timeout = float(self.get_parameter('safe_block_timeout').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.reverse_distance = float(self.get_parameter('reverse_distance').value)
        self.max_recovery_attempts = int(self.get_parameter('max_recovery_attempts').value)
        self.goal_idle_reset_time = float(self.get_parameter('goal_idle_reset_time').value)
        self.reverse_retry_cooldown = float(
            self.get_parameter('reverse_retry_cooldown').value
        )

        self.create_subscription(Twist, '/cmd_vel_nav', self.nav_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_reverse', self.reverse_cmd_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.safe_cmd_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.create_subscription(String, '/obstacle_warning', self.warning_callback, 10)
        self.create_subscription(String, '/reverse_control/status', self.reverse_status_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reverse_request_pub = self.create_publisher(String, '/reverse_control/request', 10)
        self.status_pub = self.create_publisher(String, '/nav_controller/status', 10)

        self.create_service(Trigger, '/nav_controller/start_reverse', self.start_reverse_service)
        self.create_service(Trigger, '/nav_controller/cancel_reverse', self.cancel_reverse_service)
        self.create_service(Trigger, '/nav_controller/reset_attempts', self.reset_attempts_service)

        self.cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )

        self.latest_nav_cmd = Twist()
        self.latest_reverse_cmd = Twist()
        self.latest_safe_cmd = Twist()
        now = self.get_clock().now()
        self.last_nav_cmd_time = now
        self.last_reverse_cmd_time = now
        self.last_safe_cmd_time = now
        self.last_warning_time = None
        self.last_nav_active_time = None
        self.nav_motion_start_time = None
        self.safe_block_start_time = None
        self.reverse_retry_block_until = None

        self.odom_window = deque()
        self.latest_odom_pose = None

        self.recovery_attempts = 0
        self.reverse_active = False
        self.cancelled_goal = False
        self.last_status_state = ''

        self.create_timer(1.0 / self.update_rate, self.control_loop)
        self.publish_status('IDLE', 'ready')
        self.get_logger().info(
            f"nav_controller_node started: max_recovery_attempts={self.max_recovery_attempts}"
        )

    def nav_cmd_callback(self, msg: Twist):
        """缓存 Nav2 输出速度，并在新一段导航运动开始时重置卡住检测窗口。"""
        self.latest_nav_cmd = msg
        self.last_nav_cmd_time = self.get_clock().now()
        if twist_norm(msg) >= self.nav_cmd_active_threshold:
            self.last_nav_active_time = self.last_nav_cmd_time
            if self.nav_motion_start_time is None:
                self.nav_motion_start_time = self.last_nav_cmd_time
                self.reset_stuck_window()
                self.safe_block_start_time = None
        else:
            self.nav_motion_start_time = None
            self.safe_block_start_time = None

    def reverse_cmd_callback(self, msg: Twist):
        """缓存 reverse_node 输出速度。"""
        self.latest_reverse_cmd = msg
        self.last_reverse_cmd_time = self.get_clock().now()

    def safe_cmd_callback(self, msg: Twist):
        """缓存安全速度，用于判断 obstacle_avoidance 是否长期拦停。"""
        self.latest_safe_cmd = msg
        self.last_safe_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        """维护最近一段 odom 位移窗口。"""
        pose = {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'time': self.get_clock().now(),
        }
        self.latest_odom_pose = pose
        self.odom_window.append(pose)
        self.trim_odom_window()

    def warning_callback(self, msg: String):
        """只要安全层持续发布告警，就认为附近存在障碍风险。"""
        if msg.data:
            self.last_warning_time = self.get_clock().now()

    def reverse_status_callback(self, msg: String):
        """根据 reverse_node 终态恢复 Nav2 速度或保持取消状态。"""
        try:
            status = json.loads(msg.data)
            state = str(status.get('state', '')).upper()
            reason = str(status.get('reason', ''))
        except (json.JSONDecodeError, TypeError):
            state = msg.data.strip().upper()
            reason = ''

        if state == 'RUNNING':
            self.reverse_active = True
            self.publish_status('REVERSING', reason)
        elif state in {'COMPLETED', 'FAILED', 'CANCELED'}:
            was_active = self.reverse_active
            self.reverse_active = False
            if state == 'FAILED':
                self.reverse_retry_block_until = (
                    self.get_clock().now(),
                    self.reverse_retry_cooldown
                )
            self.publish_zero()
            if was_active and not self.cancelled_goal:
                self.reset_stuck_window()
                self.publish_status(f'REVERSE_{state}', reason)

    def start_reverse_service(self, request, response):
        """手动触发一次原路回退。"""
        del request
        if self.cancelled_goal:
            self.cancelled_goal = False
        self.request_reverse('manual')
        response.success = True
        response.message = 'reverse requested'
        return response

    def cancel_reverse_service(self, request, response):
        """手动取消回退。"""
        del request
        self.reverse_request_pub.publish(String(data='CANCEL'))
        self.reverse_active = False
        self.publish_zero()
        self.publish_status('REVERSE_CANCEL_REQUESTED', 'manual')
        response.success = True
        response.message = 'reverse cancel requested'
        return response

    def reset_attempts_service(self, request, response):
        """手动清零恢复次数和取消标记。"""
        del request
        self.recovery_attempts = 0
        self.cancelled_goal = False
        self.reset_stuck_window()
        self.publish_status('ATTEMPTS_RESET', 'manual')
        response.success = True
        response.message = 'recovery attempts reset'
        return response

    def control_loop(self):
        """主循环：选择 Nav2 或回退速度，并检测卡住。"""
        now = self.get_clock().now()

        if self.reverse_active:
            if self.age(self.last_reverse_cmd_time) <= self.cmd_vel_timeout:
                self.cmd_pub.publish(self.latest_reverse_cmd)
            else:
                self.publish_zero()
            return

        if self.cancelled_goal:
            self.publish_zero()
            return

        nav_active = self.nav_command_active()
        if nav_active:
            self.cmd_pub.publish(self.latest_nav_cmd)
            if self.detect_stuck():
                self.handle_stuck()
            return

        self.publish_zero()
        self.nav_motion_start_time = None
        self.safe_block_start_time = None
        if self.last_nav_active_time is not None:
            idle_time = (now - self.last_nav_active_time).nanoseconds * 1e-9
            if idle_time > self.goal_idle_reset_time and self.recovery_attempts:
                self.recovery_attempts = 0
                self.cancelled_goal = False
                self.reset_stuck_window()
                self.publish_status('IDLE', 'attempts_reset_after_idle')

    def reverse_retry_blocked(self) -> bool:
        """回退失败后的短冷却，避免反复请求导致 /cmd_vel_reverse 连续刷零。"""
        if self.reverse_retry_block_until is None:
            return False
        start_time, duration = self.reverse_retry_block_until
        if self.age(start_time) < duration:
            return True
        self.reverse_retry_block_until = None
        return False

    def handle_stuck(self):
        """卡住时按次数决定回退或取消 Nav2 目标。"""
        if self.reverse_active or self.cancelled_goal:
            return
        if self.reverse_retry_blocked():
            return

        if self.recovery_attempts >= self.max_recovery_attempts:
            self.cancel_nav_goal()
            self.cancelled_goal = True
            self.publish_zero()
            self.publish_status('CANCEL_NAV_GOAL', 'max_recovery_attempts_exceeded')
            return

        self.recovery_attempts += 1
        self.request_reverse('stuck_detected')
        self.reset_stuck_window()

    def request_reverse(self, reason: str):
        """请求 reverse_node 开始回退。"""
        payload = {
            'command': 'START',
            'distance': self.reverse_distance,
            'reason': reason,
            'attempt': self.recovery_attempts,
        }
        self.reverse_request_pub.publish(String(data=json.dumps(payload)))
        self.reverse_active = True
        self.publish_zero()
        self.publish_status('REQUEST_REVERSE', reason)

    def cancel_nav_goal(self):
        """调用 Nav2 NavigateToPose action 的 cancel_goal 服务取消当前目标。"""
        if not self.cancel_client.service_is_ready():
            self.cancel_client.wait_for_service(timeout_sec=0.2)
        if not self.cancel_client.service_is_ready():
            self.publish_status('CANCEL_NAV_GOAL_FAILED', 'cancel_service_unavailable')
            return

        request = CancelGoal.Request()
        request.goal_info = GoalInfo()
        future = self.cancel_client.call_async(request)
        future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """记录取消服务返回结果。"""
        try:
            response = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.publish_status('CANCEL_NAV_GOAL_FAILED', str(exc))
            return
        self.publish_status('CANCEL_NAV_GOAL', f'return_code:{response.return_code}')

    def detect_stuck(self) -> bool:
        """检测 Nav2 有速度但 odom 没有足够位移，或安全层长期输出 0。"""
        if self.nav_motion_start_time is None:
            return False
        if self.age(self.nav_motion_start_time) < self.stuck_timeout:
            return False

        if len(self.odom_window) >= 2:
            oldest = self.odom_window[0]
            newest = self.odom_window[-1]
            span = (newest['time'] - oldest['time']).nanoseconds * 1e-9
            moved = math.hypot(newest['x'] - oldest['x'], newest['y'] - oldest['y'])
            if span >= self.stuck_timeout and moved < self.stuck_distance:
                return True

        if self.safe_layer_blocking():
            return True
        return False

    def safe_layer_blocking(self) -> bool:
        """点云模式下判断 /cmd_vel_safe 是否被安全层持续压到接近 0。"""
        blocking_now = (
            self.last_warning_time is not None
            and self.age(self.last_safe_cmd_time) <= self.safe_block_timeout
            and self.age(self.last_warning_time) <= self.safe_block_timeout
            and twist_norm(self.latest_safe_cmd) <= self.safe_cmd_zero_threshold
        )
        if not blocking_now:
            self.safe_block_start_time = None
            return False
        if self.safe_block_start_time is None:
            self.safe_block_start_time = self.get_clock().now()
            return False
        return self.age(self.safe_block_start_time) >= self.safe_block_timeout

    def nav_command_active(self) -> bool:
        """判断 Nav2 速度是否还有效且表示正在移动。"""
        if self.age(self.last_nav_cmd_time) > self.cmd_vel_timeout:
            return False
        return twist_norm(self.latest_nav_cmd) >= self.nav_cmd_active_threshold

    def trim_odom_window(self):
        """只保留 stuck_timeout 时间窗内的 odom 样本。"""
        if not self.odom_window:
            return
        newest = self.odom_window[-1]
        while len(self.odom_window) > 2:
            age = (newest['time'] - self.odom_window[0]['time']).nanoseconds * 1e-9
            if age <= self.stuck_timeout + 0.5:
                break
            self.odom_window.popleft()

    def reset_stuck_window(self):
        """回退或重置后重新开始累计卡住检测窗口。"""
        self.odom_window.clear()
        if self.latest_odom_pose is not None:
            self.odom_window.append(self.latest_odom_pose)
        self.nav_motion_start_time = self.get_clock().now()
        self.safe_block_start_time = None

    def publish_zero(self):
        """向底盘链路发布零速度。"""
        self.cmd_pub.publish(Twist())

    def publish_status(self, state: str, reason: str):
        """发布当前仲裁状态。"""
        payload = {
            'state': state,
            'reason': reason,
            'recovery_attempts': self.recovery_attempts,
            'max_recovery_attempts': self.max_recovery_attempts,
            'reverse_active': self.reverse_active,
            'cancelled_goal': self.cancelled_goal,
        }
        data = json.dumps(payload, ensure_ascii=False)
        if state == self.last_status_state and state in {'NAV_FORWARD', 'REVERSING'}:
            return
        self.last_status_state = state
        self.status_pub.publish(String(data=data))

    def age(self, stamp) -> float:
        """计算某个 rclpy 时间点距离现在的秒数。"""
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
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

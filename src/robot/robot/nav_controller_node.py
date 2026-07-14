"""
nav_controller_node 使用说明：
本节点位于 Nav2 velocity_smoother 和底盘安全层之间，只做目标状态监督和速度门控。
它不再执行自定义原地旋转、倒车恢复或取消 Nav2 goal，恢复行为交给 Nav2 自身处理。

输入：
- /cmd_vel_nav_smoothed：Nav2 velocity_smoother 输出的平滑速度。
- /navigate_to_pose/_action/status：单目标导航 action 状态。
- /navigate_through_poses/_action/status：多目标导航 action 状态。

输出：
- /cmd_vel_nav：输入速度新鲜时转发速度；目标结束、失败、取消或速度超时时发布零速。
- /nav_controller/status：JSON 状态，便于 Foxglove 和命令行排查。
"""

import json

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
try:
    from rclpy.qos import qos_profile_action_status
except ImportError:
    from rclpy.qos import qos_profile_action_status_default as qos_profile_action_status
from std_msgs.msg import String


def twist_norm(msg: Twist) -> float:
    """用线速度和角速度合成一个简单运动强度。"""
    return abs(msg.linear.x) + abs(msg.linear.y) + 0.25 * abs(msg.angular.z)


class NavControllerNode(Node):
    """Nav2 平滑速度的最后一道状态门控。"""

    ACTIVE_STATUSES = {
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    }
    TERMINAL_STATUSES = {
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    }

    def __init__(self):
        super().__init__('nav_controller_node')

        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('require_active_nav_goal', False)
        self.declare_parameter('nav_goal_status_timeout', 3.0)
        self.declare_parameter('nav_cmd_active_threshold', 0.02)

        update_rate = float(self.get_parameter('update_rate').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.require_active_nav_goal = bool(
            self.get_parameter('require_active_nav_goal').value
        )
        self.nav_goal_status_timeout = float(
            self.get_parameter('nav_goal_status_timeout').value
        )
        self.nav_cmd_active_threshold = float(
            self.get_parameter('nav_cmd_active_threshold').value
        )

        self.create_subscription(
            Twist,
            '/cmd_vel_nav_smoothed',
            self.nav_cmd_callback,
            10
        )
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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/nav_controller/status', 10)

        now = self.get_clock().now()
        self.latest_nav_cmd = Twist()
        self.last_nav_cmd_time = now
        self.goal_states = {
            'navigate_to_pose': {
                'active': False,
                'last_status': GoalStatus.STATUS_UNKNOWN,
                'status_time': None,
            },
            'navigate_through_poses': {
                'active': False,
                'last_status': GoalStatus.STATUS_UNKNOWN,
                'status_time': None,
            },
        }
        self.last_status_state = ''

        self.create_timer(1.0 / update_rate, self.control_loop)
        self.publish_status('IDLE', 'ready')
        self.get_logger().info(
            "nav_controller_node started: input=/cmd_vel_nav_smoothed, "
            "output=/cmd_vel_nav, recovery=nav2_only"
        )

    def nav_cmd_callback(self, msg: Twist):
        """缓存 Nav2 平滑速度。"""
        self.latest_nav_cmd = msg
        self.last_nav_cmd_time = self.get_clock().now()

    def nav_goal_status_callback(self, msg: GoalStatusArray, action_name: str):
        """记录 Nav2 action 状态，并在终态时切断遗留速度。"""
        state = self.goal_states[action_name]
        state['active'] = any(status.status in self.ACTIVE_STATUSES for status in msg.status_list)
        state['status_time'] = self.get_clock().now()

        # action status 数组可能短时间保留旧 goal 终态；只要存在活跃 goal，就优先放行。
        if state['active']:
            state['last_status'] = GoalStatus.STATUS_EXECUTING
            return

        terminal_statuses = [
            status.status for status in msg.status_list
            if status.status in self.TERMINAL_STATUSES
        ]
        if terminal_statuses:
            latest_terminal = terminal_statuses[-1]
            state['last_status'] = latest_terminal
            if latest_terminal == GoalStatus.STATUS_SUCCEEDED:
                self.publish_zero('GOAL_SUCCEEDED', action_name)
            elif latest_terminal == GoalStatus.STATUS_ABORTED:
                self.get_logger().error(f"{action_name} aborted by Nav2")
                self.publish_zero('GOAL_ABORTED', action_name)
            elif latest_terminal == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn(f"{action_name} canceled")
                self.publish_zero('GOAL_CANCELED', action_name)
            return

        if not self.nav_goal_active():
            self.publish_zero('IDLE', 'no_active_goal')

    def control_loop(self):
        """速度新鲜时转发速度；启用严格门控时才要求 action 目标活跃。"""
        if self.require_active_nav_goal and not self.nav_goal_active():
            self.publish_zero('IDLE', 'no_active_goal')
            return

        if self.age(self.last_nav_cmd_time) > self.cmd_vel_timeout:
            self.publish_zero('CMD_TIMEOUT', 'cmd_vel_nav_smoothed_timeout')
            return

        self.cmd_pub.publish(self.latest_nav_cmd)
        if twist_norm(self.latest_nav_cmd) >= self.nav_cmd_active_threshold:
            self.publish_status('NAV_FORWARD', 'forward_smoothed_cmd')
        else:
            self.publish_status('NAV_ZERO', 'smoothed_cmd_zero')

    def nav_goal_active(self) -> bool:
        """判断是否仍有 Nav2 导航目标处于活动状态。"""
        if not self.require_active_nav_goal:
            return True
        for state in self.goal_states.values():
            status_time = state['status_time']
            if not state['active'] or status_time is None:
                continue
            if self.age(status_time) <= self.nav_goal_status_timeout:
                return True
        return False

    def publish_zero(self, state: str, reason: str):
        """向底盘链路发布零速度并记录状态。"""
        self.latest_nav_cmd = Twist()
        self.cmd_pub.publish(Twist())
        self.publish_status(state, reason)

    def publish_status(self, state: str, reason: str):
        """发布当前门控状态。"""
        if state == self.last_status_state and state in {'NAV_FORWARD', 'NAV_ZERO', 'IDLE'}:
            return
        self.last_status_state = state
        payload = {
            'state': state,
            'reason': reason,
            'require_active_nav_goal': self.require_active_nav_goal,
            'navigate_to_pose_active': self.goal_states['navigate_to_pose']['active'],
            'navigate_through_poses_active': self.goal_states['navigate_through_poses']['active'],
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

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

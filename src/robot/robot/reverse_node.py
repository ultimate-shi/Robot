"""
reverse_node 使用说明：
本节点只负责记录 /odom 历史轨迹，并在收到 /reverse_control/request 后沿历史轨迹倒车回退。

输入：
- /odom：底盘里程计，用于记录最近走过的轨迹。
- /ultrasonic/front_rl、/ultrasonic/front_rr：后向超声波，后方过近时停止回退。
- /reverse_control/request：std_msgs/String，支持 START、CANCEL，或 JSON：{"command": "START"}。

输出：
- /cmd_vel_reverse：回退速度，由 nav_controller_node 在回退期间接管到 /cmd_vel。
- /reverse_control/status：std_msgs/String，JSON 状态，包含 IDLE、RUNNING、COMPLETED、FAILED、CANCELED。

注意：
本节点不直接控制 Nav2 goal，也不绕过 obstacle_avoidance。倒车速度仍由 nav_controller_node
发布到 /cmd_vel，再经过点云模式下的超声波安全层过滤为 /cmd_vel_safe。
"""

import json
import math
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String


def yaw_from_quaternion(q) -> float:
    """从四元数提取 yaw，回退控制只需要平面朝向。"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """把角度约束到 [-pi, pi]，避免跨越 pi 时控制跳变。"""
    return math.atan2(math.sin(angle), math.cos(angle))


class ReverseNode(Node):
    """记录里程计轨迹并执行原路回退。"""

    def __init__(self):
        super().__init__('reverse_node')

        self.declare_parameter('reverse_distance', 0.8)
        self.declare_parameter('reverse_speed', -0.06)
        self.declare_parameter('max_angular_speed', 0.35)
        self.declare_parameter('history_distance', 5.0)
        self.declare_parameter('history_timeout', 60.0)
        self.declare_parameter('sample_distance', 0.03)
        self.declare_parameter('rear_stop_distance', 0.15)
        self.declare_parameter('safe_ultrasonic_distance', 0.25)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('lookahead_distance', 0.12)
        self.declare_parameter('angular_gain', 1.6)

        self.reverse_distance = float(self.get_parameter('reverse_distance').value)
        self.reverse_speed = -abs(float(self.get_parameter('reverse_speed').value))
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.history_distance = float(self.get_parameter('history_distance').value)
        self.history_timeout = float(self.get_parameter('history_timeout').value)
        self.sample_distance = float(self.get_parameter('sample_distance').value)
        self.rear_stop_distance = float(self.get_parameter('rear_stop_distance').value)
        self.safe_ultrasonic_distance = float(
            self.get_parameter('safe_ultrasonic_distance').value
        )
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        control_rate = float(self.get_parameter('control_rate').value)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 50)
        self.create_subscription(String, '/reverse_control/request', self.request_callback, 10)

        self.rear_ranges = {
            '/ultrasonic/front_rl': float('inf'),
            '/ultrasonic/front_rr': float('inf'),
        }
        for topic in self.rear_ranges:
            self.create_subscription(
                Range,
                topic,
                lambda msg, t=topic: self.ultrasonic_callback(msg, t),
                10
            )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_reverse', 10)
        self.status_pub = self.create_publisher(String, '/reverse_control/status', 10)

        self.history = deque()
        self.latest_pose = None
        self.active = False
        self.target_distance = self.reverse_distance
        self.reverse_start_pose = None
        self.reverse_start_time = None
        self.reverse_path = []
        self.last_status = ''

        self.create_timer(1.0 / control_rate, self.control_loop)
        self.publish_status('IDLE', 'ready')
        self.get_logger().info(
            f"reverse_node started: distance={self.reverse_distance:.2f}m, "
            f"speed={self.reverse_speed:.2f}m/s"
        )

    def odom_callback(self, msg: Odometry):
        """记录抽稀后的 /odom 轨迹，保留最近 history_distance/history_timeout 范围。"""
        pose = msg.pose.pose
        stamp = self.get_clock().now()
        sample = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
            'time': stamp,
        }
        self.latest_pose = sample

        if self.active:
            # 回退期间只更新当前位姿，不把倒车轨迹写回历史，避免污染原路快照。
            return

        if not self.history:
            self.history.append(sample)
            return

        last = self.history[-1]
        if self.distance(sample, last) >= self.sample_distance:
            self.history.append(sample)
            self.trim_history()

    def ultrasonic_callback(self, msg: Range, topic: str):
        """缓存后向超声波；无效值按最大量程处理，避免空白模式误触发。"""
        if msg.min_range <= msg.range <= msg.max_range:
            self.rear_ranges[topic] = float(msg.range)
        else:
            self.rear_ranges[topic] = float(msg.max_range)

    def request_callback(self, msg: String):
        """处理 START/CANCEL 回退请求。"""
        command = msg.data.strip()
        distance = self.reverse_distance
        try:
            payload = json.loads(command)
            command = str(payload.get('command', command)).upper()
            distance = float(payload.get('distance', distance))
        except (json.JSONDecodeError, TypeError, ValueError):
            command = command.upper()

        if command == 'START':
            self.start_reverse(distance)
        elif command == 'CANCEL':
            self.cancel_reverse()
        else:
            self.publish_status('FAILED', f'unknown_request:{command}')

    def start_reverse(self, distance: float):
        """启动一次回退，距离受历史轨迹长度约束。"""
        if self.latest_pose is None:
            self.publish_status('FAILED', 'no_odom')
            return
        if self.rear_min_distance() < self.rear_stop_distance:
            self.publish_status('FAILED', 'rear_obstacle')
            self.publish_zero()
            return

        self.reverse_path = self.build_reverse_path()
        available = self.path_distance(self.reverse_path)
        if available < 0.05:
            self.publish_status('FAILED', 'insufficient_history')
            self.publish_zero()
            return
        self.target_distance = max(0.05, min(float(distance), available))

        self.active = True
        self.reverse_start_pose = dict(self.latest_pose)
        self.reverse_start_time = self.get_clock().now()
        self.publish_status('RUNNING', f'distance:{self.target_distance:.2f}')

    def cancel_reverse(self):
        """取消正在执行的回退。"""
        if self.active:
            self.active = False
            self.reverse_path = []
            self.publish_zero()
            self.publish_status('CANCELED', 'manual_cancel')
        else:
            self.publish_status('IDLE', 'no_active_reverse')

    def control_loop(self):
        """按照历史轨迹反向选取目标点，并发布倒车速度。"""
        if not self.active:
            return
        if self.latest_pose is None or self.reverse_start_pose is None:
            self.finish_reverse('FAILED', 'no_odom')
            return
        if self.rear_min_distance() < self.rear_stop_distance:
            self.finish_reverse('FAILED', 'rear_obstacle')
            return

        traveled = self.distance(self.latest_pose, self.reverse_start_pose)
        if traveled >= self.target_distance:
            self.finish_reverse('COMPLETED', f'traveled:{traveled:.2f}')
            return

        elapsed = (self.get_clock().now() - self.reverse_start_time).nanoseconds * 1e-9
        max_duration = max(8.0, self.target_distance / abs(self.reverse_speed) * 3.0)
        if elapsed > max_duration:
            self.finish_reverse('FAILED', 'timeout')
            return

        target = self.pick_reverse_target(traveled)
        if target is None:
            self.finish_reverse('FAILED', 'no_history_target')
            return

        cmd = Twist()
        cmd.linear.x = self.reverse_speed
        angle_error = self.reverse_angle_error(self.latest_pose, target)
        angular = -self.angular_gain * angle_error
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        if self.rear_min_distance() < self.safe_ultrasonic_distance:
            # 后方接近障碍时主动降速，真正 stop 仍由 rear_stop_distance 触发。
            cmd.linear.x *= 0.5

        self.cmd_pub.publish(cmd)
        self.publish_status('RUNNING', f'traveled:{traveled:.2f}')

    def build_reverse_path(self):
        """冻结一份从当前位置向历史轨迹反向展开的回退路径。"""
        if self.latest_pose is None:
            return []

        path = [dict(self.latest_pose)]
        previous = path[0]
        for sample in reversed(self.history):
            if self.distance(previous, sample) < 1e-4:
                continue
            path.append(dict(sample))
            previous = sample
            if self.path_distance(path) >= self.reverse_distance + self.lookahead_distance:
                break
        return path

    def pick_reverse_target(self, traveled: float):
        """从冻结路径中选取当前应倒向的后方目标点。"""
        if not self.reverse_path:
            return None
        target_back_distance = min(self.target_distance, traveled + self.lookahead_distance)
        accum = 0.0
        previous = self.reverse_path[0]
        for sample in self.reverse_path[1:]:
            segment = self.distance(previous, sample)
            if accum + segment >= target_back_distance:
                return sample
            accum += segment
            previous = sample
        return self.reverse_path[-1]

    def path_distance(self, path) -> float:
        """计算路径采样的累计长度。"""
        if len(path) < 2:
            return 0.0
        total = 0.0
        previous = path[0]
        for sample in path[1:]:
            total += self.distance(previous, sample)
            previous = sample
        return total

    def reverse_angle_error(self, pose, target) -> float:
        """计算倒车时车尾指向历史目标点所需的角速度误差。"""
        dx = target['x'] - pose['x']
        dy = target['y'] - pose['y']
        local_x = math.cos(pose['yaw']) * dx + math.sin(pose['yaw']) * dy
        local_y = -math.sin(pose['yaw']) * dx + math.cos(pose['yaw']) * dy
        return normalize_angle(math.atan2(local_y, -local_x))

    def finish_reverse(self, state: str, reason: str):
        """停止输出速度并发布终态。"""
        self.active = False
        self.reverse_path = []
        self.publish_zero()
        self.publish_status(state, reason)

    def trim_history(self):
        """按时间和累计距离修剪历史轨迹。"""
        now = self.get_clock().now()
        while self.history:
            age = (now - self.history[0]['time']).nanoseconds * 1e-9
            if age <= self.history_timeout:
                break
            self.history.popleft()

        while self.available_history_distance() > self.history_distance and len(self.history) > 2:
            self.history.popleft()

    def available_history_distance(self) -> float:
        """计算当前保留历史轨迹的累计长度。"""
        if len(self.history) < 2:
            return 0.0
        total = 0.0
        previous = self.history[0]
        for sample in list(self.history)[1:]:
            total += self.distance(previous, sample)
            previous = sample
        return total

    def rear_min_distance(self) -> float:
        """返回后向两个超声波最小距离。"""
        return min(self.rear_ranges.values()) if self.rear_ranges else float('inf')

    def publish_zero(self):
        """发布零速度，确保回退结束后不会沿用旧命令。"""
        self.cmd_pub.publish(Twist())

    def publish_status(self, state: str, reason: str):
        """发布 JSON 状态；相同状态限频输出，减少调试话题刷屏。"""
        payload = {
            'state': state,
            'reason': reason,
            'rear_min_distance': self.rear_min_distance(),
            'history_distance': self.available_history_distance(),
        }
        data = json.dumps(payload, ensure_ascii=False)
        if state == 'RUNNING' and data == self.last_status:
            return
        self.last_status = data
        self.status_pub.publish(String(data=data))

    @staticmethod
    def distance(a, b) -> float:
        """计算两个平面采样点间距。"""
        return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def main(args=None):
    rclpy.init(args=args)
    node = ReverseNode()
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

#!/usr/bin/env python3
"""使用方法：仅在虚拟导航预演中启动，把 Foxglove 人工目标提交给 Nav2。"""

import json
import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


class GoalManager(Node):
    """统一管理人工目标，并为后续探索和识别目标保留入口。"""

    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('server_timeout', 1.0)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.server_timeout = float(
            self.get_parameter('server_timeout').value)
        self.action_client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter('action_name').value))
        self.status_pub = self.create_publisher(
            String, '/mission/status', 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('goal_topic').value),
            self._goal_callback, 10)
        self.create_service(
            Trigger, '/mission/cancel', self._cancel_callback)
        self.active_goal = None
        self.pending_pose = None
        self.send_in_progress = False
        self._publish_status('idle', '等待 Foxglove 目标')

    def _goal_callback(self, pose):
        error = self._validate_pose(pose)
        if error:
            self._publish_status('rejected', error)
            return
        self.pending_pose = pose
        if self.active_goal is not None:
            self._publish_status('preempting', '正在取消旧目标')
            future = self.active_goal.cancel_goal_async()
            future.add_done_callback(self._cancelled_for_preemption)
            return
        if not self.send_in_progress:
            self._send_pending_goal()

    def _cancelled_for_preemption(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning(f'取消旧目标失败，继续发送新目标: {exc}')
        self.active_goal = None
        self._send_pending_goal()

    def _send_pending_goal(self):
        if self.pending_pose is None:
            return
        if not self.action_client.wait_for_server(
                timeout_sec=self.server_timeout):
            self._publish_status('waiting_nav2', 'Nav2 action 服务尚未就绪')
            return
        pose = self.pending_pose
        self.pending_pose = None
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.send_in_progress = True
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)
        self._publish_status('sending', '正在提交导航目标', pose)

    def _goal_response_callback(self, future):
        self.send_in_progress = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status('error', f'发送导航目标失败: {exc}')
            return
        if not goal_handle.accepted:
            self._publish_status('rejected', 'Nav2 拒绝了导航目标')
            return
        self.active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
        self._publish_status('navigating', 'Nav2 已接受目标')

    def _feedback_callback(self, feedback):
        distance = float(feedback.feedback.distance_remaining)
        self._publish_status(
            'navigating', f'剩余距离 {distance:.2f} m',
            distance_remaining=distance)

    def _result_callback(self, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self.active_goal = None
            self._publish_status('error', f'读取导航结果失败: {exc}')
            return
        self.active_goal = None
        if status == 4:
            self._publish_status('succeeded', '已到达目标')
        elif status == 5:
            self._publish_status('canceled', '导航目标已取消')
        else:
            self._publish_status('failed', f'导航失败，状态码 {status}')
        if self.pending_pose is not None:
            self._send_pending_goal()

    def _cancel_callback(self, request, response):
        del request
        self.pending_pose = None
        if self.active_goal is None:
            response.success = True
            response.message = '当前没有活动导航目标'
            self._publish_status('idle', response.message)
            return response
        self.active_goal.cancel_goal_async()
        response.success = True
        response.message = '已请求取消导航目标'
        self._publish_status('canceling', response.message)
        return response

    def _validate_pose(self, pose):
        if pose.header.frame_id != self.map_frame:
            return f'目标坐标系必须是 {self.map_frame}'
        values = [
            pose.pose.position.x, pose.pose.position.y,
            pose.pose.position.z, pose.pose.orientation.x,
            pose.pose.orientation.y, pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
        if not all(math.isfinite(value) for value in values):
            return '目标包含 NaN 或无穷值'
        q = pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 0.5:
            return '目标四元数无效'
        return ''

    def _publish_status(
            self, state, message, pose=None, distance_remaining=None):
        payload = {'state': state, 'message': message}
        if pose is not None:
            payload['goal'] = {
                'frame_id': pose.header.frame_id,
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
            }
        if distance_remaining is not None:
            payload['distance_remaining'] = distance_remaining
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
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

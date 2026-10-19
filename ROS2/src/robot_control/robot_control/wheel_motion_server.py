#!/usr/bin/env python3
"""使用方法：调用 /execute_wheel_motion Action，执行先转向再四轮定距."""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from robot_interfaces.action import ExecuteWheelMotion
from robot_interfaces.srv import ExecuteEncoderMotion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger


STEER_JOINTS = [
    'front_left_steer_joint', 'front_right_steer_joint',
    'rear_left_steer_joint', 'rear_right_steer_joint']
WHEEL_JOINTS = [
    'front_left_wheel_joint', 'front_right_wheel_joint',
    'rear_left_wheel_joint', 'rear_right_wheel_joint']


class WheelMotionServer(Node):
    """实机调用 DMC T 指令；没有实机服务时按编码器反馈做虚拟闭环."""

    def __init__(self):
        super().__init__('wheel_motion_server')
        self.declare_parameter('wheel_radius', 0.055)
        self.declare_parameter('virtual_speed', 0.10)
        self.declare_parameter('distance_tolerance', 0.005)
        self.declare_parameter('default_steering_timeout', 5.0)
        self.declare_parameter('default_motion_timeout', 30.0)
        self.declare_parameter('feedback_timeout', 0.25)
        self.declare_parameter('steering_limit', 1.57)
        self.declare_parameter('require_encoder_service', True)
        self.radius = float(self.get_parameter('wheel_radius').value)
        self.virtual_speed = float(self.get_parameter('virtual_speed').value)
        self.distance_tolerance = float(
            self.get_parameter('distance_tolerance').value)
        self.default_steering_timeout = float(
            self.get_parameter('default_steering_timeout').value)
        self.default_motion_timeout = float(
            self.get_parameter('default_motion_timeout').value)
        self.feedback_timeout = float(
            self.get_parameter('feedback_timeout').value)
        self.steering_limit = float(self.get_parameter('steering_limit').value)
        self.require_encoder_service = bool(
            self.get_parameter('require_encoder_service').value)
        self.steering = None
        self.wheel_position = None
        self.feedback_monotonic = None
        self.goal_active = False
        self.goal_lock = threading.RLock()
        self.pending_t_future = None
        self.pending_stop_future = None
        self.hardware_motion_submitted = False
        self.steer_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands_raw', 10)
        self.active_pub = self.create_publisher(
            Bool, '/wheel_motion/active', 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 20)
        self.encoder_client = self.create_client(
            ExecuteEncoderMotion, '/hardware/execute_encoder_motion')
        self.stop_encoder_client = self.create_client(
            Trigger, '/hardware/stop_encoder_motion')
        self.server = ActionServer(
            self, ExecuteWheelMotion, '/execute_wheel_motion',
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=lambda _: CancelResponse.ACCEPT)

    def _joint_callback(self, message):
        index = {name: offset for offset, name in enumerate(message.name)}
        if any(name not in index for name in STEER_JOINTS + WHEEL_JOINTS):
            return
        if any(index[name] >= len(message.position)
               for name in STEER_JOINTS + WHEEL_JOINTS):
            return
        self.steering = [message.position[index[name]] for name in STEER_JOINTS]
        self.wheel_position = [message.position[index[name]] for name in WHEEL_JOINTS]
        self.feedback_monotonic = time.monotonic()

    def _goal_callback(self, request):
        values = list(request.steering_angle_rad) + list(request.distance_m)
        if (self.goal_active or
                (self.pending_t_future is not None and
                 not self.pending_t_future.done()) or
                (self.pending_stop_future is not None and
                 not self.pending_stop_future.done()) or
                not all(math.isfinite(value) for value in values) or
                any(abs(value) > self.steering_limit
                    for value in request.steering_angle_rad)):
            return GoalResponse.REJECT
        zero_count = sum(abs(value) < 1e-9 for value in request.distance_m)
        if 0 < zero_count < 4:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _duration_seconds(duration, fallback):
        value = float(duration.sec) + float(duration.nanosec) * 1e-9
        return value if value > 0.0 else fallback

    def _fresh_feedback(self):
        return (self.feedback_monotonic is not None and
                time.monotonic() - self.feedback_monotonic <= self.feedback_timeout)

    def _finish(self, goal_handle, success, message, canceled=False):
        self.wheel_pub.publish(Float64MultiArray(data=[0.0] * 4))
        self.active_pub.publish(Bool(data=False))
        if self.hardware_motion_submitted:
            # DMC1 的 T 目标需显式撤销；零轮速不能覆盖仍在运行的 T。
            if self.stop_encoder_client.service_is_ready():
                future = self.stop_encoder_client.call_async(Trigger.Request())
                self.pending_stop_future = future
                deadline = time.monotonic() + 1.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.01)
                if not future.done() or not future.result().success:
                    success = False
                    message += '；硬件停止服务未确认，请检查 /hardware/chassis_state'
            else:
                success = False
                message += '；硬件停止服务不可用，请检查 /hardware/chassis_state'
            self.hardware_motion_submitted = False
        result = ExecuteWheelMotion.Result()
        result.success = success
        result.message = message
        if canceled:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        with self.goal_lock:
            self.goal_active = False
        return result

    def _execute(self, goal_handle):
        with self.goal_lock:
            if self.goal_active:
                return self._finish(goal_handle, False, '已有定距动作正在执行')
            self.goal_active = True
        request = goal_handle.request
        target_steering = [float(value) for value in request.steering_angle_rad]
        target_distance = [float(value) for value in request.distance_m]
        tolerance = (float(request.steering_tolerance_rad)
                     if request.steering_tolerance_rad > 0.0 else
                     math.radians(1.5))
        steering_timeout = self._duration_seconds(
            request.steering_timeout, self.default_steering_timeout)
        motion_timeout = self._duration_seconds(
            request.motion_timeout, self.default_motion_timeout)
        self.active_pub.publish(Bool(data=True))
        steering_deadline = time.monotonic() + steering_timeout
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, False, '动作已取消', canceled=True)
            self.active_pub.publish(Bool(data=True))
            self.steer_pub.publish(Float64MultiArray(data=target_steering))
            if self._fresh_feedback() and self.steering is not None:
                errors = [target - actual for target, actual in
                          zip(target_steering, self.steering)]
                feedback = ExecuteWheelMotion.Feedback()
                feedback.phase = 'steering'
                feedback.steering_error_rad = errors
                feedback.remaining_distance_m = target_distance
                goal_handle.publish_feedback(feedback)
                if all(abs(error) <= tolerance for error in errors):
                    break
            if time.monotonic() >= steering_deadline:
                return self._finish(
                    goal_handle, False, '四轮转向未在超时前到位')
            time.sleep(0.02)

        if all(abs(value) < 1e-9 for value in target_distance):
            return self._finish(goal_handle, True, '转向完成，距离均为零')
        if not self._fresh_feedback() or self.wheel_position is None:
            return self._finish(goal_handle, False, '缺少有效轮编码器反馈')
        start_position = list(self.wheel_position)
        use_encoder_service = self.encoder_client.service_is_ready()
        if self.require_encoder_service and not use_encoder_service:
            return self._finish(goal_handle, False, '实机编码器服务不可用，禁止轮速回退')
        if use_encoder_service:
            raw_request = ExecuteEncoderMotion.Request()
            raw_request.distance_m = target_distance
            future = self.encoder_client.call_async(raw_request)
            self.pending_t_future = future
            self.hardware_motion_submitted = True
            service_deadline = time.monotonic() + 1.0
            while not future.done() and time.monotonic() < service_deadline:
                time.sleep(0.01)
            if not future.done() or not future.result().success:
                message = ('实机 T 指令服务超时' if not future.done()
                           else future.result().message)
                return self._finish(goal_handle, False, message)
        motion_deadline = time.monotonic() + motion_timeout
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, False, '动作已取消', canceled=True)
            if not self._fresh_feedback() or self.wheel_position is None:
                return self._finish(goal_handle, False, '轮编码器反馈超时')
            traveled = [(current - start) * self.radius for current, start in
                        zip(self.wheel_position, start_position)]
            remaining = [target - actual for target, actual in
                         zip(target_distance, traveled)]
            arrived = [abs(value) <= self.distance_tolerance for value in remaining]
            if not use_encoder_service:
                speeds = [
                    0.0 if done else math.copysign(
                        self.virtual_speed / self.radius, distance)
                    for done, distance in zip(arrived, remaining)]
                self.wheel_pub.publish(Float64MultiArray(data=speeds))
            self.active_pub.publish(Bool(data=True))
            feedback = ExecuteWheelMotion.Feedback()
            feedback.phase = (
                'moving_hardware' if use_encoder_service else 'moving_virtual'
            )
            feedback.steering_error_rad = [
                target - actual for target, actual in
                zip(target_steering, self.steering)]
            feedback.remaining_distance_m = remaining
            goal_handle.publish_feedback(feedback)
            if all(arrived):
                return self._finish(goal_handle, True, '四轮定距运动完成')
            if time.monotonic() >= motion_deadline:
                return self._finish(goal_handle, False, '定距运动超时')
            time.sleep(0.02)

        return self._finish(goal_handle, False, 'ROS 已关闭')


def main(args=None):
    rclpy.init(args=args)
    node = WheelMotionServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

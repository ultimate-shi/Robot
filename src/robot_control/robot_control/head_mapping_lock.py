#!/usr/bin/env python3
"""使用方法：建图 launch 启动本节点，将可动头部持续控制在摄像头正前方。"""

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


class HeadMappingLock(Node):
    """发布头部零位命令，并检查实际关节反馈是否保持在容差内。"""

    def __init__(self):
        super().__init__('head_mapping_lock')
        defaults = {
            'command_topic': '/head_controller/commands',
            'joint_state_topic': '/joint_states',
            'status_topic': '/mapping/head_lock_status',
            'ready_topic': '/mapping/head_ready',
            'yaw_joint': 'head_yaw_joint',
            'pitch_joint': 'head_pitch_joint',
            'target_yaw': 0.0,
            'target_pitch': 0.0,
            'position_tolerance': math.radians(2.0),
            'feedback_timeout': 0.5,
            'require_feedback': False,
            'command_rate': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)
        self.command_pub = self.create_publisher(
            Float64MultiArray, str(self.command_topic), 10)
        self.ready_pub = self.create_publisher(Bool, str(self.ready_topic), 10)
        self.status_pub = self.create_publisher(String, str(self.status_topic), 10)
        self.create_subscription(
            JointState, str(self.joint_state_topic), self._joint_callback, 20)
        self.latest_positions = None
        self.latest_feedback_monotonic = None
        self.create_timer(
            1.0 / max(1.0, float(self.command_rate)), self._command_and_check)

    def _joint_callback(self, message):
        positions = {
            name: message.position[index]
            for index, name in enumerate(message.name)
            if index < len(message.position)
        }
        if self.yaw_joint in positions and self.pitch_joint in positions:
            self.latest_positions = (
                float(positions[self.yaw_joint]),
                float(positions[self.pitch_joint]))
            self.latest_feedback_monotonic = time.monotonic()

    def _command_and_check(self):
        self.command_pub.publish(Float64MultiArray(data=[
            float(self.target_yaw), float(self.target_pitch)]))
        now = time.monotonic()
        feedback_age = (
            None if self.latest_feedback_monotonic is None
            else now - self.latest_feedback_monotonic)
        feedback_fresh = (
            feedback_age is not None
            and feedback_age <= float(self.feedback_timeout))
        if feedback_fresh:
            yaw_error = abs(self._angle_difference(
                self.latest_positions[0], float(self.target_yaw)))
            pitch_error = abs(self._angle_difference(
                self.latest_positions[1], float(self.target_pitch)))
            centered = max(yaw_error, pitch_error) <= float(
                self.position_tolerance)
        else:
            yaw_error = pitch_error = None
            centered = not bool(self.require_feedback)
        ready = bool(centered and (feedback_fresh or not self.require_feedback))
        self.ready_pub.publish(Bool(data=ready))
        status = {
            'state': 'ready' if ready else 'waiting_for_center',
            'require_feedback': bool(self.require_feedback),
            'feedback_age': None if feedback_age is None else round(feedback_age, 3),
            'yaw_error_deg': None if yaw_error is None else round(math.degrees(yaw_error), 3),
            'pitch_error_deg': None if pitch_error is None else round(math.degrees(pitch_error), 3),
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    @staticmethod
    def _angle_difference(value, target):
        return math.atan2(math.sin(value - target), math.cos(value - target))


def main(args=None):
    rclpy.init(args=args)
    node = HeadMappingLock()
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


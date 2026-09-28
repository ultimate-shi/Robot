#!/usr/bin/env python3
"""使用方法：启动后调用 /body_balance/set，以 IMU 和腿反馈控制主动腿."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_interfaces.msg import BodyBalanceStatus, ChassisState
from robot_interfaces.srv import SetBodyBalance
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


LEG_NAMES = ['lap_body_joint_fl', 'lap_body_joint_fr',
             'lap_body_joint_rl', 'lap_body_joint_rr']
CONTROLLER_TOPICS = [
    '/lap_fl_position_controller/commands',
    '/lap_fr_position_controller/commands',
    '/lap_rl_position_controller/commands',
    '/lap_rr_position_controller/commands',
]
LEG_FRONT = [True, True, False, False]


class BodyBalance(Node):
    """ROS 2 版四腿姿态平衡，参数和保护策略来自原 balancer.py."""

    def __init__(self):
        super().__init__('body_balance')
        defaults = {
            'enabled': False,
            'imu_topic': '/sensors/imu/data',
            'joint_state_topic': '/joint_states',
            'hardware_state_topic': '/hardware/chassis_state',
            'control_rate': 10.0,
            'leg_length': 0.060,
            'wheelbase': 0.312,
            'track': 0.280,
            'deadzone_deg': 1.0,
            'imu_filter_alpha': 0.5,
            'kp': 0.1,
            'kd_pitch': 2.0,
            'kd_roll': 2.0,
            'derivative_window': 0.05,
            'max_step_deg': 5.0,
            'soft_margin_mm': 4.0,
            'hard_margin_mm': 1.0,
            'safe_margin_mm': 0.1,
            'imu_timeout': 0.5,
            'overload_current_ma': 2300.0,
            'overload_hold': 2.0,
            'recover_current_ma': 1000.0,
            'recover_hold': 2.0,
            'max_setpoint_deg': 10.0,
            'high_offset_mm': 25.0,
            'low_offset_mm': -25.0,
            'mid_restore_hold': 2.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)
        self.enabled = bool(self.enabled)
        self.level = 'mid'
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.filtered_roll = None
        self.filtered_pitch = None
        self.filtered_gyro_roll = None
        self.filtered_gyro_pitch = None
        self.last_imu_time = None
        self.leg_angle_deg = None
        self.leg_current = None
        self.heights = None
        self.last_command_deg = None
        self.saved_mid_angles = None
        self.restore_until = None
        self.overload_active = False
        self.overload_since = None
        self.recover_since = None
        self.state = 'disabled' if not self.enabled else 'waiting_imu'
        self.command_publishers = [
            self.create_publisher(Float64MultiArray, topic, 10)
            for topic in CONTROLLER_TOPICS]
        self.status_pub = self.create_publisher(
            BodyBalanceStatus, '/body_balance/status', 10)
        self.create_subscription(
            Imu, str(self.imu_topic), self._imu_callback, 20)
        self.create_subscription(
            JointState, str(self.joint_state_topic), self._joint_callback, 20)
        self.create_subscription(
            ChassisState, str(self.hardware_state_topic),
            self._hardware_callback, 10)
        self.create_service(
            SetBodyBalance, '/body_balance/set', self._set_balance)
        self.create_timer(1.0 / max(float(self.control_rate), 1.0), self._control)

    @staticmethod
    def _quaternion_to_roll_pitch(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        roll = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))
        pitch_value = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        return math.asin(pitch_value) * 180.0 / math.pi, roll * 180.0 / math.pi

    def _imu_callback(self, message):
        pitch_deg, roll_deg = self._quaternion_to_roll_pitch(
            message.orientation)
        roll_deg = -roll_deg
        gyro_roll = -float(message.angular_velocity.x)
        gyro_pitch = float(message.angular_velocity.y)
        alpha = float(self.imu_filter_alpha)
        if self.filtered_roll is None:
            self.filtered_roll = roll_deg
            self.filtered_pitch = pitch_deg
            self.filtered_gyro_roll = gyro_roll
            self.filtered_gyro_pitch = gyro_pitch
        else:
            self.filtered_roll = alpha * roll_deg + (1.0 - alpha) * self.filtered_roll
            self.filtered_pitch = alpha * pitch_deg + (1.0 - alpha) * self.filtered_pitch
            self.filtered_gyro_roll = (alpha * gyro_roll +
                                       (1.0 - alpha) * self.filtered_gyro_roll)
            self.filtered_gyro_pitch = (alpha * gyro_pitch +
                                        (1.0 - alpha) * self.filtered_gyro_pitch)
        self.last_imu_time = time.monotonic()

    def _joint_callback(self, message):
        index = {name: offset for offset, name in enumerate(message.name)}
        if any(name not in index for name in LEG_NAMES):
            return
        if any(index[name] >= len(message.position) for name in LEG_NAMES):
            return
        self.leg_angle_deg = [
            math.degrees(message.position[index[name]]) for name in LEG_NAMES]

    def _hardware_callback(self, message):
        self.leg_current = [float(value) for value in message.leg_current_ma]

    def _set_balance(self, request, response):
        level = str(request.level or self.level).strip().lower()
        if level not in ('low', 'mid', 'high'):
            response.success = False
            response.message = 'level 必须是 low、mid 或 high'
            return response
        if (not math.isfinite(request.target_roll_deg) or
                not math.isfinite(request.target_pitch_deg) or
                abs(request.target_roll_deg) > float(self.max_setpoint_deg) or
                abs(request.target_pitch_deg) > float(self.max_setpoint_deg)):
            response.success = False
            response.message = 'roll/pitch 超出允许范围'
            return response
        if level != 'mid' and (abs(request.target_roll_deg) > 1e-9 or
                               abs(request.target_pitch_deg) > 1e-9):
            response.success = False
            response.message = 'high/low 是静态档位，不能同时设置姿态目标'
            return response
        if self.overload_active and level != 'low':
            response.success = False
            response.message = '过流保护仍有效，只允许 low 档'
            return response
        if self.level == 'mid' and level != 'mid' and self.leg_angle_deg:
            self.saved_mid_angles = list(self.leg_angle_deg)
        if self.level != 'mid' and level == 'mid':
            self.restore_until = time.monotonic() + float(self.mid_restore_hold)
            self.last_command_deg = None
        self.enabled = bool(request.enabled)
        self.level = level
        self.target_roll = float(request.target_roll_deg) if level == 'mid' else 0.0
        self.target_pitch = float(request.target_pitch_deg) if level == 'mid' else 0.0
        if not self.enabled:
            self.state = 'disabled'
        response.success = True
        response.message = f'enabled={self.enabled} level={self.level}'
        return response

    def _evaluate_overload(self, now):
        if not self.leg_current:
            return
        maximum = max(self.leg_current)
        if maximum > float(self.overload_current_ma):
            self.recover_since = None
            if self.overload_since is None:
                self.overload_since = now
            elif (not self.overload_active and
                  now - self.overload_since >= float(self.overload_hold)):
                if self.level == 'mid' and self.leg_angle_deg:
                    self.saved_mid_angles = list(self.leg_angle_deg)
                self.overload_active = True
                self.level = 'low'
                self.target_roll = 0.0
                self.target_pitch = 0.0
                self.state = 'overload_low'
        else:
            self.overload_since = None
            if self.overload_active and maximum < float(self.recover_current_ma):
                if self.recover_since is None:
                    self.recover_since = now
                elif now - self.recover_since >= float(self.recover_hold):
                    self.overload_active = False
                    self.state = 'overload_recovered_low'
            else:
                self.recover_since = None

    def _height_to_degree(self, height_mm, index):
        length_mm = float(self.leg_length) * 1000.0
        value = math.degrees(math.asin(max(-1.0, min(1.0, height_mm / length_mm))))
        if not LEG_FRONT[index]:
            value = -value
        return max(-30.0, min(30.0, value))

    def _angles_to_heights(self):
        length_mm = float(self.leg_length) * 1000.0
        result = []
        for index, degree in enumerate(self.leg_angle_deg or [0.0] * 4):
            height = length_mm * math.sin(math.radians(degree))
            result.append(height if LEG_FRONT[index] else -height)
        average = sum(result) / 4.0
        return [value - average for value in result]

    def _publish_commands(self, degrees):
        for publisher, degree in zip(self.command_publishers, degrees):
            publisher.publish(Float64MultiArray(data=[math.radians(degree)]))
        self.last_command_deg = list(degrees)

    def _control(self):
        now = time.monotonic()
        self._evaluate_overload(now)
        if not self.enabled:
            self._publish_status()
            return
        if self.leg_angle_deg is None:
            self.state = 'waiting_joint_state'
            self._publish_status()
            return
        if self.level != 'mid':
            offset = (float(self.high_offset_mm) if self.level == 'high'
                      else float(self.low_offset_mm))
            self._publish_commands([
                self._height_to_degree(offset, index) for index in range(4)])
            self.state = 'static_' + self.level
            self._publish_status()
            return
        if self.restore_until is not None and now < self.restore_until:
            self._publish_commands(self.saved_mid_angles or [0.0] * 4)
            self.state = 'restoring_mid'
            self._publish_status()
            return
        self.restore_until = None
        if self.last_imu_time is None or now - self.last_imu_time > float(self.imu_timeout):
            self.state = 'imu_stale'
            self._publish_status()
            return
        if self.heights is None:
            self.heights = self._angles_to_heights()
        error_roll = self.filtered_roll - self.target_roll
        error_pitch = self.filtered_pitch - self.target_pitch
        if (max(abs(error_roll), abs(error_pitch)) < float(self.deadzone_deg) and
                abs(self.filtered_gyro_roll) < 0.05 and
                abs(self.filtered_gyro_pitch) < 0.05):
            self.state = 'balanced'
            self._publish_status()
            return
        half_length = float(self.wheelbase) * 500.0
        half_width = float(self.track) * 500.0
        raw_pitch = half_length * math.tan(math.radians(error_pitch))
        raw_roll = half_width * math.tan(math.radians(error_roll))
        d_pitch = (-half_length * math.tan(
            self.filtered_gyro_pitch * float(self.derivative_window)) *
            float(self.kd_pitch))
        d_roll = (-half_width * math.tan(
            self.filtered_gyro_roll * float(self.derivative_window)) *
            float(self.kd_roll))
        p = float(self.kp)
        steps = [
            -p * (raw_pitch + raw_roll) + d_pitch + d_roll,
            -p * (raw_pitch - raw_roll) + d_pitch - d_roll,
            +p * (raw_pitch - raw_roll) - d_pitch + d_roll,
            +p * (raw_pitch + raw_roll) - d_pitch - d_roll,
        ]
        length_mm = float(self.leg_length) * 1000.0
        hard = float(self.hard_margin_mm)
        soft = float(self.soft_margin_mm)
        margin = float(self.safe_margin_mm)
        height_limit = length_mm * math.sin(math.radians(30.0))
        for index in range(4):
            distance = height_limit - abs(self.heights[index])
            if distance < soft:
                steps[index] *= max(0.2, distance / soft)
            if ((self.heights[index] >= height_limit - hard and steps[index] > 0.0) or
                    (self.heights[index] <= -height_limit + hard and steps[index] < 0.0)):
                steps[index] = 0.0
            self.heights[index] = max(
                -height_limit + margin,
                min(height_limit - margin, self.heights[index] + steps[index]))
        average = sum(self.heights) / 4.0
        relaxation = 1.0 if max(abs(error_roll), abs(error_pitch)) < 3.0 else 0.4
        self.heights = [value - average * relaxation for value in self.heights]
        targets = [self._height_to_degree(self.heights[index], index)
                   for index in range(4)]
        previous = self.last_command_deg or list(self.leg_angle_deg)
        maximum_step = float(self.max_step_deg)
        targets = [max(previous[index] - maximum_step,
                       min(previous[index] + maximum_step, targets[index]))
                   for index in range(4)]
        self._publish_commands(targets)
        self.state = 'balancing'
        self._publish_status()

    def _publish_status(self):
        message = BodyBalanceStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.enabled = self.enabled
        message.level = self.level
        message.target_roll_deg = self.target_roll
        message.target_pitch_deg = self.target_pitch
        message.actual_roll_deg = self.filtered_roll or 0.0
        message.actual_pitch_deg = self.filtered_pitch or 0.0
        message.overload_active = self.overload_active
        message.state = self.state
        message.leg_target_rad = [math.radians(value) for value in
                                  (self.last_command_deg or [0.0] * 4)]
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BodyBalance()
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

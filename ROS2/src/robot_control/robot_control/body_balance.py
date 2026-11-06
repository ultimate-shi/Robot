#!/usr/bin/env python3
"""作用：基于 IMU 和实际腿角解算完整到位位置、抑制重复微调并响应停止锁；用法：启动后调用 /body_balance/set。"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_interfaces.msg import BodyBalanceStatus, ChassisState
from robot_interfaces.srv import SetBodyBalance
from sensor_msgs.msg import Imu, JointState
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, Float64MultiArray


LEG_NAMES = ['lap_body_joint_fl', 'lap_body_joint_fr',
             'lap_body_joint_rl', 'lap_body_joint_rr']
CONTROLLER_TOPICS = [
    '/lap_fl_position_controller/commands',
    '/lap_fr_position_controller/commands',
    '/lap_rl_position_controller/commands',
    '/lap_rr_position_controller/commands',
]
LEG_FRONT = [True, True, False, False]
DEFAULT_LEG_MIN_DEG = [-30.0, -30.0, -30.0, -30.0]
DEFAULT_LEG_MAX_DEG = [30.0, 30.0, 30.0, 30.0]


class BodyBalance(Node):
    """按 IMU 与实际腿角计算完整到位目标；保留停止、反馈和过流保护。"""

    def __init__(self):
        super().__init__('body_balance')
        defaults = {
            'enabled': False,
            'require_hardware_timing': False,
            'diagnostics_topic': '/diagnostics',
            'imu_topic': '/sensors/imu/data',
            'joint_state_topic': '/joint_states',
            'hardware_state_topic': '/hardware/chassis_state',
            'control_rate': 10.0,
            'position_gain': 1.0,
            'rate_prediction_s': 0.06,
            'target_change_deg': 0.5,
            'attitude_gain': 1.0,
            'rate_damping': 1.0,
            'max_target_lead_deg': 3.0,
            'max_velocity_deg_s': 30.0,
            'max_acceleration_deg_s2': 60.0,
            'resume_deadzone_deg': 1.5,
            'settle_gyro_rad_s': 0.03,
            'resume_gyro_rad_s': 0.05,
            'max_cycle_ratio': 2.0,
            'leg_length': 0.060,
            'leg_min_deg': DEFAULT_LEG_MIN_DEG,
            'leg_max_deg': DEFAULT_LEG_MAX_DEG,
            'wheelbase': 0.312,
            'track': 0.280,
            'deadzone_deg': 1.0,
            'imu_filter_alpha': 0.7,
            'kp': 0.1,
            'kd_pitch': 2.0,
            'kd_roll': 2.0,
            'derivative_window': 0.05,
            'max_step_deg': 5.0,
            'soft_margin_mm': 4.0,
            'hard_margin_mm': 1.0,
            'safe_margin_mm': 0.1,
            'imu_timeout': 0.5,
            'joint_feedback_timeout': 0.5,
            'stop_topic': '/web_console/stop_latched',
            'overload_current_ma': 3200.0,
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
        for name in ('control_rate', 'attitude_gain', 'rate_damping',
                     'max_target_lead_deg', 'max_velocity_deg_s',
                     'max_acceleration_deg_s2', 'deadzone_deg', 'resume_deadzone_deg',
                     'settle_gyro_rad_s', 'resume_gyro_rad_s', 'max_cycle_ratio', 'position_gain', 'rate_prediction_s', 'target_change_deg'):
            if not math.isfinite(float(getattr(self, name))) or float(getattr(self, name)) <= 0:
                raise ValueError(f'{name} 必须为有限正数')
        if (self.resume_deadzone_deg <= self.deadzone_deg or
                self.resume_gyro_rad_s <= self.settle_gyro_rad_s or self.max_cycle_ratio <= 1):
            raise ValueError('退出保持阈值必须大于进入阈值，周期倍率必须大于 1')
        self.leg_min_deg = [float(value) for value in self.leg_min_deg]
        self.leg_max_deg = [float(value) for value in self.leg_max_deg]
        if (len(self.leg_min_deg) != 4 or len(self.leg_max_deg) != 4 or
                any(not math.isfinite(low) or not math.isfinite(high) or
                    low < -90.0 or high > 90.0 or low >= high
                    for low, high in zip(self.leg_min_deg, self.leg_max_deg))):
            raise ValueError('四腿角度限位必须各有四项，且位于 -90°～90° 内')
        self.enabled = bool(self.enabled)
        self.hardware_available = False
        self.last_hardware_feedback_time = None
        self.position_target_sent = False
        self.last_position_imu_time = None
        self.last_control_time = None
        self.command_velocity_deg_s = [0.0] * 4
        self.feedback_interrupted = True
        self.settled = False
        self.level = 'mid'
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.filtered_roll = None
        self.filtered_pitch = None
        self.filtered_gyro_roll = None
        self.filtered_gyro_pitch = None
        self.stop_latched = False
        self.last_joint_time = None
        self.last_imu_time = None
        self.leg_angle_deg = None
        self.leg_current = None
        self.heights = None
        self.height_center = None
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
        self.create_subscription(Bool, str(self.stop_topic), self._stop_callback, 10)
        self.create_subscription(DiagnosticArray, str(self.diagnostics_topic), self._timing_callback, 10)
        self.create_service(
            SetBodyBalance, '/body_balance/set', self._set_balance)
        self.create_timer(1.0 / max(float(self.control_rate), 1.0), self._control)

    def _stop_callback(self, message):
        """停止锁直接撤销平衡，避免网页服务异常时仍持续动作。"""
        self.stop_latched = bool(message.data)
        if self.stop_latched:
            was_enabled = self.enabled
            self.enabled = False
            self.state = 'stopped'
            if was_enabled and self.last_joint_time is not None and (
                    time.monotonic()-self.last_joint_time <= float(self.joint_feedback_timeout)):
                self._hold_current_angles()

    @staticmethod
    def _quaternion_to_roll_pitch(quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        roll = math.atan2(2.0 * (w * x + y * z),
                          1.0 - 2.0 * (x * x + y * y))
        pitch_value = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        return math.asin(pitch_value) * 180.0 / math.pi, roll * 180.0 / math.pi

    def _imu_callback(self, message):
        values = [message.orientation.x, message.orientation.y, message.orientation.z,
                  message.orientation.w, message.angular_velocity.x, message.angular_velocity.y]
        if (not all(math.isfinite(v) for v in values) or
                sum(v*v for v in values[:4]) < 1e-8 or message.orientation_covariance[0] < 0):
            return
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
        if any(index[name] >= len(message.position) or
               not math.isfinite(message.position[index[name]])
               for name in LEG_NAMES):
            return
        self.last_joint_time = time.monotonic()
        self.leg_angle_deg = [
            math.degrees(message.position[index[name]]) for name in LEG_NAMES]

    def _timing_callback(self, message):
        """实机只接受 DMC0 成功查询的真实年龄，重复广播关节缓存不能续期。"""
        for status in message.status:
            if status.name != 'robot_hardware/timing/DMC0':
                continue
            values = {entry.key: entry.value for entry in status.values}
            try:
                age = float(values['feedback_age_ms']) / 1000.0
            except (KeyError, ValueError):
                return
            if not math.isfinite(age) or age < 0:
                return
            stamp = message.header.stamp
            transport_age = max(0.0, self.get_clock().now().nanoseconds / 1e9 -
                                stamp.sec - stamp.nanosec / 1e9)
            self.last_hardware_feedback_time = time.monotonic()-age-transport_age

    def _hardware_callback(self, message):
        self.hardware_available = bool(message.outputs_enabled and not message.manual_recovery_required)
        if self.require_hardware_timing and not self.hardware_available and self.enabled:
            self.enabled = False
            self.feedback_interrupted = True
            self.state = 'hardware_fault'
        self.leg_current = [float(value) for value in message.leg_current_ma]

    def _set_balance(self, request, response):
        if request.enabled and getattr(self, 'stop_latched', False):
            response.success = False
            response.message = '紧急停止已锁定'
            return response
        if request.enabled and self.require_hardware_timing and (
                not self.hardware_available or self.last_hardware_feedback_time is None or
                time.monotonic()-self.last_hardware_feedback_time > self.joint_feedback_timeout):
            response.success = False
            response.message = '实机腿板反馈未就绪或已经过期'
            return response
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
        if self.overload_active and request.enabled and level != 'low':
            response.success = False
            response.message = '过流保护仍有效，只允许 low 档'
            return response
        if self.level == 'mid' and level != 'mid' and self.leg_angle_deg:
            self.saved_mid_angles = list(self.leg_angle_deg)
        if self.level != 'mid' and level == 'mid':
            self.restore_until = time.monotonic() + float(self.mid_restore_hold)
            self.last_command_deg = None
        was_enabled = self.enabled
        previous_level = self.level
        self.enabled = bool(request.enabled)
        self.level = level
        if not self.enabled or not was_enabled or previous_level != level:
            self.heights = None
            self.height_center = None
            self.last_command_deg = None
            self.feedback_interrupted = True
            self.settled = False
        self.target_roll = float(request.target_roll_deg) if level == 'mid' else 0.0
        self.target_pitch = float(request.target_pitch_deg) if level == 'mid' else 0.0
        self.last_position_imu_time = None
        self.settled = False
        if not self.enabled:
            if (was_enabled and self.leg_angle_deg and self.last_joint_time is not None
                    and time.monotonic()-self.last_joint_time <= float(self.joint_feedback_timeout)):
                # 使用方法：关闭平衡时立即以实时反馈角保持，避免位置控制器继续追旧目标。
                self._hold_current_angles()
            self.state = 'disabled'
        response.success = True
        response.message = f'enabled={self.enabled} level={self.level}'
        return response

    def _evaluate_overload(self, now):
        if not self.leg_current:
            return
        maximum = max(abs(current) for current in self.leg_current)
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
        lower, upper = self._height_limits(index)
        bounded = max(lower, min(upper, height_mm))
        value = math.degrees(math.asin(max(-1.0, min(1.0, bounded / length_mm))))
        if not LEG_FRONT[index]:
            value = -value
        return max(self.leg_min_deg[index], min(self.leg_max_deg[index], value))

    def _height_limits(self, index):
        """按前后腿角度方向换算该腿的有符号高度端点，单位 mm。"""
        length_mm = float(self.leg_length) * 1000.0
        sign = 1.0 if LEG_FRONT[index] else -1.0
        endpoints = [
            sign * length_mm * math.sin(math.radians(degree))
            for degree in (self.leg_min_deg[index], self.leg_max_deg[index])]
        return min(endpoints), max(endpoints)

    def _angles_to_heights(self):
        length_mm = float(self.leg_length) * 1000.0
        result = []
        for index, degree in enumerate(self.leg_angle_deg or [0.0] * 4):
            height = length_mm * math.sin(math.radians(degree))
            result.append(height if LEG_FRONT[index] else -height)
        return result

    def _publish_commands(self, degrees):
        bounded = [max(low, min(high, degree)) for degree, low, high in
                   zip(degrees, self.leg_min_deg, self.leg_max_deg)]
        for publisher, degree in zip(self.command_publishers, bounded):
            publisher.publish(Float64MultiArray(data=[math.radians(degree)]))
        self.last_command_deg = bounded

    def _hold_current_angles(self):
        """停止平衡后让底层保持当前反馈；硬件插件继续负责最终软件限位。"""
        for publisher, degree in zip(self.command_publishers,
                                     self.leg_angle_deg):
            publisher.publish(Float64MultiArray(data=[math.radians(degree)]))
        self.last_command_deg = list(self.leg_angle_deg)

    def _reset_tracking(self):
        """反馈恢复或调度中断时从真实腿角重建状态，不追赶旧目标。"""
        self.last_command_deg = list(self.leg_angle_deg)
        self.command_velocity_deg_s = [0.0] * 4
        self.heights = self._angles_to_heights()
        self.height_center = sum(self.heights) / 4.0
        self.settled = False
        self.position_target_sent = False
        self.last_position_imu_time = None
        self.feedback_interrupted = False

    def _limited_commands(self, targets, dt):
        """普通档位共用速度/加速度限制；反馈领先和机械限位优先于平滑。"""
        from robot_control.balance_motion import limit_motion
        targets, velocity = limit_motion(
            targets, self.last_command_deg, self.command_velocity_deg_s,
            self.leg_angle_deg, self.leg_min_deg, self.leg_max_deg,
            dt, self.max_velocity_deg_s, self.max_acceleration_deg_s2,
            self.max_target_lead_deg)
        self.command_velocity_deg_s = velocity
        self._publish_commands(targets)
        # 把速度、领先量及机械限位后的最终目标回算，不能隐藏累积目标。
        length = float(self.leg_length) * 1000.0
        self.heights = [length * math.sin(math.radians(value)) *
                        (1 if LEG_FRONT[i] else -1)
                        for i, value in enumerate(self.last_command_deg)]

    def _control(self):
        now = time.monotonic()
        nominal = 1.0 / float(self.control_rate)
        dt = nominal if self.last_control_time is None else now-self.last_control_time
        self.last_control_time = now
        self._evaluate_overload(now)
        if not self.enabled:
            self.feedback_interrupted = True
            self._publish_status()
            return
        if self.leg_angle_deg is None or self.last_joint_time is None:
            self.state = 'waiting_joint_state'
            self.feedback_interrupted = True
            self._publish_status()
            return
        if now-self.last_joint_time > float(self.joint_feedback_timeout):
            self.state = 'joint_stale'
            self.feedback_interrupted = True
            self._publish_status()
            return
        if self.require_hardware_timing and (not self.hardware_available or
                self.last_hardware_feedback_time is None or
                now-self.last_hardware_feedback_time > float(self.joint_feedback_timeout)):
            self.state = 'joint_stale'
            self.feedback_interrupted = True
            self._publish_status()
            return
        needs_imu = self.level == 'mid'  and not (
            self.restore_until is not None and now < self.restore_until)
        if needs_imu and (self.last_imu_time is None or
                          now-self.last_imu_time > float(self.imu_timeout)):
            self.state = 'imu_stale'
            self.feedback_interrupted = True
            self._publish_status()
            return
        if self.feedback_interrupted or self.last_command_deg is None:
            self._reset_tracking()
        if dt <= 0 or dt > nominal * float(self.max_cycle_ratio):
            self._reset_tracking()
            self.state = 'cycle_late'
            self._publish_status()
            return
        if self.level != 'mid':
            offset = (float(self.high_offset_mm) if self.level == 'high'
                      else float(self.low_offset_mm))
            self._limited_commands([
                self._height_to_degree(offset, index) for index in range(4)], dt)
            self.state = 'static_' + self.level
            self._publish_status()
            return
        if self.restore_until is not None and now < self.restore_until:
            self._limited_commands(self.saved_mid_angles or [0.0] * 4, dt)
            self.state = 'restoring_mid'
            self._publish_status()
            return
        self.restore_until = None
        error_roll = self.filtered_roll - self.target_roll
        error_pitch = self.filtered_pitch - self.target_pitch
        error = max(abs(error_roll), abs(error_pitch))
        gyro = max(abs(self.filtered_gyro_roll), abs(self.filtered_gyro_pitch))
        if self.settled:
            self.settled = error <= self.resume_deadzone_deg and gyro <= self.resume_gyro_rad_s
        elif error < self.deadzone_deg and gyro < self.settle_gyro_rad_s:
            self.settled = True
        if self.settled and self.position_target_sent:
            # 保持已提交的完整到位目标，让板端完成定位，不反复以反馈重写微小目标。
            self.state = 'balanced'
            self._publish_status()
            return
        if self.last_position_imu_time == self.last_imu_time:
            self._publish_status()
            return
        self.last_position_imu_time = self.last_imu_time
        from robot_control.balance_motion import position_plane
        # 角速度只用于短时预测，不再作为逐周期累加的腿高速度。
        predicted_pitch = (math.radians(error_pitch)*self.position_gain +
                           self.filtered_gyro_pitch*self.rate_prediction_s)
        predicted_roll = (math.radians(error_roll)*self.position_gain +
                          self.filtered_gyro_roll*self.rate_prediction_s)
        if self.settled:
            predicted_pitch = predicted_roll = 0.0
        bounds = [self._height_limits(i) for i in range(4)]
        heights = position_plane(
            self._angles_to_heights(), self.height_center, predicted_pitch, predicted_roll,
            float(self.wheelbase)*500.0, float(self.track)*500.0,
            [low+self.safe_margin_mm for low, _ in bounds],
            [high-self.safe_margin_mm for _, high in bounds])
        targets = [self._height_to_degree(height, i) for i, height in enumerate(heights)]
        if (not self.position_target_sent or
                max(abs(goal-last) for goal, last in zip(targets, self.last_command_deg)) >=
                self.target_change_deg):
            # 中档直接发送完整位置；不再被 3° 领先窗和上位机小步轨迹拖慢。
            self._publish_commands(targets)
            self.position_target_sent = True
            self.command_velocity_deg_s = [0.0]*4
            self.heights = heights
            # 限位迫使公共高度平移后保留新基准，避免下一帧又追逐旧高度。
            self.height_center = sum(heights)/4.0
        self.state = 'balanced' if self.settled else 'balancing'
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

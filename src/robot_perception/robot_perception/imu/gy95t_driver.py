#!/usr/bin/env python3
"""使用方法：由 imu.launch.py 读取 GY95T USB 串口并发布标准 ROS IMU 数据."""

from collections import deque
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:  # pragma: no cover - 由运行环境依赖检查覆盖
    serial = None

SerialException = serial.SerialException if serial is not None else OSError


ADDRESS = 0xA4
READ_FUNCTION = 0x03
WRITE_FUNCTION = 0x06
REG_START = 0x08
REG_END = 0x2A
REGISTER_COUNT = REG_END - REG_START + 1


def additive_checksum(data):
    """返回协议使用的 8 位累加校验."""
    return sum(data) & 0xFF


def make_command(function, register, value):
    """生成五字节读写命令."""
    command = bytes((ADDRESS, function, register, value))
    return command + bytes((additive_checksum(command),))


def signed_int16_le(low, high):
    """解析寄存器中的小端有符号 16 位整数."""
    value = (int(high) << 8) | int(low)
    return value - 0x10000 if value & 0x8000 else value


def decode_registers(data, accel_range=0, gyro_range=0):
    """把 0x08 起的寄存器数据转换为 SI 单位."""
    if len(data) < REGISTER_COUNT:
        raise ValueError(f'GY95T 数据长度不足：{len(data)} < {REGISTER_COUNT}')
    accel_lsb = (16384.0, 8192.0, 4096.0, 2048.0)
    gyro_lsb = (131.0, 65.5, 32.8, 16.4)
    if accel_range not in range(4) or gyro_range not in range(4):
        raise ValueError('GY95T 量程索引必须为 0..3')

    def pair(register):
        index = register - REG_START
        return signed_int16_le(data[index], data[index + 1])

    accel = np.array([pair(0x08), pair(0x0A), pair(0x0C)], dtype=float)
    accel = accel / accel_lsb[accel_range] * 9.80665
    gyro = np.array([pair(0x0E), pair(0x10), pair(0x12)], dtype=float)
    gyro = gyro / gyro_lsb[gyro_range] * math.pi / 180.0
    rpy = np.array([pair(0x14), pair(0x16), pair(0x18)], dtype=float)
    rpy = rpy / 100.0 * math.pi / 180.0
    return accel, gyro, rpy


class Gy95tDriver(Node):
    """轮询 GY95T，校验、滤波并发布 sensor_msgs/Imu."""

    def __init__(self):
        super().__init__('gy95t_driver')
        defaults = {
            'device': '/dev/gy95t',
            'baud_rate': 115200,
            'poll_rate': 50.0,
            'serial_timeout': 0.08,
            'reconnect_interval': 1.0,
            'configure_on_start': True,
            'validate_checksum': True,
            'frame_id': 'imu_link',
            'accel_range_index': 0,
            'gyro_range_index': 0,
            'axis_permutation': [0, 1, 2],
            'axis_sign': [1.0, 1.0, 1.0],
            'median_window': 3,
            'low_pass_cutoff': 10.0,
            'gyro_bias_duration': 2.0,
            'gyro_stationary_threshold': 0.08,
            'orientation_variance': 0.03,
            'angular_velocity_variance': 0.0025,
            'linear_acceleration_variance': 0.04,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in defaults:
            setattr(self, name, self.get_parameter(name).value)
        self._validate_parameters()

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.raw_pub = self.create_publisher(
            Imu, '/sensors/imu/raw_unfiltered', qos)
        self.filtered_pub = self.create_publisher(
            Imu, '/sensors/imu/data_raw', qos)
        self.rpy_pub = self.create_publisher(
            Vector3Stamped, '/sensors/imu/vendor_rpy', 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self._serial = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._median_accel = [deque(maxlen=int(self.median_window)) for _ in range(3)]
        self._median_gyro = [deque(maxlen=int(self.median_window)) for _ in range(3)]
        self._filtered_accel = None
        self._filtered_gyro = None
        self._gyro_bias_samples = []
        self._gyro_bias = np.zeros(3)
        self._bias_ready = float(self.gyro_bias_duration) <= 0.0
        self._bias_started = None
        self._last_frame_monotonic = None
        self._last_connect_attempt = 0.0
        self._valid_frames = 0
        self._invalid_frames = 0
        self._reconnects = 0
        self._last_error = ''
        self._thread.start()
        self.create_timer(1.0, self._publish_diagnostics)

    def _validate_parameters(self):
        permutation = [int(v) for v in self.axis_permutation]
        signs = [float(v) for v in self.axis_sign]
        if sorted(permutation) != [0, 1, 2]:
            raise ValueError('axis_permutation 必须是 [0,1,2] 的排列')
        if len(signs) != 3 or any(abs(abs(value) - 1.0) > 1e-6 for value in signs):
            raise ValueError('axis_sign 必须包含三个 +1 或 -1')
        self.axis_permutation = permutation
        self.axis_sign = np.asarray(signs, dtype=float)
        if int(self.median_window) < 1 or int(self.median_window) % 2 == 0:
            raise ValueError('median_window 必须是正奇数')

    def _run(self):
        while rclpy.ok() and not self._stop_event.is_set():
            if self._serial is None and not self._connect():
                self._stop_event.wait(0.05)
                continue
            started = time.monotonic()
            try:
                registers = self._query_registers()
                accel, gyro, rpy = decode_registers(
                    registers,
                    int(self.accel_range_index), int(self.gyro_range_index))
                accel = accel[self.axis_permutation] * self.axis_sign
                gyro = gyro[self.axis_permutation] * self.axis_sign
                rpy = rpy[self.axis_permutation] * self.axis_sign
                self._handle_sample(accel, gyro, rpy, started)
                self._valid_frames += 1
                self._last_frame_monotonic = time.monotonic()
            except (OSError, ValueError, RuntimeError, SerialException) as exc:
                self._invalid_frames += 1
                self._last_error = str(exc)
                self.get_logger().warning(
                    f'GY95T 读取失败，将重新连接 {self.device}: {exc}',
                    throttle_duration_sec=5.0)
                self._disconnect()
            period = 1.0 / max(1.0, float(self.poll_rate))
            self._stop_event.wait(max(0.0, period - (time.monotonic() - started)))

    def _connect(self):
        now = time.monotonic()
        if now - self._last_connect_attempt < float(self.reconnect_interval):
            return False
        self._last_connect_attempt = now
        if serial is None:
            self._last_error = '缺少 python3-serial'
            return False
        try:
            port = serial.Serial(
                str(self.device), int(self.baud_rate), timeout=float(self.serial_timeout),
                write_timeout=float(self.serial_timeout))
            port.reset_input_buffer()
            self._serial = port
            if bool(self.configure_on_start):
                self._write_register(0x03, 0x01)  # 查询输出
                self._write_register(0x02, 0x01)  # 50 Hz 内部更新
                self._write_register(0x07, 0x90)  # ±8 gauss、±2 g、±250°/s
            self._reconnects += 1
            self._last_error = ''
            return True
        except (OSError, RuntimeError, ValueError, SerialException) as exc:
            self._last_error = str(exc)
            self.get_logger().warning(
                f'GY95T 连接失败，将继续重试 {self.device}: {exc}',
                throttle_duration_sec=5.0)
            self._disconnect()
            return False

    def _disconnect(self):
        if self._serial is not None:
            try:
                self._serial.close()
            except OSError:
                pass
        self._serial = None

    def _write_register(self, register, value):
        command = make_command(WRITE_FUNCTION, register, value)
        self._serial.write(command)
        response = self._read_exact(len(command))
        if response != command:
            raise RuntimeError(
                f'GY95T 写寄存器回显错误：{response.hex(" ")}')

    def _query_registers(self):
        command = make_command(READ_FUNCTION, REG_START, REGISTER_COUNT)
        self._serial.reset_input_buffer()
        self._serial.write(command)
        header = self._read_sync_header()
        tail = self._read_exact(REGISTER_COUNT + 1)
        response = header + tail
        if bool(self.validate_checksum) and additive_checksum(response[:-1]) != response[-1]:
            raise ValueError('GY95T 响应累加校验失败')
        return tail[:-1]

    def _read_sync_header(self):
        deadline = time.monotonic() + float(self.serial_timeout)
        matched = bytearray()
        while time.monotonic() < deadline:
            byte = self._serial.read(1)
            if not byte:
                continue
            if not matched and byte[0] != ADDRESS:
                continue
            matched.extend(byte)
            if len(matched) == 2 and matched[1] != READ_FUNCTION:
                matched = bytearray()
                continue
            if len(matched) == 4:
                return bytes(matched)
        raise RuntimeError('GY95T 响应头超时')

    def _read_exact(self, length):
        data = bytearray()
        deadline = time.monotonic() + float(self.serial_timeout)
        while len(data) < length and time.monotonic() < deadline:
            chunk = self._serial.read(length - len(data))
            if chunk:
                data.extend(chunk)
        if len(data) != length:
            raise RuntimeError(f'GY95T 响应超时：{len(data)}/{length} 字节')
        return bytes(data)

    def _handle_sample(self, accel, gyro, rpy, stamp_monotonic):
        stamp = self.get_clock().now().to_msg()
        self.raw_pub.publish(self._make_imu(accel, gyro, stamp, unavailable=True))
        self._publish_vendor_rpy(rpy, stamp)

        if not self._update_bias(gyro, stamp_monotonic):
            return
        accel_filtered = self._filter_vector(accel, self._median_accel, '_filtered_accel')
        gyro_filtered = self._filter_vector(
            gyro - self._gyro_bias, self._median_gyro, '_filtered_gyro')
        self.filtered_pub.publish(
            self._make_imu(accel_filtered, gyro_filtered, stamp, unavailable=True))

    def _update_bias(self, gyro, now):
        if self._bias_ready:
            return True
        if np.linalg.norm(gyro) > float(self.gyro_stationary_threshold):
            self._gyro_bias_samples.clear()
            self._bias_started = None
            return False
        if self._bias_started is None:
            self._bias_started = now
        self._gyro_bias_samples.append(gyro.copy())
        if now - self._bias_started >= float(self.gyro_bias_duration):
            self._gyro_bias = np.mean(self._gyro_bias_samples, axis=0)
            self._bias_ready = True
        return self._bias_ready

    def _filter_vector(self, vector, windows, state_name):
        for index, value in enumerate(vector):
            windows[index].append(float(value))
        median = np.array([np.median(window) for window in windows])
        previous = getattr(self, state_name)
        cutoff = float(self.low_pass_cutoff)
        if previous is None or cutoff <= 0.0:
            result = median
        else:
            dt = 1.0 / max(1.0, float(self.poll_rate))
            alpha = (2.0 * math.pi * cutoff * dt) / (1.0 + 2.0 * math.pi * cutoff * dt)
            result = previous + alpha * (median - previous)
        setattr(self, state_name, result)
        return result

    def _make_imu(self, accel, gyro, stamp, unavailable):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = str(self.frame_id)
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0 if unavailable else float(
            self.orientation_variance)
        (message.angular_velocity.x, message.angular_velocity.y,
         message.angular_velocity.z) = gyro
        (message.linear_acceleration.x, message.linear_acceleration.y,
         message.linear_acceleration.z) = accel
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = float(
                self.angular_velocity_variance)
            message.linear_acceleration_covariance[index] = float(
                self.linear_acceleration_variance)
        return message

    def _publish_vendor_rpy(self, rpy, stamp):
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = str(self.frame_id)
        message.vector.x, message.vector.y, message.vector.z = rpy
        self.rpy_pub.publish(message)

    def _publish_diagnostics(self):
        now = time.monotonic()
        age = -1.0 if self._last_frame_monotonic is None else now - self._last_frame_monotonic
        if self._serial is None:
            level, state = DiagnosticStatus.ERROR, 'disconnected'
        elif not self._bias_ready:
            level, state = DiagnosticStatus.WARN, 'calibrating_gyro_bias'
        elif age > max(0.2, 3.0 / max(1.0, float(self.poll_rate))):
            level, state = DiagnosticStatus.ERROR, 'stale'
        else:
            level, state = DiagnosticStatus.OK, 'ok'
        status = DiagnosticStatus(
            level=level, name='robot_perception/GY95T', message=state,
            hardware_id=str(self.device))
        values = {
            'valid_frames': self._valid_frames,
            'invalid_frames': self._invalid_frames,
            'reconnects': self._reconnects,
            'data_age_sec': round(age, 3),
            'bias_ready': self._bias_ready,
            'gyro_bias_rad_s': np.round(self._gyro_bias, 6).tolist(),
            'last_error': self._last_error,
        }
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def destroy_node(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        self._disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Gy95tDriver()
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

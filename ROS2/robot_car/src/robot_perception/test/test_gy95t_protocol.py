"""使用方法：pytest 运行本文件，验证 GY95T 协议与 SI 单位换算。"""

import math
from types import SimpleNamespace

import numpy as np

from robot_perception.imu.gy95t_driver import (
    READ_FUNCTION, REG_START, REGISTER_COUNT, additive_checksum,
    Gy95tDriver, decode_extended_registers, decode_registers,
    make_command, rpy_to_quaternion)


def _write_int16(data, register, value):
    encoded = int(value) & 0xFFFF
    index = register - REG_START
    data[index] = encoded & 0xFF
    data[index + 1] = encoded >> 8


def test_read_command_uses_documented_checksum():
    """0x08 到 0x2A 的查询命令应为附件代码中的 A4 03 08 23 D2。"""
    command = make_command(READ_FUNCTION, REG_START, REGISTER_COUNT)
    assert command == bytes.fromhex('A4 03 08 23 D2')
    assert additive_checksum(command[:-1]) == command[-1]


def test_register_conversion_uses_little_endian_and_si_units():
    """负数、量程和角度单位必须与 GY95T 附件实现一致。"""
    registers = bytearray(REGISTER_COUNT)
    _write_int16(registers, 0x08, 16384)
    _write_int16(registers, 0x0A, -8192)
    _write_int16(registers, 0x0C, 0)
    _write_int16(registers, 0x0E, 131)
    _write_int16(registers, 0x10, 0)
    _write_int16(registers, 0x12, -262)
    _write_int16(registers, 0x14, 100)
    _write_int16(registers, 0x16, -200)
    _write_int16(registers, 0x18, 9000)

    accel, gyro, rpy = decode_registers(registers)

    assert np.allclose(accel, [9.80665, -4.903325, 0.0])
    assert np.allclose(gyro, [math.radians(1), 0.0, math.radians(-2)])
    assert np.allclose(rpy, np.radians([1.0, -2.0, 90.0]))


def test_extended_registers_preserve_magnetic_raw_counts_and_quaternion_components():
    """厂家未给磁场 SI 比例，只允许保留三轴有符号原始计数。"""
    registers = bytearray(REGISTER_COUNT)
    registers[0x1A - REG_START] = 98
    for address, value in ((0x1D, 490), (0x1F, 3), (0x21, -62),
                           (0x23, 10000), (0x25, 0), (0x27, 0), (0x29, 0)):
        _write_int16(registers, address, value)
    level, mag_counts, quaternion = decode_extended_registers(registers)
    assert level == 98
    assert np.array_equal(mag_counts, [490, 3, -62])
    assert np.allclose(quaternion, [1.0, 0.0, 0.0, 0.0])


def test_vendor_rpy_uses_ros_xyzw_and_quality_gate():
    """厂家 RPY 转换为 ROS xyzw；校准或四元数异常时不声明绝对姿态。"""
    assert np.allclose(
        rpy_to_quaternion([0.0, 0.0, math.pi / 2]),
        [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
    params = SimpleNamespace(
        vendor_orientation_enabled=True, vendor_level_min=80,
        vendor_level_max=120)
    check = Gy95tDriver._vendor_orientation_is_valid
    rpy = np.array([0.0, 0.0, math.pi / 2])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    assert check(params, rpy, 100, quaternion)
    assert not check(params, rpy, 79, quaternion)
    assert not check(params, rpy, 100, np.zeros(4))
    params.vendor_orientation_enabled = False
    assert not check(params, rpy, 100, quaternion)

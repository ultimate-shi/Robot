"""使用方法：pytest 运行本文件，验证 GY95T 协议与 SI 单位换算。"""

import math

import numpy as np

from robot_perception.imu.gy95t_driver import (
    READ_FUNCTION, REG_START, REGISTER_COUNT, additive_checksum,
    decode_registers, make_command)


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

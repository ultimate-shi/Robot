"""作用：防止无效传感器量程破坏状态 API；使用方法：pytest -q src/robot_web/test。"""
import math

from robot_web.console import RobotConsole


def test_invalid_sensor_numbers_are_json_safe():
    assert RobotConsole.json_number(float('inf')) is None
    assert RobotConsole.json_number(float('nan')) is None
    assert RobotConsole.json_number(1.25) == 1.25
    assert math.isfinite(RobotConsole.json_number(0.0))

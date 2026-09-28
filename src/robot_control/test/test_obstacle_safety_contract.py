"""使用方法：pytest 运行本文件，检查超声波最终安全层保持失效关闭。"""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]


def test_ultrasonic_safety_is_fail_closed():
    """测距缺失时必须停车，且不能默认自动倒车。"""
    parameters = yaml.safe_load(
        (PACKAGE / 'config' / 'obstacle_avoidance.yaml').read_text(
            encoding='utf-8'))['obstacle_avoidance']['ros__parameters']

    assert parameters['require_valid_ranges'] is True
    assert parameters['escape_reverse_enabled'] is False


def test_range_subscriptions_use_sensor_qos():
    """SE2 Range 发布端是 Best Effort，两个消费节点必须使用传感器 QoS。"""
    obstacle_source = (
        PACKAGE / 'robot_control' / 'obstacle_avoidance.py'
    ).read_text(encoding='utf-8')
    range_to_scan_source = (
        PACKAGE.parent / 'robot_perception' / 'robot_perception' /
        'virtual_sensors' / 'range_to_scan.py'
    ).read_text(encoding='utf-8')

    assert 'lambda msg, t=topic: self.ultrasonic_callback(msg, t),\n                qos_profile_sensor_data' in obstacle_source
    assert 'lambda msg, s=sensor: self.range_callback(msg, s),\n                qos_profile_sensor_data' in range_to_scan_source
    assert 'ESCAPE_REAR_RANGE_STALE' in obstacle_source

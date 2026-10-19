"""使用方法：pytest 运行本文件，检查超声波最终安全层保持失效关闭。"""

from pathlib import Path

import yaml
from geometry_msgs.msg import Twist
from robot_control.obstacle_avoidance import stop_if_motion_component_blocked


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


def test_blocked_component_stops_composite_motion():
    """复合速度的任一分量被拦截后，不能继续沿错误方向运动。"""
    arc = Twist()
    arc.linear.y = 0.03
    arc.angular.z = 0.10
    assert stop_if_motion_component_blocked(arc, (0.08, 0.03, 0.10))
    assert (arc.linear.x, arc.linear.y, arc.angular.z) == (0.0, 0.0, 0.0)

    spin = Twist()
    spin.angular.z = 0.10
    assert not stop_if_motion_component_blocked(spin, (0.0, 0.0, 0.10))
    assert spin.angular.z == 0.10

    diagonal = Twist()
    diagonal.linear.x = 0.08
    assert stop_if_motion_component_blocked(diagonal, (0.08, 0.03, 0.0))
    assert diagonal.linear.x == 0.0

    turn = Twist()
    turn.linear.x = 0.08
    assert stop_if_motion_component_blocked(turn, (0.08, 0.0, 0.10))
    assert turn.linear.x == 0.0

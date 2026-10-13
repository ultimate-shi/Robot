"""使用方法：pytest 运行本文件，检查视觉六轴方差保护与无效数据处理。"""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest
import yaml


@pytest.fixture
def guard_module(monkeypatch):
    """使用最小 ROS 类型替身加载纯协方差逻辑，无需启动 ROS。"""
    nav_msgs = ModuleType('nav_msgs')
    nav_msgs_msg = ModuleType('nav_msgs.msg')
    nav_msgs_msg.Odometry = type('Odometry', (), {})
    nav_msgs.msg = nav_msgs_msg
    rclpy = ModuleType('rclpy')
    rclpy_executors = ModuleType('rclpy.executors')
    rclpy_executors.ExternalShutdownException = type(
        'ExternalShutdownException', (Exception,), {})
    rclpy_node = ModuleType('rclpy.node')
    rclpy_node.Node = type('Node', (), {})
    for name, module in {
        'nav_msgs': nav_msgs, 'nav_msgs.msg': nav_msgs_msg,
        'rclpy': rclpy, 'rclpy.executors': rclpy_executors,
        'rclpy.node': rclpy_node,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    path = Path(__file__).parents[1] / 'robot_navigation' / 'odometry_covariance_guard.py'
    spec = importlib.util.spec_from_file_location('guard_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_six_pose_variance_floors_preserve_valid_measurements(guard_module):
    covariance = [0.0] * 36
    input_values = [0.001, 0.2, 0.03, 0.001, 0.3, 0.001]
    for index, value in zip(guard_module.POSE_VARIANCE_INDICES.values(), input_values):
        covariance[index] = value
    invalid = guard_module.protect_pose_covariance(
        covariance, guard_module.DEFAULT_VARIANCE_FLOORS, 1e6)
    assert invalid == []
    assert [covariance[index] for index in guard_module.POSE_VARIANCE_INDICES.values()] == [
        0.02, 0.2, 0.1, 0.1, 0.3, 0.03]


def test_invalid_pose_variances_are_not_treated_as_confident(guard_module):
    covariance = [0.0] * 36
    values = [math.nan, -1.0, 0.0, math.inf, 0.02, 0.04]
    for index, value in zip(guard_module.POSE_VARIANCE_INDICES.values(), values):
        covariance[index] = value
    invalid = guard_module.protect_pose_covariance(
        covariance, guard_module.DEFAULT_VARIANCE_FLOORS, 1e6)
    assert invalid == ['x', 'y', 'z', 'roll']
    assert [covariance[index] for index in guard_module.POSE_VARIANCE_INDICES.values()] == [
        1e6, 1e6, 1e6, 1e6, 0.1, 0.04]


def test_zero_covariance_keeps_legacy_planar_floors_but_distrusts_3d(guard_module):
    covariance = [0.0] * 36
    invalid = guard_module.protect_pose_covariance(
        covariance, guard_module.DEFAULT_VARIANCE_FLOORS, 1e6)
    assert invalid == ['z', 'roll', 'pitch']
    assert [covariance[index] for index in guard_module.POSE_VARIANCE_INDICES.values()] == [
        0.02, 0.02, 1e6, 1e6, 1e6, 0.03]


def test_callback_preserves_pose_and_reports_invalid_axes(guard_module):
    published = []
    warnings = []
    guard = object.__new__(guard_module.OdometryCovarianceGuard)
    guard.variance_floors = guard_module.DEFAULT_VARIANCE_FLOORS
    guard.invalid_variance = 1e6
    guard.publisher = SimpleNamespace(publish=published.append)
    guard.get_logger = lambda: SimpleNamespace(
        warning=lambda *args, **kwargs: warnings.append((args, kwargs)))
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                           orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    covariance = [0.0] * 36
    covariance[0] = 0.2
    message = SimpleNamespace(pose=SimpleNamespace(pose=pose, covariance=covariance))

    guard._callback(message)

    assert published == [message]
    assert message.pose.pose is pose
    assert message.pose.pose.position.z == 3.0
    assert message.pose.covariance[14] == 1e6
    assert warnings and 'z' in warnings[0][0][0]


def test_config_defines_all_six_floors_and_invalid_fallback(guard_module):
    path = Path(__file__).parents[1] / 'config' / 'odometry_covariance_guard.yaml'
    config = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'odometry_covariance_guard']['ros__parameters']
    for axis in guard_module.POSE_VARIANCE_INDICES:
        assert config[f'{axis}_variance_floor'] > 0.0
    assert config['invalid_variance'] >= max(
        config[f'{axis}_variance_floor'] for axis in guard_module.POSE_VARIANCE_INDICES)


def test_ekf_3d_inputs_use_only_observed_dimensions():
    path = Path(__file__).parents[1] / 'config' / 'state_estimation.yaml'
    config = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'ekf_filter_node']['ros__parameters']
    assert config['two_d_mode'] is False
    assert config['odom0'] == '/visual_odom'
    assert len(config['odom0_config']) == 15
    assert config['odom0_config'][:6] == [True] * 6
    assert config['odom0_config'][6:] == [False] * 9
    assert config['imu0'] == '/sensors/imu/data'
    assert len(config['imu0_config']) == 15
    assert config['imu0_config'][3:6] == [True, True, False]
    assert config['imu0_config'][6:12] == [False, False, False, True, True, True]
    assert config['imu0_config'][12:] == [False] * 3
    assert 'imu0_twist_rejection_threshold' in config
    assert 'imu0_angular_velocity_rejection_threshold' not in config
    assert config['odom1'] == '/wheel/odom'
    assert len(config['odom1_config']) == 15
    assert config['odom1_config'][6:12] == [True, True, False, False, False, True]

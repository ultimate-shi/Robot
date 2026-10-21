"""文件作用：验证架空探针不会把安全层缩减过的路线误判为通过。

使用方法：在 Jazzy 容器运行 pytest -q test/test_chassis_mode_hardware_probe.py。
"""

from chassis_mode_hardware_probe import summarize


def _samples(safe):
    return [{
        'time': 2.1 + index * 0.1,
        'wheel_target': [1.0] * 4,
        'wheel': [1.0] * 4,
        'steer_target': [0.0] * 4,
        'steer': [0.0] * 4,
        'safe': safe,
        'current_ma': [100.0] * 4,
    } for index in range(65)]


def test_safety_layer_reduced_turn_cannot_pass():
    result = summarize(_samples([0.08, 0.0, 0.007]), 0.0, 9.0,
                       (0.08, 0.0, 0.1), 0.005, 0.1)
    assert result['active_seconds'] >= 5.0
    assert result['safe_command_match'] is False
    assert result['pass'] is False


def test_unmodified_safe_command_can_pass():
    result = summarize(_samples([0.08, 0.0, 0.1]), 0.0, 9.0,
                       (0.08, 0.0, 0.1), 0.005, 0.1)
    assert result['safe_command_match'] is True
    assert result['pass'] is True

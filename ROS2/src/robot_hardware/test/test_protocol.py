"""使用方法：pytest 执行纯协议约束测试，不需要连接真实控制板。"""

import pathlib


def test_hardware_description_keeps_original_protocol_contract():
    source = (pathlib.Path(__file__).parents[1] / 'src' /
              'robot_serial_system.cpp').read_text(encoding='utf-8')
    for command in ('"$info\\n"', '"S \\n"', '"R \\n"', '"f    \\n"'):
        assert command in source
    assert '"I" + std::to_string(index + 4) + " \\n"' in source
    assert 'B115200' in source
    assert 'TIOCM_DTR' in source
    assert 'tcflush(fd_, TCIFLUSH)' in source
    assert 'discovery_retry_count_' in source
    assert 'kStuckDetect' in source
    assert 'kStuckCooldown' in source
    assert 'ranges_mm[index] < sonar_min_valid_mm_' in source
    assert 'std::numeric_limits<float>::quiet_NaN()' in source


def test_wheel_speed_commands_use_individual_ids_and_checked_replies():
    source = (pathlib.Path(__file__).parents[1] / 'src' /
              'robot_serial_system.cpp').read_text(encoding='utf-8')
    assert "command << 'S' << id << ' ' << ticks << ' ' << duration_ms" in source
    assert 'response_command != "S" + std::to_string(id)' in source
    assert 'error != 0' in source
    assert 'stop_wheel_axes(dmc1_.get(), command_timeout_ms_)' in source
    assert 'command_timeout_ms_ > 1000' in source
    assert '"S 0 0 0 0 0\\n"' not in source


def test_calibration_has_six_servos():
    import yaml
    path = pathlib.Path(__file__).parents[1] / 'config' / 'servo_calibration.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert len(data['servos']) == 6
    assert [item['name'] for item in data['servos']] == [
        'head_yaw', 'head_pitch', 'steer_fl', 'steer_fr',
        'steer_rl', 'steer_rr']
    assert all(
        item['brake_deadzone'] >= 2.0
        for item in data['servos'][:2]
    ), '头部停止死区过小会被编码器噪声反复触发，导致相机来回抖动'

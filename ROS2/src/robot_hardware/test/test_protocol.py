"""使用方法：pytest 执行纯协议约束测试，不需要连接真实控制板。"""

import pathlib
import re


def test_hardware_description_keeps_original_protocol_contract():
    source = (pathlib.Path(__file__).parents[1] / 'src' /
              'robot_serial_system.cpp').read_text(encoding='utf-8')
    for command in ('"$info\\n"', '"D \\n"', '"R \\n"', '"f    \\n"'):
        assert command in source
    assert '"I" + std::to_string(index + 4) + " \\n"' in source
    assert 'B115200' in source
    assert 'TIOCM_DTR' in source
    assert 'tcflush(fd_, TCIFLUSH)' in source
    assert 'discovery_retry_count_' in source
    assert 'kStuckDetect' in source
    assert 'kStuckCooldown' in source
    assert 'ranges_mm[index] < sonar_min_valid_mm_' in source
    assert 'if (!outputs_enabled_ || !sonar_valid_[index]) continue;' in source


def test_wheel_control_uses_d_for_motion_stop_and_feedback():
    source = (pathlib.Path(__file__).parents[1] / 'src' /
              'robot_serial_system.cpp').read_text(encoding='utf-8')
    assert '"D" + std::to_string(id) + " " + std::to_string(pwm)' in source
    assert 'command == "D" + std::to_string(id)' in source
    assert 'command_pwm_axis(port, index, 0, duration_ms)' in source
    assert 'command_pwm_axis(dmc1_.get(), index, 0, command_timeout_ms_)' in source
    assert '"S \\n"' not in source
    assert 'command_wheel_axis' not in source
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


def test_steering_resume_threshold_is_below_wheel_release_threshold():
    """使用方法：同时校验舵机恢复调整与轮速放行门槛，避免两者之间出现死区。"""
    import math
    import yaml
    root = pathlib.Path(__file__).parents[3]
    servos = yaml.safe_load((root / 'src/robot_hardware/config/servo_calibration.yaml')
                            .read_text(encoding='utf-8'))['servos']
    control = yaml.safe_load((root / 'src/robot_control/config/chassis_control.yaml')
                             .read_text(encoding='utf-8'))
    ready_rad = control['chassis_controller']['ros__parameters'][
        'steering_ready_tolerance']
    # 0.5° 是硬件 PID 固定基础死区；恢复调整必须早于轮速放行门槛。
    assert all(math.radians(0.5 + item['resume_deadzone']) < ready_rad
               for item in servos[2:])


def test_servo_refresh_stays_ahead_of_firmware_watchdog():
    """使用方法：核对 ROS 刷新周期、失效回中和固件原有看门狗的时间关系。"""
    import yaml
    root = pathlib.Path(__file__).parents[3]
    hardware = yaml.safe_load((root / 'src/robot_hardware/config/hardware_bringup.yaml')
                              .read_text(encoding='utf-8'))['hardware']
    control = yaml.safe_load((root / 'src/robot_control/config/controllers.yaml')
                             .read_text(encoding='utf-8'))
    firmware = (root.parent / 'STM32/Sevo_Echo_F103_V2/Core/Src/tim_servo.c')
    # 独立构建环境可能只有 ROS2 工作区，固件缺失时仍检查 ROS 参数互相匹配。
    watchdog_ms = 100
    if firmware.exists():
        match = re.search(r'#define SERVO_UPDATE_INTERVAL\s+(\d+)',
                          firmware.read_text(encoding='utf-8'))
        assert match is not None
        watchdog_ms = int(match.group(1))
    refresh_ms = hardware['servo_keepalive_period_ms']
    stale_ms = hardware['servo_command_timeout_ms']
    wheel_ms = hardware['ros_command_timeout_ms']
    controller_period_ms = 1000 / control['controller_manager']['ros__parameters']['update_rate']
    assert refresh_ms * 2 < watchdog_ms < controller_period_ms
    assert controller_period_ms * 2 < stale_ms < wheel_ms

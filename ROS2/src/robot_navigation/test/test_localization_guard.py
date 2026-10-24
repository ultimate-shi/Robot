"""作用：验证地图定位确认、超时和重试不绕过导航许可；使用方法：pytest -q 本文件。"""

from robot_navigation.localization_guard import LocalizationState


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


def make_state():
    clock = Clock()
    return LocalizationState(60, 5, 3, 0.25, 0.12, clock), clock


def covariance(xy=0.01, yaw=0.01):
    values = [0.0] * 36
    values[0] = values[7] = xy
    values[35] = yaw
    return values


def test_requires_three_distinct_matches_and_valid_tf():
    state, clock = make_state()
    assert state.snapshot(True)['state'] == 'waiting'
    state.observe(covariance(), 1)
    state.observe(covariance(), 1)
    state.observe(covariance(), 0)
    assert state.count == 1
    assert not state.snapshot(True)['ready']
    state.observe(covariance(), 2)
    state.observe(covariance(), 3)
    assert not state.snapshot(False)['ready']
    assert state.snapshot(True)['ready']
    clock.value = 6
    assert state.snapshot(True)['state'] == 'lost'
    state.observe(covariance(), 4)
    assert not state.snapshot(True)['ready']
    state.observe(covariance(), 5)
    state.observe(covariance(), 6)
    assert state.snapshot(True)['ready']


def test_bad_covariance_revokes_previous_confirmation():
    for xy, yaw in [(1.0, 0.01), (0.01, 1.0), (float('nan'), 0.01), (-1, 0.01)]:
        state, _ = make_state()
        for stamp in range(1, 4):
            state.observe(covariance(), stamp)
        assert state.snapshot(True)['ready']
        state.observe(covariance(xy, yaw), 4)
        assert not state.snapshot(True)['ready']


def test_timeout_does_not_grant_permission_and_retry_needs_new_matches():
    state, clock = make_state()
    clock.value = 61
    assert state.snapshot(True)['state'] == 'failed'
    for stamp in range(1, 4):
        state.observe(covariance(), stamp)
    assert state.snapshot(True)['ready']
    state.reset()
    assert not state.snapshot(True)['ready']
    assert state.snapshot(True)['state'] == 'waiting'


def test_initial_pose_message_uses_radians_and_keeps_uncertainty():
    import math
    import pytest
    from robot_navigation.localization_guard import initial_pose_message
    msg = initial_pose_message(-0.4, -1.0, math.radians(-130), 0.25, 0.12)
    assert msg.header.frame_id == 'map'
    assert msg.pose.pose.position.x == -0.4
    assert math.isclose(2*math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                        math.radians(-130))
    assert msg.pose.covariance[0] == msg.pose.covariance[7] == 0.25
    assert msg.pose.covariance[35] == 0.12
    with pytest.raises(ValueError):
        initial_pose_message(float('nan'), -1.0, 0.0, 0.25, 0.12)


def test_startup_seed_waits_for_reader_and_fresh_odom_then_sends_once():
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time as Stamp
    from robot_navigation.localization_guard import LocalizationGuard, initial_pose_message
    guard = LocalizationGuard.__new__(LocalizationGuard)
    guard.startup_seed = initial_pose_message(-0.4, -1.0, -2.2, 0.25, 0.12)
    count = [0]
    age = [2.0]
    import time
    guard.tf_timeout = 1.0
    guard.input_ready = False
    guard.last_info = time.monotonic()
    guard.state = SimpleNamespace(match_timeout=5.0, reset=lambda: None)
    sent = []
    guard.initial_pub = SimpleNamespace(get_subscription_count=lambda: count[0], publish=sent.append)
    guard.buffer = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(
        header=SimpleNamespace(stamp=Stamp(sec=int(10-age[0])))))
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: Stamp(sec=10)))
    guard.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    guard.try_startup_seed()
    count[0] = 1
    guard.try_startup_seed()
    assert not sent
    age[0] = 0
    guard.try_startup_seed()
    assert not sent  # 已发现订阅者和 odom 仍不等于数据库/图像处理已就绪。
    guard.input_ready = True
    guard.try_startup_seed()
    guard.try_startup_seed()
    assert len(sent) == 1
    assert sent[0].header.stamp.sec == 10


def test_manual_hint_cancels_pending_startup_seed():
    from types import SimpleNamespace
    from robot_navigation.localization_guard import LocalizationGuard, initial_pose_message
    guard = LocalizationGuard.__new__(LocalizationGuard)
    guard.startup_seed = initial_pose_message(0.0, 0.0, 0.0, 0.25, 0.12)
    guard.reset = lambda: None
    published = []
    guard.initial_pub = SimpleNamespace(publish=published.append)
    manual = initial_pose_message(-0.4, -1.0, -2.2, 0.25, 0.12)
    guard.on_initial(manual)
    assert guard.startup_seed is manual
    assert guard.initial_hint is manual
    assert not published  # 未就绪时仅保存最新人工估计。


def test_real_processing_ack_starts_matching_timeout_after_loading():
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time as Stamp
    from robot_navigation.localization_guard import LocalizationGuard
    guard = LocalizationGuard.__new__(LocalizationGuard)
    state, clock = make_state()
    guard.state = state
    guard.input_ready = False
    guard.last_info = None
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    events = []
    guard.publish_status = lambda: events.append(True)
    clock.value = 70.0
    assert state.snapshot(False)['state'] == 'failed'
    guard.on_info(SimpleNamespace(header=SimpleNamespace(stamp=Stamp(sec=0))))
    assert not guard.input_ready
    guard.on_info(SimpleNamespace(header=SimpleNamespace(stamp=Stamp(sec=10))))
    assert guard.input_ready
    assert state.snapshot(False)['state'] == 'waiting'
    assert events == [True]
    clock.value = 71.0
    guard.on_info(SimpleNamespace(header=SimpleNamespace(stamp=Stamp(sec=10))))
    assert state.started == 70.0  # 后续帧不能不停重置超时。

"""作用：验证地图定位确认、超时和重试不绕过导航许可；使用方法：pytest -q 本文件。"""

from robot_navigation.localization_guard import LocalizationState


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


def make_state():
    clock = Clock()
    return LocalizationState(60, 5, 3, 0.25, 0.12, clock, tracking_timeout=5,
                             tracking_max_distance=1, tracking_max_angle=1.57), clock


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


def test_bad_covariance_is_rejected_without_replacing_valid_anchor():
    for xy, yaw in [(1.0, 0.01), (0.01, 1.0), (float('nan'), 0.01), (-1, 0.01)]:
        state, clock = make_state()
        for stamp in range(1, 4):
            state.observe(covariance(), stamp)
        assert state.snapshot(True)['ready']
        state.observe(covariance(xy, yaw), 4)
        assert state.last_stamp == 4
        assert state.last_match == 0
        assert state.snapshot(True)['ready']
        clock.value = 6
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


def test_visual_gaps_use_bounded_odometry_and_visual_return_keeps_permission():
    state, clock = make_state()
    state.tracking_timeout = 60
    for stamp in range(1, 4):
        state.observe(covariance(), stamp)
    assert state.snapshot(True)['ready']
    state.observe_odometry(0, 0, 0)
    clock.value = 10
    state.observe_odometry(0.3, 0, 0.2)
    assert state.snapshot(True, tracking_ready=True)['state'] == 'tracking'
    assert not state.snapshot(False, tracking_ready=True)['ready']
    assert state.snapshot(True, tracking_ready=True)['ready']
    # 正常跟踪时重新看到地图，只需纠偏，不应先撤销许可等三帧。
    state.observe(covariance(), 4)
    assert state.snapshot(True)['state'] == 'localized'
    assert state.tracking_distance == 0
    clock.value = 71
    assert not state.snapshot(True, tracking_ready=True)['ready']
    state.observe(covariance(), 5)
    assert not state.snapshot(True)['ready']  # 真正丢失后仍需重新连续确认。


def test_odometry_never_bootstraps_localization_and_cannot_bypass_limits():
    state, clock = make_state()
    state.tracking_timeout = 60
    state.observe_odometry(0, 0, 0)
    clock.value = 10
    assert not state.snapshot(True, tracking_ready=True)['ready']
    for stamp in range(1, 4):
        state.observe(covariance(), stamp)
    assert state.snapshot(True)['ready']
    clock.value = 20
    assert not state.snapshot(True, tracking_ready=False)['ready']
    state.observe_odometry(1.1, 0, 0)
    assert '里程' in state.snapshot(True, tracking_ready=True)['message']
    state.observe(covariance(), 4)
    state.observe(covariance(), 5)
    state.observe(covariance(), 6)
    assert state.snapshot(True)['ready']
    clock.value = 30
    state.observe_odometry(1.1, 0, 2)
    assert '转角' in state.snapshot(True, tracking_ready=True)['message']


def test_odometry_checks_frames_stamp_covariance_and_accumulates_travel():
    from types import SimpleNamespace
    from nav_msgs.msg import Odometry
    from robot_navigation.localization_guard import LocalizationGuard
    guard = LocalizationGuard.__new__(LocalizationGuard)
    guard.state, _ = make_state()
    guard.tf_timeout = 1
    guard.last_odom = None
    guard.last_odom_stamp = -1
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.header.stamp.sec = 9
    msg.pose.pose.orientation.w = 1
    msg.pose.covariance = covariance()
    guard.on_odom(msg)
    assert guard.last_odom is not None
    msg.header.stamp.sec = 10
    msg.pose.pose.position.x = 0.4
    guard.on_odom(msg)
    assert guard.state.tracking_distance == 0.4
    msg.header.stamp.nanosec = 1
    msg.pose.covariance[0] = float('nan')
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_001))
    guard.on_odom(msg)
    assert guard.last_odom is None
    msg.header.stamp.nanosec = 2
    msg.header.frame_id = 'map'
    msg.pose.covariance = covariance()
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_002))
    guard.on_odom(msg)
    assert guard.last_odom is None
    assert guard.state.tracking_distance == 0.4


def dropout_guard(monkeypatch):
    import json
    from types import SimpleNamespace
    from builtin_interfaces.msg import Time as Stamp
    from robot_navigation.localization_guard import LocalizationGuard
    import robot_navigation.localization_guard as module
    guard = LocalizationGuard.__new__(LocalizationGuard)
    clock = Clock()
    clock.value = 10
    monkeypatch.setattr(module.time, 'monotonic', clock)
    guard.state = LocalizationState(60, 5, 3, 0.25, 0.12, clock,
                                    tracking_timeout=120, tracking_max_distance=5)
    guard.tf_timeout = 1
    guard.input_dropout_timeout = 20
    guard.input_ready_timeout = 120
    guard.last_info = guard.last_odom = guard.input_started = 10
    guard.input_ready = True
    guard.initial_hint = guard.startup_seed = None
    guard.try_startup_seed = lambda: None
    guard.tf_valid = True
    guard.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock.value*1e9)))
    guard.buffer = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(header=SimpleNamespace(
        stamp=Stamp(sec=int(clock.value if guard.tf_valid else clock.value-2), nanosec=int(clock.value%1*1e9)))))
    statuses = []
    guard.status_pub = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))
    guard.ready_pub = SimpleNamespace(publish=lambda msg: None)
    for stamp in (1, 2, 3):
        guard.state.observe(covariance(), stamp)
    guard.publish_status()
    assert statuses[-1]['ready']
    return guard, clock, statuses


def test_short_visual_input_gap_uses_odometry_then_recovers_without_cancel(monkeypatch):
    guard, clock, statuses = dropout_guard(monkeypatch)
    clock.value = guard.last_odom = 18
    guard.publish_status()
    assert statuses[-1]['ready']
    assert statuses[-1]['state'] == 'tracking'
    assert not statuses[-1]['input_fresh']
    assert '短时中断' in statuses[-1]['message']
    clock.value = guard.last_odom = 29.9
    guard.publish_status()
    assert statuses[-1]['ready']
    clock.value = guard.last_odom = guard.last_info = 30
    guard.state.observe(covariance(), 4)
    guard.publish_status()
    assert statuses[-1]['ready']
    assert statuses[-1]['state'] == 'localized'


def test_continuous_dropout_stale_odom_or_tf_still_stop(monkeypatch):
    guard, clock, statuses = dropout_guard(monkeypatch)
    clock.value = 18
    guard.publish_status()
    assert not statuses[-1]['ready']  # 有视觉锚点也不能忽略里程计断流。
    guard.last_odom = 18
    guard.tf_valid = False
    guard.publish_status()
    assert not statuses[-1]['ready']
    guard.tf_valid = True
    clock.value = guard.last_odom = 30.1
    guard.publish_status()
    assert not statuses[-1]['ready']
    assert '超过 20 秒' in statuses[-1]['message']
    clock.value = guard.last_odom = guard.last_info = 31
    guard.publish_status()
    assert not statuses[-1]['ready']  # 持续失效后，单独恢复处理心跳不能复用旧许可。
    for stamp in (4, 5, 6):
        guard.state.observe(covariance(), stamp)
    guard.publish_status()
    assert statuses[-1]['ready']


def test_dropout_never_initializes_localization_or_bypasses_tracking_budget(monkeypatch):
    guard, clock, statuses = dropout_guard(monkeypatch)
    guard.state.reset()
    clock.value = guard.last_odom = 18
    guard.publish_status()
    assert not statuses[-1]['ready']
    for stamp in (4, 5, 6):
        guard.state.observe(covariance(), stamp)
    guard.last_info = 18
    guard.publish_status()
    assert statuses[-1]['ready']
    clock.value = guard.last_odom = 25
    guard.state.tracking_distance = 5.1
    guard.publish_status()
    assert not statuses[-1]['ready']
    assert '里程' in statuses[-1]['message']

"""作用：验证网页导航进度来自当前 Nav2 目标，迟到反馈不会覆盖取消或新目标；使用方法：pytest -q src/robot_web/test。"""
from robot_web.navigation_status import NavigationStatus


class Clock:
    def __init__(self):
        self.value = 10.0

    def __call__(self):
        return self.value


def test_live_navigation_feedback_and_result():
    clock = Clock()
    status = NavigationStatus(clock)
    goal = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    status.planning(goal)
    assert status.snapshot()['phase'] == 'planning'
    status.planned(goal, 12)
    assert status.snapshot()['phase'] == 'ready'
    status.start('first', goal)
    status.accepted('first')
    clock.value += 4
    assert status.snapshot()['start_age_seconds'] == 4
    assert status.feedback('first', 3.2, 5, 18, 1)
    clock.value += 1.5
    current = status.snapshot()
    assert current['phase'] == 'navigating'
    assert current['distance_remaining_m'] == 3.2
    assert current['eta_seconds'] == 18
    assert current['recoveries'] == 1
    assert current['feedback_age_seconds'] == 1.5
    assert current['started_at'] is not None
    assert status.finish('first', 4)
    assert status.snapshot()['phase'] == 'succeeded'
    assert status.snapshot()['distance_remaining_m'] == 0
    assert status.snapshot()['finished_at'] is not None


def test_canceled_and_replaced_goal_ignores_late_feedback():
    status = NavigationStatus()
    goal = {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
    status.start('old', goal)
    status.cancel()
    assert not status.feedback('old', 1, 2, 3, 0)
    assert status.snapshot()['phase'] == 'canceled'
    status.start('new', goal)
    assert not status.finish('old', 4)
    assert status.finish('new', 6, 208, '目标没有可通行路径')
    failed = status.snapshot()
    assert failed['phase'] == 'failed'
    assert failed['message'] == '目标没有可通行路径'
    assert failed['error_code'] == 208


def test_stall_notice_uses_feedback_distance_without_ending_goal():
    clock = Clock()
    status = NavigationStatus(clock, stalled_after=10)
    status.start('goal', {'x': 2.0, 'y': 0.0, 'yaw': 0.0})
    status.feedback('goal', 2.0, 1, 20, 0)
    clock.value += 11
    status.feedback('goal', 1.98, 12, 20, 0)
    assert status.snapshot()['appears_stalled']
    assert status.snapshot()['phase'] == 'navigating'
    status.feedback('goal', 1.90, 13, 18, 0)
    assert not status.snapshot()['appears_stalled']


def test_near_goal_turn_counts_as_progress_and_failure_explains_heading():
    clock = Clock()
    status = NavigationStatus(clock, stalled_after=10)
    goal = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    status.start('goal', goal)
    status.feedback('goal', 0.08, 1, 0, 0, current_yaw=1.4)
    assert status.snapshot()['heading_error_deg'] == 80.2
    clock.value += 11
    status.feedback('goal', 0.08, 12, 0, 0, current_yaw=1.4)
    assert status.snapshot()['appears_stalled']
    status.feedback('goal', 0.08, 13, 0, 0, current_yaw=1.2)
    assert not status.snapshot()['appears_stalled']
    assert status.finish('goal', 6)
    assert '朝向未达标' in status.snapshot()['message']

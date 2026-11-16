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


def test_failure_uses_only_current_nav2_errors_and_preserves_action_detail():
    status = NavigationStatus()
    status.diagnostic('controller_server', 40, '旧目标错误')
    status.start('first', {'yaw': 0})
    status.diagnostic('usb_cam', 40, '不相关错误')
    status.diagnostic('controller_server', 40, 'Failed to make progress')
    status.finish('first', 6)
    assert 'Failed to make progress' in status.snapshot()['message']
    assert '旧目标' not in status.snapshot()['message']
    assert '不相关' not in status.snapshot()['message']
    status.start('second', {'yaw': 0})
    status.finish('second', 6, 0, '机器人超出地图范围')
    assert status.snapshot()['message'] == '机器人超出地图范围'
    status.start('third', {'yaw': 0})
    status.finish('third', 6)
    assert '未返回具体错误信息' in status.snapshot()['message']


def test_bt_child_timeout_warn_is_reason_for_status_six_and_late_log_updates():
    clock = Clock()
    status = NavigationStatus(clock, diagnostic_grace_seconds=2)
    status.start('goal', {'yaw': 0})
    status.finish('goal', 6)
    assert '未返回具体错误信息' in status.snapshot()['message']
    clock.value += 0.1
    status.diagnostic('bt_navigator_navigate_to_pose_rclcpp_node', 30,
                      'Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose')
    assert '重新规划请求确认超时' in status.snapshot()['message']
    status.diagnostic('bt_navigator_navigate_to_pose_rclcpp_node', 40,
                      'Failed to get result for follow_path in node halt!')
    status.diagnostic('bt_navigator', 40, 'Goal failed')
    assert '重新规划请求确认超时' in status.snapshot()['message']
    assert len(status.snapshot()['failure_details']) == 2
    before = status.snapshot()['message']
    clock.value += 3
    status.diagnostic('planner_server', 40, '后来其他任务的错误')
    assert status.snapshot()['message'] == before


def test_old_diagnostics_and_explicit_action_errors_are_not_replaced():
    status = NavigationStatus()
    status.start('new', {'yaw': 0})
    status.diagnostic('controller_server', 40, '旧任务错误', stamp=0)
    assert not status.diagnostics
    status.finish('new', 6, 208, '目标无法到达')
    status.diagnostic('controller_server', 30, 'RegulatedPurePursuitController detected collision ahead!')
    assert status.snapshot()['message'] == '目标无法到达'

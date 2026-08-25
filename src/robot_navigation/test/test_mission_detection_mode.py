"""使用方法：pytest 运行本文件，验证任务结束后恢复入口配置的 YOLO 模式。"""

from types import SimpleNamespace

from robot_navigation.mission_planner import BrainMission


class _Publisher:
    def publish(self, message):
        self.message = message


def test_clear_task_restores_continuous_detection_for_brain_web():
    fake = type('FakeMission', (), {})()
    fake.active_task = 'explore'
    fake.active_target_id = 'frontier'
    fake.last_follow_target = (1.0, 2.0)
    fake.mission_confirmed = True
    fake.plan_generation = 3
    fake.path_pub = _Publisher()
    fake.idle_detection_mode = 'continuous'
    calls = []
    fake._set_detection_mode = calls.append
    fake._publish_status = lambda *_args, **_kwargs: None

    BrainMission._clear_task(fake, 'canceled', '任务已取消')

    assert calls == [True]
    assert fake.active_task == ''
    assert fake.active_target_id == ''


def test_goal_near_map_edge_is_moved_inside_safe_boundary():
    fake = type('FakeMission', (), {})()
    fake.goal_boundary_margin = 0.3
    fake.latest_map = SimpleNamespace(info=SimpleNamespace(
        resolution=0.05, width=40, height=20,
        origin=SimpleNamespace(position=SimpleNamespace(x=-0.6, y=-0.5))))
    warnings = []
    fake.get_logger = lambda: SimpleNamespace(warning=warnings.append)

    bounded = BrainMission._bound_target_to_map(fake, {
        'x': -0.582057, 'y': -0.150966, 'yaw': 0.0,
    })

    assert bounded['x'] == -0.3
    assert bounded['y'] == -0.150966
    assert warnings

"""使用方法：pytest 运行本文件，验证多用户租约、前沿和停靠算法。"""

from concurrent.futures import ThreadPoolExecutor

import numpy as np

from robot_navigation.frontier import find_frontiers, standoff_pose
from robot_brain.multi_user import MultiUserMissionState


class FakeClock:
    """可控单调时钟，用于验证断线十秒后的确定行为."""

    def __init__(self):
        self.value = 100.0

    def __call__(self):
        return self.value


def test_only_one_of_five_clients_gets_control_lease():
    """五个浏览器同时确认时，只允许一个原子取得控制权."""
    state = MultiUserMissionState()

    def confirm(index):
        return state.confirm(
            f'client-{index}', f'request-{index}', f'mission-{index}',
            'explore')

    with ThreadPoolExecutor(max_workers=5) as pool:
        results = list(pool.map(confirm, range(5)))

    assert sum(result['success'] for result in results) == 1
    assert state.snapshot()['controller_id'].startswith('client-')


def test_request_id_is_idempotent_and_non_controller_cannot_cancel():
    state = MultiUserMissionState()
    first = state.confirm('owner', 'same-request', 'mission', 'explore')
    duplicate = state.confirm('other', 'same-request', 'other', 'goto_object')

    assert first == duplicate
    rejected = state.cancel('other', 'cancel-other')
    assert rejected['success'] is False
    assert state.snapshot()['controller_id'] == 'owner'


def test_any_client_can_stop_and_release_control():
    state = MultiUserMissionState()
    state.confirm('owner', 'confirm', 'mission', 'follow_person')

    result = state.stop('observer', 'emergency-stop')

    assert result['success'] is True
    assert state.snapshot()['state'] == 'stopped'
    assert state.snapshot()['controller_id'] == ''


def test_disconnected_controller_expires_after_ten_seconds():
    clock = FakeClock()
    state = MultiUserMissionState(grace_seconds=10.0, clock=clock)
    state.connect('owner')
    state.confirm('owner', 'confirm', 'mission', 'explore')
    state.disconnect('owner')
    clock.value += 9.9
    assert state.tick() is None
    assert state.snapshot()['grace_remaining'] == 0.1
    clock.value += 0.1

    result = state.tick()

    assert result['state'] == 'lease_expired'
    assert state.snapshot()['controller_id'] == ''


def test_object_standoff_uses_surface_clearance_plus_robot_radius():
    pose = standoff_pose(
        robot_xy=(0.0, 0.0), target_xy=(2.0, 0.0),
        surface_clearance=0.5, robot_radius=0.25)

    center_distance = abs(2.0 - pose['x'])
    assert np.isclose(center_distance, 0.75)
    assert np.isclose(center_distance - 0.25, 0.5)
    assert np.isclose(pose['yaw'], 0.0)


def test_frontier_detector_groups_free_unknown_boundary():
    # 左半部分自由、右半部分未知，中间形成一条连续前沿。
    grid = np.full((8, 10), -1, dtype=np.int16)
    grid[:, :5] = 0

    frontiers = find_frontiers(grid.reshape(-1), 10, 8, min_cells=4)

    assert len(frontiers) == 1
    assert frontiers[0]['col'] == 4
    assert frontiers[0]['size'] == 8

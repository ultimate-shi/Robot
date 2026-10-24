"""作用：验证地图自动恢复按真实状态重试且不会并发转换；使用方法：pytest -q 本文件。"""

from concurrent.futures import Future
from types import SimpleNamespace
import time

import pytest
from lifecycle_msgs.msg import State, Transition
from nav2_msgs.srv import ManageLifecycleNodes
from robot_navigation.map_server_recovery import MapServerRecovery, recovery_command


class Client:
    def __init__(self):
        self.srv_name = 'test'
        self.ready = True
        self.requests = []
        self.futures = []
        self.removed = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)

    def respond(self, response):
        self.futures[-1].set_result(response)


def recovery():
    node = MapServerRecovery.__new__(MapServerRecovery)
    node.settings = {'request_timeout': 8.0, 'retry_interval': 3.0,
                     'slow_retry_interval': 30.0, 'max_fast_retries': 5,
                     'check_interval': 2.0}
    node.recovery_clients = {key: Client() for key in ('manager', 'state', 'recover', 'direct')}
    node.pending = None
    node.next_check = 0.0
    node.manager_active = False
    node.manager_bypass_until = 0.0
    node.failures = node.attempts = 0
    node.last_status = None
    node.enabled = True
    node.get_parameter = lambda key: SimpleNamespace(value=node.enabled)
    logger = SimpleNamespace(info=lambda msg: None, warning=lambda msg: None)
    node.get_logger = lambda: logger
    node.status_pub = SimpleNamespace(publish=lambda msg: None)
    return node


def state_response(value):
    return SimpleNamespace(current_state=SimpleNamespace(id=value))


@pytest.mark.parametrize('value,expected', [
    (State.PRIMARY_STATE_UNCONFIGURED, ManageLifecycleNodes.Request.STARTUP),
    (State.PRIMARY_STATE_INACTIVE, ManageLifecycleNodes.Request.RESUME),
    (State.PRIMARY_STATE_ACTIVE, None),
    (State.PRIMARY_STATE_FINALIZED, None),
    (State.TRANSITION_STATE_CONFIGURING, None),
    (State.TRANSITION_STATE_ACTIVATING, None),
])
def test_recovery_only_transitions_stable_unready_states(value, expected):
    assert recovery_command(value) == expected


def test_configure_response_lost_recovers_inactive_without_reconfiguring():
    node = recovery()
    node.tick()
    # 管理器的 is_active 尚未应答前不查询地图或发起恢复。
    node.tick()
    assert len(node.recovery_clients['manager'].requests) == 1
    assert not node.recovery_clients['state'].requests
    node.recovery_clients['manager'].respond(SimpleNamespace(success=False))
    node.tick()
    node.recovery_clients['state'].respond(state_response(State.PRIMARY_STATE_INACTIVE))
    node.tick()
    assert node.recovery_clients['recover'].requests[-1].command == ManageLifecycleNodes.Request.RESUME
    node.recovery_clients['recover'].respond(SimpleNamespace(success=True))
    node.tick()
    assert node.last_status[0] == 'recovering'
    node.recovery_clients['state'].respond(state_response(State.PRIMARY_STATE_ACTIVE))
    node.tick()
    assert node.last_status[0] == 'active'
    assert node.attempts == 1


def test_transition_timeout_rechecks_real_state_instead_of_resending_startup():
    node = recovery()
    node.on_result('state', state_response(State.PRIMARY_STATE_UNCONFIGURED))
    assert node.recovery_clients['recover'].requests[0].command == ManageLifecycleNodes.Request.STARTUP
    kind, future, _ = node.pending
    node.pending = (kind, future, time.monotonic()-9)
    node.tick()
    assert future.cancelled()
    assert node.pending is None
    node.next_check = 0.0
    node.tick()
    assert node.pending[0] == 'state'
    # 远端配置其实已完成，不能因超时重复发 STARTUP。
    node.recovery_clients['state'].respond(state_response(State.PRIMARY_STATE_INACTIVE))
    node.tick()
    assert [r.command for r in node.recovery_clients['recover'].requests] == [
        ManageLifecycleNodes.Request.STARTUP]
    assert node.recovery_clients['direct'].requests[-1].transition.id == Transition.TRANSITION_ACTIVATE


def test_unresponsive_manager_falls_back_to_map_state_and_direct_activation():
    node = recovery()
    node.tick()
    kind, future, _ = node.pending
    node.pending = (kind, future, time.monotonic()-9)
    node.tick()
    node.next_check = 0.0
    node.tick()
    assert node.pending[0] == 'state'
    node.recovery_clients['state'].respond(state_response(State.PRIMARY_STATE_INACTIVE))
    node.tick()
    assert node.recovery_clients['direct'].requests[-1].transition.id == Transition.TRANSITION_ACTIVATE
    node.recovery_clients['direct'].respond(SimpleNamespace(success=True))
    node.tick()
    node.recovery_clients['state'].respond(state_response(State.PRIMARY_STATE_ACTIVE))
    node.tick()
    assert node.last_status[0] == 'active'


def test_active_transitioning_finalized_and_disabled_do_not_restart_map():
    node = recovery()
    for value in (State.PRIMARY_STATE_ACTIVE, State.TRANSITION_STATE_ACTIVATING,
                  State.PRIMARY_STATE_FINALIZED):
        node.on_result('state', state_response(value))
    assert not node.recovery_clients['recover'].requests
    node.enabled = False
    node.next_check = 0.0
    node.tick()
    assert not node.recovery_clients['manager'].requests
    assert node.last_status[0] == 'disabled'


def test_persistent_failure_backs_off_and_keeps_retrying():
    node = recovery()
    node.recovery_clients['manager'].ready = False
    node.recovery_clients['state'].ready = False
    for _ in range(5):
        node.next_check = 0.0
        node.tick()
    assert node.failures == 5
    assert node.next_check - time.monotonic() > 29
    node.recovery_clients['state'].ready = True
    node.next_check = 0.0
    node.tick()
    assert len(node.recovery_clients['state'].requests) == 1


def test_manager_fallback_window_survives_slow_retry_delay():
    node = recovery()
    node.failures = 4
    node.tick()
    kind, future, _ = node.pending
    node.pending = (kind, future, time.monotonic()-9)
    node.tick()
    assert node.failures == 5
    assert node.next_check-time.monotonic() > 29
    # 慢重试不能恰好耗尽后备窗口，导致永远只探测卡住的管理器。
    assert node.manager_bypass_until > node.next_check+29

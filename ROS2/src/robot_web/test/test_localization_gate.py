"""作用：验证网页在定位失败、断流和手动修正期间禁止自主任务；使用方法：pytest -q 本文件。"""

from types import SimpleNamespace
import threading
import time
import json

import pytest
from robot_web.console import RobotConsole


def console_stub():
    console = RobotConsole.__new__(RobotConsole)
    console.data_lock = threading.RLock()
    console.motion_generation = 0
    console.cancel_started = None
    console.localization_required = True
    console.localization_pending = False
    console.saving_map = False
    console.localization = None
    console.settings = {'localization_status_timeout': 1.0}
    return console


def test_missing_stale_and_failed_localization_block_all_autonomous_entries():
    console = console_stub()
    for status in [None, ({'ready': True, 'message': 'ok'}, time.monotonic()-2),
                   ({'ready': False, 'message': '定位超时'}, time.monotonic())]:
        console.localization = status
        for call in [lambda: console.plan(1, 2, 0), lambda: console.navigate('old'),
                     lambda: console.start_task('explore'), lambda: console.confirm_model('id')]:
            with pytest.raises(ValueError):
                call()
    console.localization = ({'ready': True, 'message': 'ok'}, time.monotonic())
    console.require_localized()
    console.saving_map = True
    with pytest.raises(ValueError, match='保存'):
        console.require_localized()


def test_pending_initial_hint_does_not_accept_old_ready_messages():
    console = console_stub()
    console.localization_pending = True
    console.on_localization(SimpleNamespace(data=json.dumps({'ready': True})))
    assert console.localization is None
    console.on_localization(SimpleNamespace(data=json.dumps({'ready': False, 'state': 'waiting'})))
    assert not console.localization_pending
    assert not console.localization_status()['ready']
    console.on_localization(SimpleNamespace(data=json.dumps({'ready': True, 'message': 'ok'})))
    console.require_localized()


def test_manual_recovery_does_not_require_localization_and_virtual_compatible():
    console = console_stub()
    console.localization_required = False
    console.settings['static_map'] = True
    assert console.localization_status()['state'] == 'virtual'
    console.require_localized()


def test_loss_cancels_current_task_and_returns_manual_with_zero_speed():
    from robot_web.control import ControlState
    from geometry_msgs.msg import Twist
    console = console_stub()
    console.control = ControlState()
    console.control.set_mode('navigation')
    from robot_web.navigation_status import NavigationStatus
    console.navigation_status = NavigationStatus()
    console.last_navigation_event = None
    console.navigation_events = []
    console.navigation_event_pub = SimpleNamespace(publish=console.navigation_events.append)
    console.stop_state_pub = SimpleNamespace(publish=lambda msg: None)
    outputs = []
    console.output = SimpleNamespace(publish=outputs.append)
    console.chassis_output = SimpleNamespace(publish=outputs.append)
    canceled = []
    console.cancel_motion = lambda: canceled.append(True)
    console.control_loop()
    assert canceled
    assert '定位服务未就绪' in json.loads(console.navigation_events[-1].data)['message']
    assert console.control.mode == 'manual'
    assert all(isinstance(msg, Twist) and msg.linear.x == 0 and msg.angular.z == 0 for msg in outputs)


def test_later_localization_loss_preserves_first_navigation_failure():
    console = console_stub()
    from robot_web.navigation_status import NavigationStatus
    from robot_web.control import ControlState
    console.navigation_status = NavigationStatus()
    console.navigation_status.start('goal', {'yaw': 0})
    console.navigation_status.finish('goal', 6, 0, '重新规划请求确认超时')
    before = console.navigation_status.snapshot()
    console.control = ControlState()
    console.control.set_mode('navigation')
    console.cancel_motion = lambda: None
    console.stop_state_pub = console.output = console.chassis_output = SimpleNamespace(publish=lambda msg: None)
    console.last_navigation_event = None
    console.navigation_event_pub = SimpleNamespace(publish=lambda msg: None)
    console.control_loop()
    assert console.navigation_status.snapshot()['message'] == before['message']
    assert console.navigation_status.snapshot()['finished_at'] == before['finished_at']
    assert console.control.mode == 'manual'

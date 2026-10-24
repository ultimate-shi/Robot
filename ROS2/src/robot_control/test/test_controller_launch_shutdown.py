"""作用：验证 Ctrl+C 时控制器回调不重复关闭整车；使用方法：pytest -q test/test_controller_launch_shutdown.py。"""

from pathlib import Path
import runpy
import signal
from types import SimpleNamespace

from launch.actions import EmitEvent


_ACTIONS = runpy.run_path(str(
    Path(__file__).resolve().parents[1] / 'launch' / 'controllers.launch.py'))[
        'bootstrap_exit_actions']


def test_signal_and_shutdown_do_not_restart_or_repeat_shutdown():
    next_spawner = object()
    for code, is_shutdown in [(-signal.SIGINT, False),
                              (-signal.SIGTERM, False),
                              (1, True), (0, True)]:
        assert _ACTIONS(SimpleNamespace(returncode=code),
                        SimpleNamespace(is_shutdown=is_shutdown),
                        'wheel_controller', next_spawner) == []


def test_actual_failure_still_stops_launch_and_success_continues():
    context = SimpleNamespace(is_shutdown=False)
    next_spawner = object()
    assert _ACTIONS(SimpleNamespace(returncode=0), context,
                    'wheel_controller', next_spawner) == [next_spawner]
    actions = _ACTIONS(SimpleNamespace(returncode=1), context,
                       'wheel_controller', next_spawner)
    assert len(actions) == 1
    assert isinstance(actions[0], EmitEvent)

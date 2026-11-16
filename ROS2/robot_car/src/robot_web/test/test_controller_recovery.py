"""作用：验证底盘重启后控制器查询可超时重试并忽略迟到响应；用法：pytest 本文件。"""

from concurrent.futures import Future
import threading
import time
from types import SimpleNamespace

from robot_web.console import RobotConsole


def test_controller_query_recovers_after_server_restart():
    console = RobotConsole.__new__(RobotConsole)
    console.data_lock = threading.RLock()
    console.settings = {'static_map': False, 'controller_status_timeout': 5.0}
    old = Future()
    old.set_result(SimpleNamespace(controller=[]))
    new = Future()
    removed = []
    console.controller_request = old
    console.controller_request_started = time.monotonic()-6
    console.controller_client = SimpleNamespace(
        service_is_ready=lambda: True, call_async=lambda _: new,
        remove_pending_request=removed.append)
    console.refresh_diagnostics()
    assert removed == [old]
    assert console.controller_request is new
    console.on_controller_response(old)
    assert console.controller_request is new
    new.set_result(SimpleNamespace(controller=[SimpleNamespace(name='wheel_controller', state='active')]))
    assert console.controller_request is None
    assert console.controllers[0] == {'wheel_controller': 'active'}

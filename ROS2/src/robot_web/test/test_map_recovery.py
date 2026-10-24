"""作用：验证静态地图话题缺失时从地图服务恢复；使用方法：pytest -q test/test_map_recovery.py。"""

from concurrent.futures import Future
import threading

from nav_msgs.srv import GetMap
from robot_web.console import RobotConsole


class ReadyMapClient:
    def service_is_ready(self):
        return True

    def call_async(self, _request):
        future = Future()
        response = GetMap.Response()
        response.map.info.width = 1
        response.map.info.height = 1
        response.map.data = [0]
        future.set_result(response)
        return future


class UnavailableClient:
    def service_is_ready(self):
        return False


def test_static_map_service_recovers_missing_topic():
    console = RobotConsole.__new__(RobotConsole)
    console.data_lock = threading.RLock()
    console.settings = {'static_map': True}
    console.map = None
    console.map_request = None
    console.map_client = ReadyMapClient()
    console.controller_request = None
    console.controller_client = UnavailableClient()

    console.refresh_diagnostics()

    assert console.map[0].info.width == 1
    assert console.map_request is None

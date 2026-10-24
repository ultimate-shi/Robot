"""作用：验证网页无人查看画面时不执行 JPEG 编码；使用方法：pytest -q test/test_image_preview_demand.py。"""

import threading
import time
from types import SimpleNamespace

from robot_web.console import RobotConsole


def test_image_encoding_only_for_recent_viewer(monkeypatch):
    console = RobotConsole.__new__(RobotConsole)
    console.data_lock = threading.RLock()
    console.images = {}
    console.image_seen = {}
    console.frame_requested = {}
    conversions = []
    console.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *args, **kwargs:
                                     conversions.append(1) or object())
    monkeypatch.setattr('robot_web.console.cv2.imencode',
                        lambda *_args, **_kwargs: (True, SimpleNamespace(
                            tobytes=lambda: b'jpeg')))
    message = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=1, nanosec=2)))

    console.on_image('left', message)
    assert console.image_seen['left'] > 0
    assert not conversions

    console.frame_requested['left'] = time.monotonic()
    console.on_image('left', message)
    assert len(conversions) == 1
    assert console.images['left'][0] == b'jpeg'

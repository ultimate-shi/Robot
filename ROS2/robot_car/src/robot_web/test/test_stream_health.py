"""作用：验证画面短时延迟不会标成持续断流，恢复后自动正常；使用方法：pytest -q 本文件。"""
from robot_web.stream_health import stream_status


def test_stream_waiting_delayed_lost_and_recovered():
    assert stream_status(None, 10, 2, 20)['state'] == 'waiting'
    assert stream_status(10, 11, 2, 20)['state'] == 'healthy'
    delayed = stream_status(10, 13, 2, 20)
    assert delayed == {'fresh': False, 'state': 'delayed', 'age_seconds': 3}
    assert stream_status(10, 30.1, 2, 20)['state'] == 'lost'
    assert stream_status(31, 31.1, 2, 20)['state'] == 'healthy'

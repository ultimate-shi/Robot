"""作用：验证头部归中停止后必须显式恢复；用法：在 Jazzy 环境 pytest -q 本文件。"""
from types import SimpleNamespace as NS
import time

from robot_control.head_mapping_lock import HeadMappingLock


def test_stop_holds_measured_head_and_reset_does_not_resume_old_command():
    head = object.__new__(HeadMappingLock)
    head.stop_latched = False
    head.paused = False
    head.latest_positions = (0.2, -0.1)
    head.latest_feedback_monotonic = time.monotonic()
    head.feedback_timeout = 0.5
    sent = []
    head.command_pub = NS(publish=sent.append)
    head._stop_callback(NS(data=True))
    assert list(sent[-1].data) == [0.2, -0.1]
    assert head.stop_latched and head.paused
    response = head._resume(None, NS())
    assert not response.success
    head._stop_callback(NS(data=False))
    assert head.paused
    assert head._resume(None, NS()).success
    assert not head.paused


def test_stale_head_feedback_does_not_invent_stop_hold_pose():
    head = object.__new__(HeadMappingLock)
    head.stop_latched = False
    head.latest_positions = (0.2, -0.1)
    head.latest_feedback_monotonic = time.monotonic()-2
    head.feedback_timeout = 0.5
    sent = []
    head.command_pub = NS(publish=sent.append)
    head._stop_callback(NS(data=True))
    assert not sent

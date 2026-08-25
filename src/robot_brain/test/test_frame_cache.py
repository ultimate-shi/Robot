"""
使用方法：python3 -m pytest 运行本文件。

验证 YOLO 检测只绑定同时间戳的有界相机帧。
"""

from robot_brain.frame_cache import TimestampedFrameCache


def test_frame_cache_matches_exact_timestamp():
    cache = TimestampedFrameCache(max_frames=3, max_age_seconds=3.0)
    cache.add(10, b'first', now=1.0)
    cache.add(20, b'second', now=1.1)
    assert cache.get(20, now=1.2) == b'second'
    assert cache.get(21, now=1.2) is None


def test_frame_cache_prunes_by_count_and_age():
    cache = TimestampedFrameCache(max_frames=2, max_age_seconds=3.0)
    cache.add(10, b'oldest', now=1.0)
    cache.add(20, b'middle', now=2.0)
    cache.add(30, b'newest', now=2.5)
    assert cache.get(10, now=2.5) is None
    assert len(cache) == 2
    assert cache.get(20, now=5.1) is None
    assert cache.get(30, now=5.1) == b'newest'

"""使用方法：RosBridge 用本模块将 YOLO 检测与同时间戳压缩帧配对。"""

from collections import OrderedDict
import threading
import time


class TimestampedFrameCache:
    """按 ROS 时间戳保存少量压缩帧，供检测结果精确配对。"""

    def __init__(self, max_frames=30, max_age_seconds=3.0):
        self.max_frames = max(1, int(max_frames))
        self.max_age_seconds = max(0.1, float(max_age_seconds))
        self.frames = OrderedDict()
        self.lock = threading.RLock()

    def add(self, stamp_ns, data, now=None):
        received_at = time.monotonic() if now is None else float(now)
        with self.lock:
            self.frames[int(stamp_ns)] = (bytes(data), received_at)
            self.frames.move_to_end(int(stamp_ns))
            self._prune(received_at)

    def get(self, stamp_ns, now=None):
        current = time.monotonic() if now is None else float(now)
        with self.lock:
            self._prune(current)
            value = self.frames.get(int(stamp_ns))
            return None if value is None else value[0]

    def _prune(self, now):
        expired = [stamp for stamp, (_, received_at) in self.frames.items()
                   if now - received_at > self.max_age_seconds]
        for stamp in expired:
            self.frames.pop(stamp, None)
        while len(self.frames) > self.max_frames:
            self.frames.popitem(last=False)

    def __len__(self):
        with self.lock:
            return len(self.frames)

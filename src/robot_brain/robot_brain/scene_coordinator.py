"""使用方法：SharedRobotState 用本模块冻结场景并等待 Qwen 后的新 YOLO 快照。"""

from dataclasses import replace
import threading
import time

from robot_brain.contracts import SceneSnapshot


class SceneCoordinator:
    """协调场景版本和等待者；暂停通知不会创建或清空真实检测快照。"""

    def __init__(self, lock=None):
        self.lock = lock or threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.current = SceneSnapshot.freeze({'detections': []})

    def update_detection(self, detection_state, image_stamp_ns=None):
        with self.condition:
            self.current = SceneSnapshot.freeze(
                detection_state, image_stamp_ns)
            self.condition.notify_all()
            return self.current

    def pair_image(self, stamp_ns):
        """检测先到、图像后到时只补齐同一快照的图片时间戳。"""
        with self.condition:
            if self.current.detection_stamp_ns == stamp_ns:
                self.current = replace(
                    self.current, image_stamp_ns=int(stamp_ns))
                self.condition.notify_all()

    def snapshot(self):
        with self.lock:
            return self.current

    def wait_after(self, stamp_ns, timeout):
        deadline = time.monotonic() + float(timeout)
        with self.condition:
            while True:
                current = self.current
                if (current.detection_stamp_ns is not None
                        and current.detection_stamp_ns != stamp_ns
                        and current.state in ('valid', 'valid_empty')):
                    return current
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.condition.wait(remaining)

    def wake(self):
        """状态变化仅唤醒检查，不把 paused/error 制造成新场景。"""
        with self.condition:
            self.condition.notify_all()

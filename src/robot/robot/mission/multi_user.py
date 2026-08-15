"""局域网多用户控制权租约和任务状态的纯 Python 实现."""

from collections import OrderedDict
from dataclasses import asdict, dataclass
import threading
import time


@dataclass
class LeaseSnapshot:
    """可序列化的控制权和任务快照."""

    state: str = 'stopped'
    task: str = ''
    controller_id: str = ''
    controller_short_id: str = ''
    message: str = '系统已停止'
    grace_remaining: float = 0.0
    revision: int = 0


class MultiUserMissionState:
    """用互斥锁保证同一时刻只有一个客户端能取得控制权."""

    def __init__(self, grace_seconds=10.0, request_cache_size=512,
                 clock=None):
        self.grace_seconds = float(grace_seconds)
        self.request_cache_size = int(request_cache_size)
        self.clock = clock or time.monotonic
        self._lock = threading.RLock()
        self._snapshot = LeaseSnapshot()
        self._last_heartbeat = {}
        self._connected = set()
        self._processed = OrderedDict()

    @staticmethod
    def short_id(client_id):
        """匿名显示客户端编号，不把它误写成身份认证."""
        text = str(client_id or '')
        return text[:8] if text else ''

    def connect(self, client_id):
        """记录 WebSocket 连接或刷新后的重连."""
        now = self.clock()
        with self._lock:
            self._connected.add(client_id)
            self._last_heartbeat[client_id] = now
            return self.snapshot(now)

    def heartbeat(self, client_id):
        """更新连接心跳；无控制权的客户端也可保持在线状态."""
        now = self.clock()
        with self._lock:
            self._connected.add(client_id)
            self._last_heartbeat[client_id] = now
            return self.snapshot(now)

    def disconnect(self, client_id):
        """断线不立即停止，为页面刷新保留十秒恢复窗口."""
        with self._lock:
            self._connected.discard(client_id)
            self._last_heartbeat[client_id] = self.clock()

    def confirm(self, client_id, request_id, mission_id, task):
        """原子确认任务；并发请求中只有第一个客户端取得租约."""
        with self._lock:
            cached = self._cached(request_id)
            if cached is not None:
                return cached
            controller = self._snapshot.controller_id
            if controller and controller != client_id:
                result = self._result(
                    False, 'busy',
                    f'机器人正由用户 {self.short_id(controller)} 控制')
                return self._remember(request_id, result)
            now = self.clock()
            self._connected.add(client_id)
            self._last_heartbeat[client_id] = now
            self._snapshot.controller_id = client_id
            self._snapshot.controller_short_id = self.short_id(client_id)
            self._snapshot.task = str(task)
            self._snapshot.state = 'previewing'
            self._snapshot.message = f'任务 {mission_id} 已确认，仅预演路径'
            self._snapshot.revision += 1
            result = self._result(True, 'confirmed', self._snapshot.message)
            return self._remember(request_id, result)

    def cancel(self, client_id, request_id):
        """只有控制者能普通取消，取消后同时释放租约."""
        with self._lock:
            cached = self._cached(request_id)
            if cached is not None:
                return cached
            if not self._snapshot.controller_id:
                result = self._result(True, 'idle', '当前没有活动任务')
            elif self._snapshot.controller_id != client_id:
                result = self._result(False, 'forbidden', '只有控制者可以取消任务')
            else:
                self._clear('canceled', '控制者已取消任务')
                result = self._result(True, 'canceled', self._snapshot.message)
            return self._remember(request_id, result)

    def release(self, client_id, request_id):
        """控制者主动释放租约，并停止当前任务."""
        with self._lock:
            cached = self._cached(request_id)
            if cached is not None:
                return cached
            if self._snapshot.controller_id != client_id:
                result = self._result(False, 'forbidden', '当前客户端没有控制权')
            else:
                self._clear('stopped', '控制者已释放控制权')
                result = self._result(True, 'released', self._snapshot.message)
            return self._remember(request_id, result)

    def stop(self, client_id, request_id):
        """任何客户端都可立即停止，且不经过模型推理队列."""
        with self._lock:
            cached = self._cached(request_id)
            if cached is not None:
                return cached
            actor = self.short_id(client_id) or 'unknown'
            self._clear('stopped', f'用户 {actor} 已立即停止机器人')
            result = self._result(True, 'stopped', self._snapshot.message)
            return self._remember(request_id, result)

    def tick(self):
        """检查控制者断线；宽限期结束后停止任务并释放租约."""
        now = self.clock()
        with self._lock:
            controller = self._snapshot.controller_id
            if not controller or controller in self._connected:
                return None
            last_seen = self._last_heartbeat.get(controller, now)
            if now - last_seen < self.grace_seconds:
                return None
            self._clear('stopped', '控制者断线超时，任务已停止并释放控制权')
            return self._result(True, 'lease_expired', self._snapshot.message)

    def snapshot(self, now=None):
        """返回共享状态，并计算控制者断线剩余宽限时间."""
        with self._lock:
            current = now if now is not None else self.clock()
            data = asdict(self._snapshot)
            controller = self._snapshot.controller_id
            if controller and controller not in self._connected:
                last_seen = self._last_heartbeat.get(controller, current)
                data['grace_remaining'] = round(max(
                    0.0, self.grace_seconds - (current - last_seen)), 1)
            return data

    def _clear(self, state, message):
        self._snapshot.state = state
        self._snapshot.task = ''
        self._snapshot.controller_id = ''
        self._snapshot.controller_short_id = ''
        self._snapshot.message = message
        self._snapshot.grace_remaining = 0.0
        self._snapshot.revision += 1

    def _result(self, success, state, message):
        shared = self.snapshot()
        shared.pop('controller_id', None)
        return {
            'success': bool(success),
            'state': state,
            'message': message,
            'shared': shared,
        }

    def _cached(self, request_id):
        if not request_id:
            return None
        return self._processed.get(request_id)

    def _remember(self, request_id, result):
        if request_id:
            self._processed[request_id] = result
            self._processed.move_to_end(request_id)
            while len(self._processed) > self.request_cache_size:
                self._processed.popitem(last=False)
        return result

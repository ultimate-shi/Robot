"""作用：管理网页模式、软件停止锁和手动速度；使用方法：由网页 ROS 节点持有。"""
import math
import threading
import time


class ControlState:
    """任一页面均可操作；松键和旧指令保持停车保护。"""

    MODES = {'manual', 'navigation', 'explore', 'follow'}

    def __init__(self, command_timeout=0.35, clock=time.monotonic):
        self.lock = threading.RLock()
        self.clock = clock
        self.command_timeout = command_timeout
        self.stop_latched = False
        self.mode = 'manual'
        self.last_command = None
        self.velocity = (0.0, 0.0)
        self.stream_sequences = {}
        self.active_stream = None

    @property
    def stopped(self):
        with self.lock:
            return self.stop_latched

    def heartbeat(self):
        """兼容旧页面；心跳不再参与运动授权。"""

    def stop(self):
        with self.lock:
            self.stop_latched = True
            self.velocity = (0.0, 0.0)
            self.last_command = None
            self.mode = 'manual'

    def reset_stop(self):
        with self.lock:
            self.stop_latched = False
            self.velocity = (0.0, 0.0)
            self.last_command = None

    def set_mode(self, mode):
        with self.lock:
            if self.stopped:
                raise ValueError('软件停止锁尚未解除')
            if mode not in self.MODES:
                raise ValueError('不支持的控制模式')
            self.mode = mode
            self.velocity = (0.0, 0.0)
            self.last_command = None

    def command(self, linear, angular, client_id, sequence):
        return self._command(linear, angular, client_id, sequence, None)

    def stream_command(self, linear, angular, client_id, sequence, token):
        """持续连接使用有序输入；HTTP 兼容入口仍接受未使用过流的旧页面。"""
        return self._command(linear, angular, client_id, sequence, token)

    def release_stream(self, token):
        """仅释放该连接仍持有的速度，不停车另一页面后来接管的输入。"""
        with self.lock:
            if self.active_stream is token:
                self.velocity = (0.0, 0.0)
                self.last_command = None
                self.active_stream = None

    def _command(self, linear, angular, client_id, sequence, token):
        with self.lock:
            if not all(math.isfinite(x) for x in (linear, angular)):
                raise ValueError('速度必须是有限数字')
            if self.stopped or self.mode != 'manual':
                if linear == 0 and angular == 0:
                    self.velocity = (0.0, 0.0)
                    return
                raise ValueError('当前不允许手动运动')
            if not client_id or len(client_id) > 100:
                raise ValueError('页面指令标识无效')
            # 流连接开启后，丢弃本页面更早的 HTTP/流帧，尤其不能在松键后重放非零速度。
            previous = self.stream_sequences.get(client_id)
            if previous is not None and sequence <= previous:
                return False
            if token is not None or previous is not None:
                if client_id not in self.stream_sequences and len(self.stream_sequences) >= 64:
                    del self.stream_sequences[next(iter(self.stream_sequences))]
                self.stream_sequences[client_id] = sequence
            self.active_stream = token
            self.velocity = (linear, angular)
            self.last_command = self.clock()
            return True

    def manual_velocity(self):
        with self.lock:
            if (self.stopped or self.mode != 'manual' or
                    self.last_command is None or
                    self.clock() - self.last_command > self.command_timeout):
                return (0.0, 0.0)
            return self.velocity

    def snapshot(self):
        with self.lock:
            stopped = self.stopped
            age = None if self.last_command is None else max(0.0, self.clock()-self.last_command)
            return {'stopped': stopped, 'connected': True, 'mode': self.mode,
                    'input_transport': 'stream' if self.active_stream is not None else 'http',
                    'input_age_seconds': age,
                    'input_fresh': not stopped and self.mode == 'manual' and
                    age is not None and age <= self.command_timeout}

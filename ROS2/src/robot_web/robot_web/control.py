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
            self.velocity = (linear, angular)
            self.last_command = self.clock()

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
            return {'stopped': stopped, 'connected': True, 'mode': self.mode}

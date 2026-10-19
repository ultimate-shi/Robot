"""使用方法：ros2 run robot_logging log_manager --ros-args -p config_file:=... -p mode:=real。"""

import os
import signal
import subprocess
import time

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from std_srvs.srv import Trigger

from robot_logging.storage import (
    directory_bytes, ensure_layout, finish_session, load_config,
    new_session, prune, recover_abandoned, space_ok,
)


class LogManager(Node):
    """在 ROS launch 生命周期内管理两个独立 rosbag2 录制进程。"""

    def __init__(self):
        super().__init__('robot_log_manager')
        self.declare_parameter('config_file', '')
        self.declare_parameter('mcap_config_file', '')
        self.declare_parameter('mode', 'virtual')
        self.config = load_config(self.get_parameter('config_file').value)
        if self.config.get('require_mount', False) and not os.path.ismount(self.config['root']):
            raise RuntimeError(f"日志目录没有挂载到容器：{self.config['root']}")
        self.mcap_config = str(self.get_parameter('mcap_config_file').value)
        self.mode = str(self.get_parameter('mode').value)
        ensure_layout(self.config['root'])
        for path in recover_abandoned(self.config['root']):
            self.get_logger().warning(f'恢复异常退出的录制会话：{path}')
        self.routine = None
        self.incident = None
        self.routine_path = None
        self.incident_path = None
        self.routine_started = 0.0
        self.incident_started = 0.0
        self.last_incident_end = 0.0
        self.last_cleanup = 0.0
        self.next_routine_retry = 0.0
        self.create_subscription(Log, '/rosout', self._on_log, 100)
        self.create_service(Trigger, '/logging/trigger_incident', self._manual_trigger)
        self.create_timer(1.0, self._tick)
        self._tick()

    def _command(self, path, full):
        config = self.config
        command = [
            'ros2', 'bag', 'record', '-s', 'mcap', '--all',
            '-o', str(path / 'bag'),
            '--storage-config-file', self.mcap_config,
            '--max-bag-size', str(config['segment_bytes']),
            '--max-bag-duration', str(config['segment_seconds']),
            '--disable-keyboard-controls',
            '--include-hidden-topics',
        ]
        if not full:
            command += ['--exclude-topic-types', *config['routine_excluded_types']]
            if config['routine_excluded_regex']:
                command += ['--exclude-regex', config['routine_excluded_regex']]
        return command

    def _start(self, kind, reason=''):
        reserve = (int(self.config['incident_max_bytes']) if kind == 'incidents'
                   else int(self.config['segment_bytes']))
        prune(self.config, self._active_paths())
        if not space_ok(self.config, reserve):
            self.get_logger().error('日志磁盘配额或系统空闲空间不足，拒绝启动录制')
            return False
        path = new_session(self.config['root'], kind, self.mode, reason)
        log_stream = open(path / 'recorder.log', 'wb')
        try:
            process = subprocess.Popen(
                self._command(path, kind == 'incidents'),
                stdout=log_stream, stderr=subprocess.STDOUT,
                start_new_session=True)
        except OSError as exc:
            finish_session(path)
            self.get_logger().error(f'启动 MCAP 录制失败：{exc}')
            return False
        finally:
            log_stream.close()
        if kind == 'routine':
            self.routine, self.routine_path = process, path
            self.routine_started = time.monotonic()
        else:
            self.incident, self.incident_path = process, path
            self.incident_started = time.monotonic()
        self.get_logger().info(f'{kind} MCAP 录制已启动：{path}')
        return True

    @staticmethod
    def _stop(process, path):
        if process is None:
            return
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        finish_session(path)

    def _active_paths(self):
        return [path for path in (self.routine_path, self.incident_path) if path]

    def _trigger(self, reason):
        now = time.monotonic()
        if self.incident is not None:
            return True, '当前异常片段正在录制，已合并触发'
        if now - self.last_incident_end < int(self.config['incident_cooldown_seconds']):
            return False, '异常录制处于冷却期'
        if self._start('incidents', reason):
            return True, f"已开始录制 {self.config['incident_seconds']} 秒完整片段：{reason}"
        return False, '存储空间不足或录制器启动失败'

    def _manual_trigger(self, request, response):
        del request
        response.success, response.message = self._trigger('manual')
        return response

    def _on_log(self, msg):
        if msg.level >= Log.ERROR and msg.name != self.get_name():
            self._trigger(f'rosout:{msg.name}:{msg.msg[:120]}')

    def _tick(self):
        now = time.monotonic()
        if self.routine is not None and self.routine.poll() is not None:
            self.get_logger().error(f'日常录制器退出，状态码 {self.routine.returncode}')
            self._stop(self.routine, self.routine_path)
            self.routine, self.routine_path = None, None
            self.next_routine_retry = now + 30
        if self.incident is not None:
            elapsed = now - self.incident_started
            # 为最后一块 MCAP 写入和停止收尾预留一个切片的空间。
            stop_at = max(0, int(self.config['incident_max_bytes']) -
                          int(self.config['segment_bytes']))
            limit = directory_bytes(self.incident_path) >= stop_at
            if (elapsed >= int(self.config['incident_seconds']) or limit or
                    not space_ok(self.config) or self.incident.poll() is not None):
                self._stop(self.incident, self.incident_path)
                self.incident, self.incident_path = None, None
                self.last_incident_end = now
                if limit:
                    self.get_logger().error('异常片段达到 2 GB 上限，已停止录制')
        if now - self.last_cleanup >= 10:
            prune(self.config, self._active_paths())
            self.last_cleanup = now
        if self.routine is not None:
            if not space_ok(self.config):
                self.get_logger().error('日志磁盘配额或空闲空间不足，停止日常录制')
                self._stop(self.routine, self.routine_path)
                self.routine, self.routine_path = None, None
                self.next_routine_retry = now + 30
            elif now - self.routine_started >= int(self.config['segment_seconds']):
                self._stop(self.routine, self.routine_path)
                self.routine, self.routine_path = None, None
        if self.routine is None and now >= self.next_routine_retry:
            if not self._start('routine'):
                self.next_routine_retry = now + 30

    def destroy_node(self):
        self._stop(self.incident, self.incident_path)
        self._stop(self.routine, self.routine_path)
        return super().destroy_node()


def main():
    rclpy.init()
    node = LogManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch 可能在收尾期间再次发送 SIGINT；先让 rosbag2 完成 MCAP 页脚。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()

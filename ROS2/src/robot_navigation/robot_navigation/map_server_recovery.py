"""作用：恢复启动超时后未激活的静态地图服务；使用方法：由 nav2.launch.py 或同名独立 launch 启动。"""

import json
import math
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def recovery_command(state):
    """只恢复稳定的未配置/未激活状态；转换中或已终结时不发起非法转换。"""
    if state == State.PRIMARY_STATE_UNCONFIGURED:
        return ManageLifecycleNodes.Request.STARTUP
    if state == State.PRIMARY_STATE_INACTIVE:
        return ManageLifecycleNodes.Request.RESUME
    return None


class MapServerRecovery(Node):
    """串行请求，优先通过原管理器恢复；管理器无响应时按真实状态单步恢复。"""

    def __init__(self):
        super().__init__('map_server_recovery')
        defaults = {
            'enabled': True,
            'map_state_service': '/map_server/get_state',
            'map_change_service': '/map_server/change_state',
            'manager_active_service': '/lifecycle_manager_map/is_active',
            'manager_service': '/lifecycle_manager_map/manage_nodes',
            'status_topic': '/mapping/map_server_recovery/status',
            'startup_grace': 6.0,
            'check_interval': 2.0,
            'request_timeout': 8.0,
            'retry_interval': 3.0,
            'max_fast_retries': 5,
            'slow_retry_interval': 30.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.settings = {key: self.get_parameter(key).value for key in defaults}
        for key in ('startup_grace', 'check_interval', 'request_timeout', 'retry_interval',
                    'max_fast_retries', 'slow_retry_interval'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(f'{key} 必须为正有限值')
        self.recovery_clients = {
            'manager': self.create_client(Trigger, self.settings['manager_active_service']),
            'state': self.create_client(GetState, self.settings['map_state_service']),
            'direct': self.create_client(ChangeState, self.settings['map_change_service']),
            'recover': self.create_client(ManageLifecycleNodes, self.settings['manager_service']),
        }
        self.status_pub = self.create_publisher(String, self.settings['status_topic'], 1)
        self.pending = None
        self.next_check = time.monotonic() + self.settings['startup_grace']
        self.manager_active = False
        self.manager_bypass_until = 0.0
        self.failures = 0
        self.attempts = 0
        self.last_status = None
        self.create_timer(self.settings['check_interval'], self.tick)

    def publish_status(self, state, message):
        payload = {'state': state, 'message': message, 'attempts': self.attempts,
                   'consecutive_failures': self.failures,
                   'manager_active': self.manager_active}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        marker = (state, message)
        if marker != self.last_status:
            if state in ('retrying', 'unavailable', 'finalized'):
                self.get_logger().warning(message)
            elif state == 'recovering':
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            self.last_status = marker

    def defer(self, state, message, failed=False):
        if failed:
            self.failures += 1
        delay = (self.settings['slow_retry_interval']
                 if self.failures >= self.settings['max_fast_retries']
                 else self.settings['retry_interval'])
        self.next_check = time.monotonic() + delay
        self.publish_status(state, message)

    def request(self, kind, message):
        client = self.recovery_clients[kind]
        if not client.service_is_ready():
            self.defer('unavailable', f'地图自动恢复等待服务：{client.srv_name}', failed=True)
            if kind in ('manager', 'recover'):
                self.manager_active = False
                self.manager_bypass_until = self.next_check + self.settings['slow_retry_interval']
            return
        self.pending = (kind, client.call_async(message), time.monotonic())

    def tick(self):
        if not self.get_parameter('enabled').value:
            if self.pending is not None:
                kind, future, _ = self.pending
                self.recovery_clients[kind].remove_pending_request(future)
                future.cancel()
                self.pending = None
            self.publish_status('disabled', '地图自动恢复已禁用')
            return
        now = time.monotonic()
        if self.pending is not None:
            kind, future, started = self.pending
            if not future.done():
                if now - started >= self.settings['request_timeout']:
                    # 超时不等于远端转换失败；下一轮先探测管理器、重读真实状态。
                    self.recovery_clients[kind].remove_pending_request(future)
                    future.cancel()
                    self.pending = None
                    self.defer('retrying', f'地图自动恢复请求超时（{kind}），稍后重新核对状态', failed=True)
                    if kind in ('manager', 'recover'):
                        self.manager_active = False
                        self.manager_bypass_until = self.next_check + self.settings['slow_retry_interval']
                return
            self.pending = None
            try:
                self.on_result(kind, future.result())
            except Exception as exc:
                self.defer('retrying', f'地图自动恢复请求异常：{exc}', failed=True)
                if kind in ('manager', 'recover'):
                    self.manager_active = False
                    self.manager_bypass_until = self.next_check + self.settings['slow_retry_interval']
            return
        if now >= self.next_check:
            # 此服务与管理器的启动/恢复共用互斥回调组，等它答复后才读取地图状态。
            if now < self.manager_bypass_until:
                self.request('state', GetState.Request())
            else:
                self.request('manager', Trigger.Request())

    def on_result(self, kind, response):
        if kind == 'manager':
            self.manager_active = response.success
            self.manager_bypass_until = 0.0
            self.request('state', GetState.Request())
        elif kind == 'state':
            state = response.current_state.id
            if state == State.PRIMARY_STATE_ACTIVE:
                self.failures = 0
                self.next_check = time.monotonic() + self.settings['check_interval']
                self.publish_status('active', '地图服务已激活，持续检查实际生命周期状态')
            elif state == State.PRIMARY_STATE_FINALIZED:
                self.next_check = time.monotonic() + self.settings['slow_retry_interval']
                self.publish_status('finalized', '地图服务已终结，不对终结状态强行发起恢复')
            else:
                command = recovery_command(state)
                if command is None:
                    self.defer('transitioning', f'地图服务正在转换或状态未知（{state}），等待稳定')
                    return
                request = ManageLifecycleNodes.Request(command=command)
                self.attempts += 1
                action = '启动配置和激活' if command == request.STARTUP else '恢复激活'
                self.publish_status('recovering', f'地图服务未就绪，自动{action}，第 {self.attempts} 次')
                if time.monotonic() < self.manager_bypass_until:
                    # 管理器可能卡在遗失的响应上。地图服务自身仍将生命周期转换串行执行；
                    # 先读取稳定状态，再单步 configure/activate，和人工恢复采取相同路径。
                    direct = ChangeState.Request()
                    direct.transition.id = (Transition.TRANSITION_CONFIGURE
                        if state == State.PRIMARY_STATE_UNCONFIGURED else Transition.TRANSITION_ACTIVATE)
                    self.publish_status('recovering', '地图管理器无响应，按地图服务实际状态执行后备恢复')
                    self.request('direct', direct)
                else:
                    self.request('recover', request)
        elif kind in ('recover', 'direct'):
            if not response.success:
                self.defer('retrying', '地图本次恢复未完成，稍后按实际状态继续恢复', failed=True)
            else:
                # 管理器成功响应后仍读回节点状态，不把请求成功当作地图已经可用。
                self.request('state', GetState.Request())


def main(args=None):
    rclpy.init(args=args)
    node = MapServerRecovery()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

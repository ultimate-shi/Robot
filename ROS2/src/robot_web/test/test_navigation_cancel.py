"""作用：验证取消确认、迟到目标和旧速度拦截；使用方法：在 Jazzy 环境执行 pytest -q 本文件。"""
from concurrent.futures import Future
from types import SimpleNamespace
import threading
import time

import pytest
from geometry_msgs.msg import Twist
from robot_web.console import RobotConsole
from robot_web.control import ControlState
from robot_web.navigation_status import NavigationStatus


def pending_client():
    requests = []
    def call(request):
        future = Future()
        requests.append(future)
        return future
    return SimpleNamespace(service_is_ready=lambda: True, call_async=call, requests=requests)


def console_stub():
    node = RobotConsole.__new__(RobotConsole)
    node.data_lock = threading.RLock()
    node.motion_generation = 0
    node.cancel_building = False
    node.cancel_started = None
    node.canceling_goals = set()
    node.cancel_requests = []
    node.pending_goal = node.active_goal = None
    node.navigation_status = NavigationStatus()
    node.last_navigation_event = None
    node.navigation_event_pub = SimpleNamespace(publish=lambda msg: None)
    node.control = ControlState()
    node.control.set_mode('navigation')
    node.task = ''
    node.task_paused = False
    node.saving_map = False
    node.require_localized = lambda: None
    node.localization_status = lambda: {'ready': True}
    node.settings = {'nav_command_timeout': 1, 'navigation_cancel_timeout_seconds': 2}
    node.output = SimpleNamespace(publish=lambda msg: outputs.append(msg))
    node.chassis_output = node.output
    node.stop_state_pub = SimpleNamespace(publish=lambda msg: None)
    outputs = []
    node.outputs = outputs
    node.nav_cancel_client = pending_client()
    node.wheel_cancel_client = node.mission_stop_client = node.wheel_stop_client = SimpleNamespace(
        service_is_ready=lambda: False)
    return node


class Handle:
    accepted = True
    def __init__(self):
        self.ack = Future()
        self.result = Future()
        self.cancels = 0
    def cancel_goal_async(self):
        self.cancels += 1
        return self.ack
    def get_result_async(self):
        return self.result


def test_cancel_waits_for_terminal_and_mode_switch_never_resumes_old_velocity():
    node = console_stub()
    handle = Handle()
    node.active_goal = handle
    node.navigation_status.start('old', {'yaw': 0})
    handle.result.add_done_callback(lambda result: node.on_goal_result(result, handle, 'old'))
    node.cancel_motion()
    assert handle.cancels == 1
    assert node.navigation_status.snapshot()['phase'] == 'canceling'
    handle.ack.set_result(SimpleNamespace(return_code=0))
    node.nav_cancel_client.requests[0].set_result(SimpleNamespace(return_code=0))
    assert node.navigation_status.snapshot()['phase'] == 'canceling'
    node.control.set_mode('navigation')
    node.nav_stamp = node.filtered_nav_stamp = time.monotonic()
    node.last_nav = node.filtered_nav = Twist()
    node.last_nav.linear.x = 0.2  # 模拟 Nav2 尚未结束时仍在发送旧速度。
    node.control_loop()
    assert all(msg.linear.x == 0 for msg in node.outputs)
    handle.result.set_result(SimpleNamespace(status=5))
    assert node.navigation_status.snapshot()['phase'] == 'canceled'
    assert not node.canceling_goals


def test_goal_accepted_after_cancel_is_canceled_and_blocks_new_navigation():
    node = console_stub()
    sent = Future()
    node.pending_goal = sent
    node.navigation_status.start('old', {'yaw': 0})
    sent.add_done_callback(lambda future: node._on_goal_sent(future, 'old'))
    node.cancel_motion()
    node.nav_cancel_client.requests[0].set_result(SimpleNamespace(return_code=0))
    assert node.navigation_status.snapshot()['phase'] == 'canceling'
    handle = Handle()
    sent.set_result(handle)
    assert handle.cancels == 1
    node.preview = {'id': 'new', 'created': time.monotonic()}
    node.navigator = SimpleNamespace(wait_for_server=lambda **kwargs: True)
    with pytest.raises(ValueError, match='尚未结束'):
        node.navigate('new')
    handle.ack.set_result(SimpleNamespace(return_code=0))
    handle.result.set_result(SimpleNamespace(status=5))
    assert node.navigation_status.snapshot()['phase'] == 'canceled'


def test_cancel_rejection_is_visible_and_keeps_velocity_blocked():
    node = console_stub()
    handle = Handle()
    node.active_goal = handle
    node.navigation_status.start('old', {'yaw': 0})
    node.cancel_motion()
    handle.ack.set_result(SimpleNamespace(return_code=1))
    assert node.navigation_status.snapshot()['phase'] == 'failed'
    assert '拒绝取消' in node.navigation_status.snapshot()['message']
    assert node.active_goal is None
    assert node.canceling_goals


def test_real_ros_action_cancel_reaches_server_and_waits_for_result():
    """独立通信域中的无运动 Action 服务，验证取消确实到达服务端。"""
    import rclpy
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    server_node = rclpy.create_node('cancel_test_server')
    client_node = rclpy.create_node('cancel_test_client')
    observed = threading.Event()
    def execute(handle):
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            if handle.is_cancel_requested:
                observed.set()
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        handle.abort()
        return NavigateToPose.Result()
    server = ActionServer(server_node, NavigateToPose, '/test_cancel_navigation',
                          execute, cancel_callback=lambda request: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    client = ActionClient(client_node, NavigateToPose, '/test_cancel_navigation')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        node = console_stub()
        node.navigator = client
        node._pose = lambda *args: PoseStamped()
        node.preview = {'id': 'test', 'x': 0, 'y': 0, 'yaw': 0, 'created': time.monotonic()}
        assert client.wait_for_server(timeout_sec=4)
        assert node.navigate('test')['accepted']
        node.preview = {'id': 'replacement', 'x': 1, 'y': 0, 'yaw': 0, 'created': time.monotonic()}
        assert node.navigate('replacement')['accepted']
        assert observed.is_set()  # 替换目标也必须先结束旧目标。
        observed.clear()
        node.cancel_motion()
        # stub 的全量取消请求不参与本次真实 Action 验证。
        node.nav_cancel_client.requests[0].set_result(SimpleNamespace(return_code=0))
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and node.navigation_status.snapshot()['phase'] == 'canceling':
            time.sleep(0.02)
        assert observed.is_set()
        assert node.navigation_status.snapshot()['phase'] == 'canceled'
        assert node.active_goal is None
        assert not node.canceling_goals
    finally:
        executor.shutdown(timeout_sec=8)
        thread.join(timeout=2)
        client.destroy()
        server.destroy()
        client_node.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()


def test_cancel_timeout_is_visible_without_releasing_old_goal_barrier():
    node = console_stub()
    node.active_goal = Handle()
    node.navigation_status.start('old', {'yaw': 0})
    node.cancel_motion()
    node.cancel_started = time.monotonic()-3
    node.control_loop()
    assert '超时' in node.navigation_status.snapshot()['message']
    assert node.canceling_goals
    assert node.active_goal is None


def test_real_empty_abort_collects_late_bt_child_reason():
    """复现空状态 6 结果先到、BT 子节点 WARN 稍后到达的真实 ROS 通信顺序。"""
    import rclpy
    from rclpy.action import ActionClient, ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from rcl_interfaces.msg import Log

    rclpy.init()
    server_node = rclpy.create_node('failure_reason_test_server')
    client_node = rclpy.create_node('failure_reason_test_client')
    node = console_stub()
    node.logs, node.log_sequence = [], 0
    publisher = server_node.create_publisher(Log, '/test_nav_rosout', 10)
    subscriber = client_node.create_subscription(Log, '/test_nav_rosout', node.on_log, 10)
    timers = []
    def emit_reason():
        publisher.publish(Log(stamp=server_node.get_clock().now().to_msg(),
                              name='bt_navigator_navigate_to_pose_rclcpp_node', level=30,
                              msg='Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose'))
    def execute(handle):
        handle.abort()
        timer = threading.Timer(0.1, emit_reason)
        timers.append(timer)
        timer.start()
        return NavigateToPose.Result()
    server = ActionServer(server_node, NavigateToPose, '/test_failed_navigation', execute,
                          callback_group=ReentrantCallbackGroup())
    client = ActionClient(client_node, NavigateToPose, '/test_failed_navigation')
    node.navigator = client
    node._pose = lambda *args: PoseStamped()
    node.preview = {'id': 'test', 'x': 0, 'y': 0, 'yaw': 0, 'created': time.monotonic()}
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=4)
        deadline = time.monotonic()+3
        while not publisher.get_subscription_count() and time.monotonic() < deadline:
            time.sleep(0.01)
        node.navigate('test')
        deadline = time.monotonic()+3
        while time.monotonic() < deadline and '重新规划请求确认超时' not in node.navigation_status.snapshot()['message']:
            time.sleep(0.02)
        result = node.navigation_status.snapshot()
        assert result['phase'] == 'failed'
        assert result['error_code'] == 0
        assert '重新规划请求确认超时' in result['message']
        assert 'compute_path_to_pose' in result['message']
    finally:
        for timer in timers:
            timer.join(timeout=1)
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=2)
        client.destroy()
        server.destroy()
        client_node.destroy_node()
        server_node.destroy_node()
        rclpy.shutdown()

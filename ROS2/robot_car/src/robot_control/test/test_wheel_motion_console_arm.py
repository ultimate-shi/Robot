"""作用：验证整车停止锁阻止定距 Action 新目标；使用方法：pytest -q src/robot_control/test/test_wheel_motion_console_arm.py。"""
import time
from types import SimpleNamespace

from rclpy.action import GoalResponse

from robot_control.wheel_motion_server import WheelMotionServer


def test_wheel_motion_rejects_when_console_stopped_or_heartbeat_stale():
    server = WheelMotionServer.__new__(WheelMotionServer)
    server.require_console_arm = True
    server.console_arm_timeout = 0.5
    server.console_stamp = None
    server.console_stopped = True
    server.goal_active = False
    server.pending_t_future = None
    server.pending_stop_future = None
    server.steering_limit = 1.57
    goal = SimpleNamespace(steering_angle_rad=[0.0] * 4,
                           distance_m=[0.1] * 4)
    assert server._goal_callback(goal) == GoalResponse.REJECT
    server.console_stopped = False
    server.console_stamp = time.monotonic()
    assert server._goal_callback(goal) == GoalResponse.ACCEPT
    server.console_stamp -= 1
    assert server._goal_callback(goal) == GoalResponse.REJECT
    server.console_stamp = time.monotonic()
    server.console_stopped = True
    assert server._goal_callback(goal) == GoalResponse.REJECT

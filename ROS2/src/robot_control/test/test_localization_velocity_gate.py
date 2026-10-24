"""作用：验证导航速度门控对定位许可丢失和断流的处理；使用方法：pytest -q 本文件。"""

from types import SimpleNamespace
from geometry_msgs.msg import Twist
from robot_control.nav_velocity_gate import NavControllerNode


def test_localization_missing_stale_and_false_never_forward_velocity():
    gate = NavControllerNode.__new__(NavControllerNode)
    gate.require_localization = True
    gate.localization_timeout = 1.0
    gate.require_active_nav_goal = False
    gate.cmd_vel_timeout = 0.5
    gate.nav_cmd_active_threshold = 0.02
    gate.last_nav_cmd_time = 0.0
    gate.latest_nav_cmd = Twist()
    gate.latest_nav_cmd.linear.x = 0.1
    outputs, blocks = [], []
    gate.cmd_pub = SimpleNamespace(publish=outputs.append)
    gate.publish_zero = lambda state, reason: blocks.append(state)
    gate.publish_status = lambda *args: None
    gate.age = lambda stamp: stamp
    for ready, stamp in [(False, None), (False, 0.0), (True, 2.0)]:
        gate.localization_ready = ready
        gate.localization_stamp = stamp
        gate.control_loop()
        assert not outputs
        assert blocks[-1] == 'LOCALIZATION_BLOCKED'
    gate.localization_ready, gate.localization_stamp = True, 0.0
    gate.control_loop()
    assert outputs[0].linear.x == 0.1

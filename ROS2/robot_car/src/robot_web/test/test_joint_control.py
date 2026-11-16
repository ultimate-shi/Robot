"""作用：验证网页关节互斥、超时、故障与停止；用法：在 Jazzy 环境 pytest -q 本文件。"""
from copy import deepcopy
from types import SimpleNamespace as NS
import threading
import time

import pytest
from robot_web.control import ControlState
from robot_interfaces.msg import BodyBalanceStatus
from robot_web.joint_control import JointControl, JOINT_DEFAULTS
from robot_web.console import JointBody, JointMotionBody, BalanceBody


def fixture():
    c = deepcopy(JOINT_DEFAULTS)
    c.update(mode='virtual')
    node = NS(settings=c, data_lock=threading.RLock(), control=ControlState(),
              saving_map=False, motion_generation=0, hardware=None, controllers=({}, time.monotonic()))
    node.get_node_names_and_namespaces = lambda: []
    def publisher(*_):
        p = NS(messages=[])
        p.publish = p.messages.append
        return p
    node.create_publisher = publisher
    node.create_subscription = lambda *_: None
    future = NS(done=lambda: False)
    node.create_client = lambda *_: NS(service_is_ready=lambda: True, call_async=lambda _: future)
    joint = JointControl(node)
    joint.feedback = {name: (0, time.monotonic()) for name in c['leg_joint_names']+c['head_joint_names']}
    node.controllers = ({topic.split('/')[1]: 'active' for topic in
                         c['leg_command_topics']+[c['head_command_topic']]}, time.monotonic())
    joint.balance = (BodyBalanceStatus(), time.monotonic())
    return node, joint


def motion(sequence=1, **kw):
    return JointMotionBody(client_id='test', sequence=sequence, **kw)


def test_pending_balance_stale_feedback_and_stop_block_commands():
    node, j = fixture()
    body = JointBody(group='leg', values=[1]*4)
    j.command(body)
    j.pending = (None, time.monotonic(), False)
    with pytest.raises(ValueError, match='确认'):
        j.command(body)
    j.pending = None
    j.balance[0].enabled = True
    with pytest.raises(ValueError, match='关闭自动平衡'):
        j.command(body)
    j.balance[0].enabled = False
    j.feedback[j.cfg['leg_joint_names'][0]] = (0, time.monotonic()-5)
    with pytest.raises(ValueError, match='过期'):
        j.command(body)
    node.control.stop()
    with pytest.raises(ValueError, match='停止'):
        j.command(body)


def test_head_lock_node_blocks_even_without_status_message():
    node, j = fixture()
    node.get_node_names_and_namespaces = lambda: [('head_mapping_lock', '/')]
    with pytest.raises(ValueError, match='归中'):
        j.command(JointBody(group='head', values=[0, 0]))


def test_motion_sequence_release_and_timeout_hold_feedback():
    node, j = fixture()
    j.motion(motion(pitch=1))
    j.last_tick -= 0.1
    j.tick()
    assert len(j.publishers[2].messages) == 1
    assert j.publishers[2].messages[-1].data[0] < 0
    with pytest.raises(ValueError, match='乱序'):
        j.motion(motion())
    j.motion(motion(sequence=2))
    assert list(j.publishers[2].messages[-1].data) == [0]
    j.motion(motion(sequence=3, pitch=1, yaw=1))
    j.last_tick -= 0.1
    j.tick()
    j.intent_stamp -= 1
    j.tick()
    assert not j.active
    assert not j.intent
    assert list(j.head_pub.messages[-1].data) == [0, 0]


def test_head_i_command_raises_head_with_negative_pitch_axis():
    _, j = fixture()
    j.motion(motion(head_pitch=1))
    j.last_tick -= 0.1
    j.tick()
    assert j.head_pub.messages[-1].data[1] < 0


def test_invalid_targets_and_inactive_controller_never_publish():
    node, j = fixture()
    for values in ([0, 0, 0, 60], [0, float('nan'), 0, 0]):
        with pytest.raises(ValueError):
            j.command(JointBody(group='leg', values=values))
    node.controllers[0]['head_controller'] = 'inactive'
    with pytest.raises(ValueError, match='控制器'):
        j.motion(motion(yaw=1))
    assert not j.head_pub.messages


def test_stop_holds_all_six_joints_and_does_not_resume():
    node, j = fixture()
    j.motion(motion(pitch=1, yaw=1))
    j.last_tick -= 0.1
    j.tick()
    node.control.stop()
    j.emergency()
    node.control.reset_stop()
    j.tick()
    assert not j.active and not j.intent
    assert all(list(pub.messages[-1].data) == [0] for pub in j.publishers)
    assert list(j.head_pub.messages[-1].data) == [0, 0]


def test_failed_balance_response_does_not_unlock_on_old_status():
    _, j = fixture()
    j.set_balance(BalanceBody(enabled=True))
    future = NS(done=lambda: True, result=lambda: NS(success=False, message='拒绝'))
    j.pending = (future, time.monotonic(), True)
    j.tick()
    assert j.balance is None
    assert j.balance_error == '拒绝'
    assert '未知' in j.reason('leg')


def test_real_hardware_fault_blocks_even_when_joint_stream_is_recent():
    node, j = fixture()
    j.cfg['mode'] = 'real'
    node.hardware = (NS(outputs_enabled=False, manual_recovery_required=True, fault='通信中断'), time.monotonic())
    with pytest.raises(ValueError, match='故障'):
        j.command(JointBody(group='head', values=[0, 0]))
    status = j.status()
    assert not any(row['fresh'] for row in status['joints'])


def test_console_initializes_with_yaml_parameters_in_isolated_ros_domain():
    """真实构造节点发现参数类型/ROS 接口问题，不启动执行器，不发布运动。"""
    import rclpy
    from pathlib import Path
    import yaml
    from robot_web.console import RobotConsole
    config = yaml.safe_load((Path(__file__).parents[1]/'config/web_console.yaml').read_text())['web_console']['ros__parameters']
    rclpy.init()
    node = None
    try:
        # 默认虚拟模式，无相机硬件或关节命令发布；只测试初始化。
        # YAML 声明类型由 rclpy 的参数文件解析负责；字面量在此校验覆盖集合。
        node = RobotConsole()
        assert set(JOINT_DEFAULTS) <= set(config)
        assert node.joint_control.status()['balance']['enabled'] is None
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_space_hold_cancels_unfinished_slider_target():
    _, j = fixture()
    j.command(JointBody(group='leg', values=[20]*4))
    j.last_tick -= 0.1
    j.tick()
    j.motion(motion(hold=True))
    assert not j.active
    assert all(list(pub.messages[-1].data) == [0] for pub in j.publishers)


def test_old_generation_cannot_restart_motion_after_stop_or_mode_change():
    node, j = fixture()
    node.motion_generation += 1
    with pytest.raises(ValueError, match='模式已更新'):
        j.motion(motion(pitch=1))
    # 即使旧页面的停止请求也应允许停车。
    j.motion(motion(sequence=2, hold=True))
    assert not j.active


def test_head_slider_does_not_cancel_continuous_body_motion():
    _, j = fixture()
    j.motion(motion(pitch=1))
    j.last_tick -= 0.1
    j.tick()
    j.command(JointBody(group='head', values=[10, 0]))
    assert j.intent['pitch'] == 1
    assert 'leg' in j.active


@pytest.mark.parametrize('board_angle', [15.0, -15.0])
def test_board_angle_target_is_published_directly_with_real_r_command_signs(board_angle):
    """跨层核对：网页 ±15° 经 ROS 映射后，实际 R 字段仍是 ±1500。"""
    import math
    import re
    from pathlib import Path
    _, j = fixture()
    cpp = (Path(__file__).parents[2]/'robot_hardware/src/robot_serial_system.cpp').read_text()
    writer = cpp[cpp.index('bool RobotSerialSystem::write_legs()'):]
    signs = [float(v) for v in re.search(r'sign = \{([^}]+)\}', writer).group(1).split(',')]
    assert signs == j.cfg['leg_hardware_sign']
    j.command(JointBody(group='leg', unit='hardware_deg', values=[board_angle]*4))
    assert len(j.publishers[0].messages) == 1
    for pub, sign in zip(j.publishers, signs):
        assert round(math.degrees(pub.messages[-1].data[0])*sign*100) == int(board_angle*100)
    j.last_tick -= 0.1
    j.tick()
    # 不生成额外渐进目标或把 15° 当成 15mm，等待板端到位。
    assert all(len(pub.messages) == 1 for pub in j.publishers)
    status = j.status()
    assert [row['hardware_target_deg'] for row in status['joints'][:4]] == [board_angle]*4
    assert [row['extension_direction'] for row in status['joints'][:4]] == [-1, 1, 1, -1]
    j.motion(motion(hold=True))
    assert not j.direct_groups
    assert all(list(pub.messages[-1].data) == [0] for pub in j.publishers)


def test_board_angles_reject_out_of_range_and_preserve_other_leg_targets():
    _, j = fixture()
    with pytest.raises(ValueError, match='超出限位'):
        j.command(JointBody(group='leg', unit='hardware_deg', values=[31, 0, 0, 0]))
    j.command(JointBody(group='leg', unit='hardware_deg', values=[15, -5, 10, -10]))
    j.command(JointBody(group='leg', unit='hardware_deg', values=[15, 8, 10, -10]))
    assert [row['hardware_target_deg'] for row in j.status()['joints'][:4]] == [15, 8, 10, -10]


def test_repeated_leg_steps_send_5_10_15_20_without_intermediate_hold():
    import math
    from robot_web.console import JointStepBody
    _, j = fixture()
    for sequence in range(1,5):
        j.step(JointStepBody(client_id='steps',sequence=sequence,action='e'))
    # 左前伸展对应负板端角度，右前为正板端角度。
    sent=[round(math.degrees(msg.data[0])) for msg in j.publishers[0].messages]
    assert sent == [5,10,15,20]
    raw=[round(math.degrees(msg.data[0])) for msg in j.publishers[1].messages]
    assert raw == [5,10,15,20]
    assert len(j.publishers[0].messages) == 4
    with pytest.raises(ValueError,match='乱序'):
        j.step(JointStepBody(client_id='steps',sequence=4,action='e'))


def test_four_leg_zero_submits_all_zero_targets_once_without_touching_head():
    from robot_web.console import JointZeroBody
    node, j = fixture()
    for name in j.cfg['leg_joint_names']:
        j.feedback[name] = (15, time.monotonic())
    j.zero(JointZeroBody(group='leg',client_id='zero',sequence=2))
    assert j.targets['leg'] == [0.0]*4
    assert j.targets['head'] is None
    assert not j.head_pub.messages
    assert all(len(pub.messages) == 1 and list(pub.messages[0].data) == [0] for pub in j.publishers)
    j.last_tick -= .1
    j.tick()
    assert all(len(pub.messages) == 1 for pub in j.publishers)
    with pytest.raises(ValueError,match='乱序'):
        j.motion(JointMotionBody(client_id='zero',sequence=1,hold=True))
    assert j.targets['leg'] == [0]*4


def test_head_zero_submits_both_axes_once_and_preserves_leg_motion():
    from robot_web.console import JointZeroBody
    _, j = fixture()
    j.intent = {'pitch': 1}
    for name in j.cfg['head_joint_names']:
        j.feedback[name] = (12, time.monotonic())
    j.zero(JointZeroBody(group='head',client_id='zero',sequence=1))
    assert list(j.head_pub.messages[0].data) == [0,0]
    assert len(j.head_pub.messages) == 1
    assert all(not pub.messages for pub in j.publishers)
    assert j.intent['pitch'] == 1
    assert j.targets['leg'] is None


def test_zero_checks_all_controllers_before_publishing_any_target():
    from robot_web.console import JointZeroBody
    node, j = fixture()
    node.controllers[0]['lap_rr_position_controller'] = 'inactive'
    with pytest.raises(ValueError,match='控制器'):
        j.zero(JointZeroBody(group='leg',client_id='zero',sequence=1))
    assert all(not pub.messages for pub in j.publishers)
    node.control.stop()
    with pytest.raises(ValueError,match='停止'):
        j.zero(JointZeroBody(group='head',client_id='zero',sequence=2))
    assert not j.head_pub.messages

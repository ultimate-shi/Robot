"""作用：验证双白名单、事件去重、导航采样与索引滚动保留；使用方法：Jazzy 环境运行 pytest -q 本文件。"""
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

from mcap.writer import Writer
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from robot_interfaces.msg import ChassisState
from nav2_msgs.action import NavigateToPose
import pytest
import yaml

from robot_logging.indexer import connect, index_file, scan, trim_index
from robot_logging.manager import LogManager
from robot_logging.storage import ensure_layout, finish_session, load_config, new_session, prune
from robot_logging.topic_policy import IndexSelector, TopicPolicy


CONFIG = Path(__file__).resolve().parents[1] / 'config'


def write_bag(path, messages):
    with path.open('wb') as stream:
        writer = Writer(stream)
        writer.start()
        schemas, channels = {}, {}
        for topic, msg, stamp in messages:
            cls = type(msg)
            namespace = cls.__module__.split('.')[1]
            name = cls.__module__.split('.')[0]+'/'+namespace+'/'+cls.__name__
            if name not in schemas:
                schemas[name] = writer.register_schema(name, 'ros2msg', b'')
            if topic not in channels:
                channels[topic] = writer.register_channel(topic, 'cdr', schemas[name])
            writer.add_message(channels[topic], stamp, serialize_message(msg), stamp)
        writer.finish()


def test_two_independent_configs_and_recording_commands(tmp_path):
    recording = TopicPolicy(CONFIG/'recording_topics.yaml')
    indexing = TopicPolicy(CONFIG/'index_topics.yaml')
    assert set(recording.config['categories']) == {'常用', '按需', '不建议'}
    assert recording.match('/odom') and indexing.match('/odom') is None
    assert recording.match('/cmd_vel_nav_raw')
    assert recording.match('/hardware/chassis_state')
    assert recording.match('/navigate_to_pose/_action/status')
    assert not recording.match('/bond')
    assert not recording.match('/local_costmap/costmap')
    assert not recording.match('/odom_sensor_data/raw')
    node = LogManager.__new__(LogManager)
    node.config = load_config(CONFIG/'logging.yaml')
    node.recording_policy = recording
    node.mcap_config = str(CONFIG/'mcap.yaml')
    command = node._command(tmp_path, False)
    assert '--all' not in command and '--regex' in command
    assert '--include-hidden-topics' in command
    assert '--all' not in node._command(tmp_path, True)
    recording.config['incident_record_all'] = True
    assert '--all' in node._command(tmp_path, True)
    recording.config['categories']['按需'][0]['enabled'] = True
    alternate = tmp_path/'recording.yaml'
    alternate.write_text(yaml.safe_dump(recording.config, allow_unicode=True))
    assert TopicPolicy(alternate).match('/scan')
    assert indexing.match('/scan') is None  # 录制开关不会连带开启索引。


def test_events_sample_feedback_and_raw_file_unchanged(tmp_path):
    policy = TopicPolicy(CONFIG/'index_topics.yaml')
    bag = tmp_path/'events.mcap'
    messages = []
    for index in range(100):
        stamp = 10_000_000_000 + index*100_000_000
        for topic in ('/odom', '/tf', '/bond', '/scan', '/sensors/imu/data'):
            messages.append((topic, String(data='不应进入索引'), stamp))
        state = {'state': 'localized', 'ready': True, 'input_ready': True,
                 'elapsed_seconds': index}
        if index >= 50:
            state.update(state='tracking', match_age_seconds=index/10)
        messages.append(('/localization/status', String(data=json.dumps(state)), stamp))
        chassis = ChassisState(dmc0_connected=True, dmc1_connected=True, se2_connected=True,
                               outputs_enabled=index < 80, fault='通信断开' if index >= 80 else '')
        chassis.header.stamp.nanosec = index
        chassis.wheel_current_ma = [float(index)]*4
        messages.append(('/hardware/chassis_state', chassis, stamp))
        feedback = NavigateToPose.Impl.FeedbackMessage()
        feedback.feedback.distance_remaining = 10-index/10
        messages.append(('/navigate_to_pose/_action/feedback', feedback, stamp))
        messages.append(('/rosout', Log(name='controller_server', level=40,
                                      msg='Failed to make progress'), stamp))
        messages.append(('/rosout', Log(name='rosbag2_recorder', level=20,
                                      msg='Subscribed to topic'), stamp))
    write_bag(bag, messages)
    digest = hashlib.sha256(bag.read_bytes()).hexdigest()
    conn = connect(tmp_path/'logs.sqlite3')
    count = index_file(conn, bag, 2048, policy)
    assert count == 16  # 2 定位变化、2 硬件变化、10 反馈采样、2 重复日志窗口。
    assert conn.execute("select count(*) from messages where name='/odom'").fetchone()[0] == 0
    assert conn.execute("select count(*) from messages where name='/hardware/chassis_state'").fetchone()[0] == 2
    assert conn.execute("select count(*) from messages where payload like '%通信断开%'").fetchone()[0] == 1
    assert index_file(conn, bag, 2048, policy) == 0
    assert hashlib.sha256(bag.read_bytes()).hexdigest() == digest
    conn.close()


def test_diagnostics_retains_fault_and_recovery_not_healthy_heartbeat(tmp_path):
    policy = TopicPolicy(CONFIG/'index_topics.yaml')
    bag = tmp_path/'diagnostics.mcap'
    messages = []
    for index, level in enumerate((0, 0, 2, 2, 0, 0)):
        diagnostic = DiagnosticStatus(level=bytes([level]), name='robot_hardware', message='断开' if level else '正常')
        messages.append(('/diagnostics', DiagnosticArray(status=[diagnostic]), 1_000_000_000*(index+1)))
    write_bag(bag, messages)
    conn = connect(tmp_path/'logs.sqlite3')
    assert index_file(conn, bag, 2048, policy) == 2
    assert conn.execute("select count(*) from messages where payload like '%诊断恢复正常%'").fetchone()[0] == 1
    conn.close()


def test_policy_change_reindexes_file_and_index_retention_keeps_mcap(tmp_path):
    path = tmp_path/'policy.yaml'
    config = {'categories': {'常用': [{'topic': '/status', 'enabled': True, 'mode': 'all'}],
                              '按需': [], '不建议': []}}
    path.write_text(yaml.safe_dump(config, allow_unicode=True))
    bag = tmp_path/'small.mcap'
    write_bag(bag, [('/status', String(data='相同状态'), i*1_000_000_000) for i in range(1, 5)])
    conn = connect(tmp_path/'logs.sqlite3')
    assert index_file(conn, bag, 2048, TopicPolicy(path)) == 4
    config['categories']['常用'][0].update(mode='changes', heartbeat_seconds=60)
    path.write_text(yaml.safe_dump(config, allow_unicode=True))
    policy = TopicPolicy(path)
    assert index_file(conn, bag, 2048, policy) == 1
    trim_index(conn, {'index_max_bytes': 1_000_000, 'index_retention_days': 1})
    assert conn.execute('select count(*) from messages').fetchone()[0] == 0
    assert bag.is_file()
    assert index_file(conn, bag, 2048, policy) == 0  # 不反复重建已淘汰的旧索引行。
    conn.close()


def test_index_quota_keeps_raw_file_and_can_continue_new_messages(tmp_path):
    conn = connect(tmp_path/'logs.sqlite3')
    bag = tmp_path/'old.mcap'
    bag.write_bytes('原始录制'.encode())
    with conn:
        conn.executemany('''insert into messages(ts_ns,category,name,type_name,summary,payload,
            bag_file,ordinal,size_bytes) values (?,'topic','/rosout','Log','',?,?,?,2048)''',
            [(i, f'故障记录{i} '+str(i)*1000, str(bag), i) for i in range(2000)])
    trim_index(conn, {'index_max_bytes': 400_000})
    assert conn.execute('select count(*) from messages').fetchone()[0] < 2000
    assert bag.read_bytes() == '原始录制'.encode()
    new_bag = tmp_path/'new.mcap'
    write_bag(new_bag, [('/rosout', Log(name='controller_server', level=40, msg='新导航故障'), 3_000_000_000)])
    assert index_file(conn, new_bag, 2048, TopicPolicy(CONFIG/'index_topics.yaml')) == 1
    assert '新导航故障' in conn.execute('select payload from messages order by ts_ns desc limit 1').fetchone()[0]
    conn.close()


def test_closed_segment_is_searchable_while_session_still_recording(tmp_path):
    ensure_layout(tmp_path)
    session = new_session(tmp_path, 'routine', 'real')
    first, writing = session/'bag_0.mcap', session/'bag_1.mcap'
    write_bag(first, [('/rosout', Log(name='controller_server', level=40, msg='导航失败原因'), 1_000_000_000)])
    os.utime(first, (1, 1))
    writing.write_bytes('尚未写完'.encode())
    conn = connect(tmp_path/'index'/'logs.sqlite3')
    config = {'root': tmp_path, 'index_max_message_bytes': 2048, 'index_max_bytes': 1_000_000,
              '_index_policy': TopicPolicy(CONFIG/'index_topics.yaml')}
    scan(conn, config)
    assert conn.execute('select count(*) from messages').fetchone()[0] == 1
    assert (session/'.active').exists()
    assert writing.read_bytes() == '尚未写完'.encode()
    conn.close()


def test_prune_reserves_room_for_next_segment(tmp_path):
    ensure_layout(tmp_path)
    old = new_session(tmp_path, 'routine', 'real')
    (old/'data').write_bytes(b'x'*200)
    finish_session(old)
    config = {'root': tmp_path, 'retention_days': 7, 'max_total_bytes': 600, 'min_free_bytes': 0}
    prune(config, required_bytes=400)
    assert not old.exists()  # 当前未超上限，但仍须为下段录制腾出空间。


def test_invalid_or_empty_policy_never_records_everything(tmp_path):
    path = tmp_path/'empty.yaml'
    path.write_text(yaml.safe_dump({'categories': {category: [] for category in ('常用','按需','不建议')}}, allow_unicode=True))
    with pytest.raises(ValueError, match='至少'):
        TopicPolicy(path)


def test_corrupt_old_segment_does_not_stop_new_fault_indexing(tmp_path):
    ensure_layout(tmp_path)
    session = new_session(tmp_path, 'routine', 'real')
    finish_session(session)
    (session/'bad.mcap').write_bytes(b'incomplete recording')
    write_bag(session/'good.mcap', [('/rosout', Log(name='controller_server', level=40, msg='新错误'), 10_000_000_000)])
    conn = connect(tmp_path/'index'/'logs.sqlite3')
    scan(conn, {'root': tmp_path, 'index_max_message_bytes': 2048, 'index_max_bytes': 1_000_000,
                '_index_policy': TopicPolicy(CONFIG/'index_topics.yaml')})
    assert conn.execute('select count(*) from messages').fetchone()[0] == 1
    assert '新错误' in conn.execute('select payload from messages').fetchone()[0]
    conn.close()

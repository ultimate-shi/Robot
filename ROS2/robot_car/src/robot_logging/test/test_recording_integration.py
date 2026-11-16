"""作用：独立 DDS 域实测 rosbag2 白名单及精简索引；使用方法：RUN_LOGGING_RECORDING_INTEGRATION=1 ROS_DOMAIN_ID=194 pytest -q 本文件。"""
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace

from mcap.reader import make_reader
import pytest
import rclpy
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray, GoalStatus
from robot_interfaces.msg import ChassisState

from robot_logging.indexer import connect, index_file
from robot_logging.manager import LogManager
from robot_logging.storage import load_config
from robot_logging.topic_policy import TopicPolicy


@pytest.mark.skipif(os.environ.get('RUN_LOGGING_RECORDING_INTEGRATION') != '1',
                    reason='真实录制集成测试需独立 DDS 域显式启用')
def test_real_rosbag_regex_keeps_faults_and_hidden_actions_only(tmp_path):
    config_dir = Path(__file__).resolve().parents[1]/'config'
    manager = LogManager.__new__(LogManager)
    manager.config = load_config(config_dir/'logging.yaml')
    manager.recording_policy = TopicPolicy(config_dir/'recording_topics.yaml')
    manager.mcap_config = str(config_dir/'mcap.yaml')
    output = tmp_path/'session'
    output.mkdir()
    rclpy.init()
    node = rclpy.create_node('logging_selection_probe')
    chassis = node.create_publisher(ChassisState, '/hardware/chassis_state', 10)
    action = node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status', 10)
    event = node.create_publisher(String, '/web_console/navigation_status', 10)
    bond = node.create_publisher(String, '/bond', 10)
    image = node.create_publisher(String, '/stereo/image_raw', 10)
    console = (tmp_path/'recorder.log').open('wb')
    recorder = subprocess.Popen(manager._command(output, False), stdout=console, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if all(pub.get_subscription_count() for pub in (chassis, action, event)):
                break
            assert recorder.poll() is None
            time.sleep(0.1)
        assert all(pub.get_subscription_count() for pub in (chassis, action, event))
        for index in range(10):
            chassis.publish(ChassisState(fault='测试控制板断连'))
            action.publish(GoalStatusArray(status_list=[GoalStatus(status=6)]))
            event.publish(String(data='{"phase":"failed","message":"测试定位丢失","goal":{"x":1}}'))
            bond.publish(String(data='冗余心跳'))
            image.publish(String(data='图像占位'))
            time.sleep(0.1)
        assert bond.get_subscription_count() == image.get_subscription_count() == 0
    finally:
        recorder.send_signal(signal.SIGINT)
        recorder.wait(timeout=12)
        console.close()
        node.destroy_node()
        rclpy.shutdown()
    assert recorder.returncode == 0
    bags = list((output/'bag').glob('*.mcap'))
    assert bags
    with bags[0].open('rb') as stream:
        topics = {channel.topic for _, channel, _ in make_reader(stream).iter_messages()}
    assert '/hardware/chassis_state' in topics
    assert '/navigate_to_pose/_action/status' in topics
    assert '/web_console/navigation_status' in topics
    assert '/bond' not in topics and '/stereo/image_raw' not in topics
    conn = connect(tmp_path/'logs.sqlite3')
    count = index_file(conn, bags[0], 2048, TopicPolicy(config_dir/'index_topics.yaml'))
    assert count >= 3
    rows = conn.execute('select payload from messages').fetchall()
    assert any('测试控制板断连' in row[0] for row in rows)
    assert any('测试定位丢失' in row[0] for row in rows)
    assert conn.execute("select count(*) from messages where category='action'").fetchone()[0] == 1
    conn.close()

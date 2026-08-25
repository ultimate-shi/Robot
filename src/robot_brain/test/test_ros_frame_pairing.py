"""使用方法：在 ROS 2 Jazzy 环境运行，验证乱序消息仍能精确配对。"""

import pytest
import rclpy
from robot_interfaces.msg import SemanticDetection, SemanticDetectionArray
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from robot_brain.ros_bridge import RosBridge, SharedRobotState


@pytest.fixture
def bridge_state():
    if not rclpy.ok():
        rclpy.init()
    shared = SharedRobotState()
    bridge = RosBridge(shared)
    yield bridge, shared
    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def messages(stamp=123):
    frame = CompressedImage()
    frame.header.stamp.sec = stamp // 1_000_000_000
    frame.header.stamp.nanosec = stamp % 1_000_000_000
    frame.data = [1, 2, 3]
    detections = SemanticDetectionArray()
    detections.header.stamp = frame.header.stamp
    detections.image_width = 640
    detections.image_height = 480
    detections.model = 'test-yolo'
    return frame, detections


def test_frame_then_detection_pairs_exactly(bridge_state):
    bridge, shared = bridge_state
    frame, detections = messages()
    bridge._frame_callback(frame)
    bridge._detections_callback(detections)
    assert shared.detection_frame == b'\x01\x02\x03'
    assert shared.detection_frame_stamp_ns == 123


def test_detection_then_frame_also_pairs_exactly(bridge_state):
    bridge, shared = bridge_state
    frame, detections = messages()
    bridge._detections_callback(detections)
    assert shared.detection_frame is None
    bridge._frame_callback(frame)
    assert shared.detection_frame == b'\x01\x02\x03'
    assert shared.detection_frame_stamp_ns == 123


def test_npu_pause_keeps_last_real_detection(bridge_state):
    bridge, shared = bridge_state
    _, detections = messages()
    person = SemanticDetection()
    person.id = 'person-1'
    person.class_name = 'person'
    person.label_zh = '人'
    detections.detections.append(person)
    bridge._detections_callback(detections)

    bridge._semantic_status_callback(String(data=(
        '{"state":"paused","reason_code":"NPU_BUSY_LLM",'
        '"message":"Qwen 正在占用 NPU"}')))

    assert shared.detections['detections'][0]['id'] == 'person-1'
    assert shared.health['semantic']['state'] == 'paused'
    assert shared.perception_status['reason_code'] == 'NPU_BUSY_LLM'


def test_wait_for_new_scene_ignores_same_timestamp(bridge_state):
    bridge, shared = bridge_state
    _, first = messages(123)
    bridge._detections_callback(first)
    assert shared.wait_for_detection_after(123, 0.01) is None

    _, second = messages(456)
    bridge._detections_callback(second)
    snapshot = shared.wait_for_detection_after(123, 0.01)
    assert snapshot.detection_stamp_ns == 456

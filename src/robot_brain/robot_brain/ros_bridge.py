"""
使用方法：web_server 创建 RosBridge。

只有本模块能够调用机器人 ROS Service 和 Action。
"""

from copy import deepcopy
import json
import threading
import time

from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from robot_interfaces.action import PlanMission
from robot_interfaces.msg import MissionState, SemanticDetectionArray
from robot_interfaces.srv import CaptureSample, ConfirmMission, DetectObjects
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_brain.frame_cache import TimestampedFrameCache
from robot_brain.scene_coordinator import SceneCoordinator


class SharedRobotState:
    """线程安全地缓存网页需要的低带宽共享状态。"""

    def __init__(self):
        self.lock = threading.RLock()
        self.detections = {'state': 'waiting', 'detections': []}
        self.perception_status = {
            'state': 'waiting', 'reason_code': '', 'message': ''}
        self.mission = {'state': 'stopped', 'message': '等待任务'}
        self.health = {
            'camera': {'state': 'waiting'}, 'slam': {'state': 'waiting'},
            'semantic': {'state': 'waiting'}, 'navigation': {'state': 'waiting'},
            'qwen': {'state': 'waiting'},
        }
        self.frame = None
        # detection_frame 只供聊天审计使用，不能进入 WebSocket JSON 状态。
        self.detection_frame = None
        self.detection_frame_stamp_ns = None
        self.map = None
        self.scene_coordinator = SceneCoordinator(self.lock)

    def snapshot(self):
        with self.lock:
            return {
                'detections': deepcopy(self.detections),
                'perception_status': deepcopy(self.perception_status),
                'mission': deepcopy(self.mission),
                'health': deepcopy(self.health),
                'server_time': time.time(),
            }

    def freeze_scene(self):
        """冻结当前场景；图片字节仍由调用者在同一把锁内读取。"""
        with self.lock:
            # coordinator 已与每次检测回调同步，图像后到时也会补齐时间戳。
            return self.scene_coordinator.snapshot()

    def wait_for_detection_after(self, stamp_ns, timeout):
        """等待一次真正完成且时间戳变化的 YOLO 场景，暂停状态不算新结果。"""
        return self.scene_coordinator.wait_after(stamp_ns, timeout)


class RosBridge(Node):
    """把网页请求转换为固定 ROS 接口，不接受任意 Topic 或坐标调用。"""

    MISSION_TYPES = {
        'goto_object': PlanMission.Goal.GOTO_OBJECT,
        'follow_person': PlanMission.Goal.FOLLOW_PERSON,
        'explore': PlanMission.Goal.EXPLORE,
    }

    def __init__(self, shared):
        super().__init__('brain_ros_bridge')
        self.declare_parameter(
            'frame_topic', '/stereo/right/image_rect/compressed')
        self.declare_parameter('frame_cache_size', 30)
        self.declare_parameter('frame_cache_age_seconds', 3.0)
        self.shared = shared
        self.frame_cache = TimestampedFrameCache(
            self.get_parameter('frame_cache_size').value,
            self.get_parameter('frame_cache_age_seconds').value)
        self.notify = None
        self.plan_client = ActionClient(self, PlanMission, '/mission/plan')
        self.confirm_client = self.create_client(ConfirmMission, '/mission/confirm')
        self.cancel_client = self.create_client(Trigger, '/mission/cancel')
        self.stop_client = self.create_client(Trigger, '/mission/stop')
        self.detect_client = self.create_client(DetectObjects, '/perception/detect_objects')
        self.capture_client = self.create_client(CaptureSample, '/perception/capture_sample')
        self.create_subscription(
            SemanticDetectionArray, '/perception/semantic_detections',
            self._detections_callback, 10)
        self.create_subscription(
            String, '/perception/semantic_status',
            self._semantic_status_callback, 10)
        self.create_subscription(
            MissionState, '/mission/state', self._mission_callback, 20)
        self.create_subscription(
            CompressedImage, str(self.get_parameter('frame_topic').value),
            self._frame_callback, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

    def send_plan(self, mission_id, task, target_id='', label=''):
        if not self.plan_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError('任务规划 Action 尚未就绪')
        goal = PlanMission.Goal()
        goal.mission_id = mission_id
        goal.mission_type = self.MISSION_TYPES[task]
        goal.target_id = target_id
        goal.target_label = label
        return self.plan_client.send_goal_async(goal)

    def confirm(self, mission_id):
        request = ConfirmMission.Request()
        request.mission_id = mission_id
        return self.confirm_client.call_async(request)

    def cancel(self):
        return self.cancel_client.call_async(Trigger.Request())

    def stop(self):
        return self.stop_client.call_async(Trigger.Request())

    def detect(self, force_refresh=True):
        request = DetectObjects.Request()
        request.force_refresh = force_refresh
        request.min_confidence = 0.0
        return self.detect_client.call_async(request)

    def capture(self, request_id, scene_id):
        request = CaptureSample.Request()
        request.request_id = request_id
        request.scene_id = scene_id
        return self.capture_client.call_async(request)

    def _detections_callback(self, msg):
        values = []
        for item in msg.detections:
            values.append({
                'id': item.id, 'class_name': item.class_name,
                'label_zh': item.label_zh, 'confidence': item.confidence,
                'bbox': [item.bbox_x_min, item.bbox_y_min,
                         item.bbox_x_max, item.bbox_y_max],
                'distance': item.distance if item.has_depth else None,
                'map_position': ({
                    'x': item.map_position.point.x,
                    'y': item.map_position.point.y,
                    'z': item.map_position.point.z,
                    'frame_id': item.map_position.header.frame_id,
                } if item.has_map_position else None),
            })
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        paired_frame = self.frame_cache.get(stamp_ns)
        with self.shared.lock:
            self.shared.detections = {
                'state': 'valid' if values else 'valid_empty',
                'model': msg.model, 'latency_ms': msg.latency_ms,
                'stamp_ns': stamp_ns,
                'image': {'width': msg.image_width, 'height': msg.image_height},
                'detections': values,
            }
            self.shared.detection_frame = paired_frame
            self.shared.detection_frame_stamp_ns = (
                stamp_ns if paired_frame is not None else None)
            self.shared.health['semantic'] = {
                'state': 'ok', 'count': len(values), 'model': msg.model}
            self.shared.perception_status = {
                'state': 'ok', 'reason_code': '',
                'message': f'识别到 {len(values)} 个目标'}
            self.shared.scene_coordinator.update_detection(
                self.shared.detections,
                self.shared.detection_frame_stamp_ns)
        self._changed()

    def _semantic_status_callback(self, msg):
        """暂停或错误只更新状态，不用空数组覆盖最后一次真实 YOLO 场景。"""
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            value = {'state': 'error', 'message': '语义状态不是有效 JSON'}
        state = str(value.get('state', 'error'))
        if state == 'ok':
            return
        with self.shared.lock:
            self.shared.perception_status = {
                'state': state,
                'reason_code': str(value.get('reason_code', '')),
                'message': str(value.get('message', '')),
            }
            previous = self.shared.health.get('semantic', {})
            self.shared.health['semantic'] = {
                **previous, 'state': state,
                'reason_code': str(value.get('reason_code', '')),
                'message': str(value.get('message', '')),
            }
            self.shared.scene_coordinator.wake()
        self._changed()

    def _mission_callback(self, msg):
        with self.shared.lock:
            self.shared.mission = {
                'mission_id': msg.mission_id, 'mission_type': msg.mission_type,
                'state': msg.phase, 'target_id': msg.target_id,
                'message': msg.message, 'motion_enabled': msg.motion_enabled,
            }
            self.shared.health['navigation'] = {
                'state': 'ok', 'message': msg.message}
        self._changed()

    def _frame_callback(self, msg):
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        frame = bytes(msg.data)
        self.frame_cache.add(stamp_ns, frame)
        with self.shared.lock:
            self.shared.frame = frame
            # 压缩图和检测回调可能乱序到达；后到的同时间戳帧补全配对。
            if self.shared.detections.get('stamp_ns') == stamp_ns:
                self.shared.detection_frame = frame
                self.shared.detection_frame_stamp_ns = stamp_ns
                self.shared.scene_coordinator.pair_image(stamp_ns)
            self.shared.health['camera'] = {'state': 'ok'}
        self._changed()

    def _map_callback(self, msg):
        stride = max(1, int(max(msg.info.width, msg.info.height) / 320))
        sampled = []
        values = list(msg.data)
        for row in range(0, msg.info.height, stride):
            start = row * msg.info.width
            sampled.extend(values[start:start + msg.info.width:stride])
        with self.shared.lock:
            self.shared.map = {
                'width': (msg.info.width + stride - 1) // stride,
                'height': (msg.info.height + stride - 1) // stride,
                'resolution': msg.info.resolution * stride,
                'origin': {'x': msg.info.origin.position.x,
                           'y': msg.info.origin.position.y},
                'data': sampled,
            }
            self.shared.health['slam'] = {
                'state': 'ok', 'size': [msg.info.width, msg.info.height]}
        self._changed()

    def _changed(self):
        if self.notify is not None:
            self.notify()

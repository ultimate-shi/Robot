#!/usr/bin/env python3
"""调用板端检测服务，把二维检测与双目深度融合成地图语义目标."""

import base64
from concurrent.futures import ThreadPoolExecutor
import json
import math
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class SemanticPerception(Node):
    """低频调用YOLO服务，高带宽图像仍留在本机ROS进程内."""

    def __init__(self):
        super().__init__('semantic_perception')
        defaults = {
            'image_topic': '/stereo/left/image_rect',
            'depth_topic': '/stereo/depth/image',
            'camera_info_topic': '/stereo/left/camera_info',
            'output_topic': '/perception/semantic_detections',
            'inference_url': 'http://127.0.0.1:9100/v1/detect',
            'map_frame': 'map',
            'max_inference_rate': 5.0,
            'request_timeout': 2.0,
            'jpeg_quality': 80,
            'min_confidence': 0.35,
            'depth_center_ratio': 0.4,
            'min_valid_depth_pixels': 12,
            'track_match_distance': 0.5,
            'track_timeout': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.inference_url = str(
            self.get_parameter('inference_url').value)
        if not self.inference_url.rstrip('/').endswith('/v1/detect'):
            self.inference_url = self.inference_url.rstrip('/') + '/v1/detect'
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.request_timeout = float(
            self.get_parameter('request_timeout').value)
        self.jpeg_quality = int(
            self.get_parameter('jpeg_quality').value)
        self.min_confidence = float(
            self.get_parameter('min_confidence').value)
        self.center_ratio = float(
            self.get_parameter('depth_center_ratio').value)
        self.min_depth_pixels = int(
            self.get_parameter('min_valid_depth_pixels').value)
        self.track_match_distance = float(
            self.get_parameter('track_match_distance').value)
        self.track_timeout = float(
            self.get_parameter('track_timeout').value)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_depth = None
        self.latest_info = None
        self.in_flight = False
        # Node.executor 是 rclpy 用于绑定 ROS 执行器的保留属性，线程池需使用独立名称。
        self.inference_pool = ThreadPoolExecutor(max_workers=1)
        self.track_counter = 0
        self.tracks = {}

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._image_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._info_callback, qos_profile_sensor_data)
        self.output_pub = self.create_publisher(
            String, str(self.get_parameter('output_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, '/perception/semantic_status', 10)
        rate = max(0.2, float(
            self.get_parameter('max_inference_rate').value))
        self.create_timer(1.0 / rate, self._schedule_inference)

    def _image_callback(self, msg):
        with self.lock:
            self.latest_image = msg

    def _depth_callback(self, msg):
        if msg.encoding != '32FC1':
            return
        with self.lock:
            self.latest_depth = msg

    def _info_callback(self, msg):
        with self.lock:
            self.latest_info = msg

    def _schedule_inference(self):
        with self.lock:
            if self.in_flight or self.latest_image is None:
                return
            self.in_flight = True
            image = self.latest_image
            depth = self.latest_depth
            info = self.latest_info
        future = self.inference_pool.submit(self._infer, image, depth, info)
        future.add_done_callback(self._inference_finished)

    def _infer(self, image_msg, depth_msg, info_msg):
        started = time.perf_counter()
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        success, encoded = cv2.imencode(
            '.jpg', image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not success:
            raise ValueError('左目图像 JPEG 编码失败')
        request_body = json.dumps({
            'image_base64': base64.b64encode(encoded).decode('ascii'),
            'min_confidence': self.min_confidence,
        }).encode('utf-8')
        request = Request(
            self.inference_url, data=request_body,
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            with urlopen(request, timeout=self.request_timeout) as response:
                payload = json.loads(response.read().decode('utf-8'))
        except (HTTPError, URLError, TimeoutError) as exc:
            raise RuntimeError(f'检测服务不可用: {exc}') from exc
        depth = self._depth_array(depth_msg)
        detections = []
        for raw in payload.get('detections', []):
            detection = self._project_detection(
                raw, image.shape[:2], depth, info_msg, image_msg)
            if detection is not None:
                detections.append(detection)
        return {
            'stamp': {
                'sec': image_msg.header.stamp.sec,
                'nanosec': image_msg.header.stamp.nanosec,
            },
            'image': {'width': image.shape[1], 'height': image.shape[0]},
            'model': payload.get('model', 'unknown'),
            'detections': detections,
            'latency_ms': round(
                (time.perf_counter() - started) * 1000.0, 1),
        }

    def _inference_finished(self, future):
        try:
            payload = future.result()
        except Exception as exc:
            self._publish_status('error', str(exc))
        else:
            self.output_pub.publish(String(data=json.dumps(
                payload, ensure_ascii=False)))
            self._publish_status(
                'ok', f"识别到 {len(payload['detections'])} 个目标",
                latency_ms=payload['latency_ms'], model=payload['model'])
        finally:
            with self.lock:
                self.in_flight = False

    @staticmethod
    def _depth_array(msg):
        if msg is None or msg.encoding != '32FC1' or msg.step < msg.width * 4:
            return None
        endian = '>' if msg.is_bigendian else '<'
        return np.ndarray(
            (msg.height, msg.width), dtype=np.dtype(endian + 'f4'),
            buffer=msg.data, strides=(msg.step, 4)).astype(
                np.float32, copy=False)

    def _project_detection(self, raw, image_shape, depth, info, image_msg):
        try:
            confidence = float(raw.get('confidence', 0.0))
            box = [float(value) for value in raw['bbox']]
        except (KeyError, TypeError, ValueError):
            return None
        if confidence < self.min_confidence or len(box) != 4:
            return None
        class_name = str(raw.get('class_name', 'unknown'))
        result = {
            'id': '',
            'class_name': class_name,
            'label_zh': str(raw.get('label_zh', class_name)),
            'confidence': confidence,
            'bbox': box,
            'distance': None,
            'map_position': None,
        }
        camera_point = self._camera_point(box, image_shape, depth, info)
        if camera_point is not None:
            result['distance'] = round(float(camera_point[2]), 3)
            result['map_position'] = self._to_map(
                camera_point, image_msg.header.frame_id,
                image_msg.header.stamp)
        result['id'] = self._track_id(class_name, result['map_position'])
        return result

    def _camera_point(self, box, image_shape, depth, info):
        if depth is None or info is None or len(info.k) < 9:
            return None
        image_height, image_width = image_shape
        scale_x = depth.shape[1] / max(1, image_width)
        scale_y = depth.shape[0] / max(1, image_height)
        x1, y1, x2, y2 = box
        center_x, center_y = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        half_width = max(1.0, (x2 - x1) * self.center_ratio / 2.0)
        half_height = max(1.0, (y2 - y1) * self.center_ratio / 2.0)
        left = max(0, int((center_x - half_width) * scale_x))
        right = min(depth.shape[1], int((center_x + half_width) * scale_x))
        top = max(0, int((center_y - half_height) * scale_y))
        bottom = min(depth.shape[0], int((center_y + half_height) * scale_y))
        values = depth[top:bottom, left:right]
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size < self.min_depth_pixels:
            return None
        z = float(np.median(valid))
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return (
            (center_x - cx) * z / fx,
            (center_y - cy) * z / fy,
            z,
        )

    def _to_map(self, point, source_frame, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, Time.from_msg(stamp))
        except Exception:
            return None
        rotation = self._quaternion_matrix(transform.transform.rotation)
        translated = rotation @ np.asarray(point, dtype=np.float64)
        translation = transform.transform.translation
        translated += [translation.x, translation.y, translation.z]
        return {
            'x': round(float(translated[0]), 3),
            'y': round(float(translated[1]), 3),
            'z': round(float(translated[2]), 3),
            'frame_id': self.map_frame,
        }

    @staticmethod
    def _quaternion_matrix(q):
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        norm = x * x + y * y + z * z + w * w
        if norm < 1e-12:
            return np.eye(3)
        scale = 2.0 / norm
        return np.asarray([
            [1.0 - scale * (y * y + z * z),
             scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w),
             1.0 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w),
             1.0 - scale * (x * x + y * y)],
        ])

    def _track_id(self, class_name, map_position):
        now = time.monotonic()
        expired = [key for key, value in self.tracks.items()
                   if now - value['seen'] > self.track_timeout]
        for key in expired:
            self.tracks.pop(key, None)
        if map_position is not None:
            best_key, best_distance = None, float('inf')
            for key, track in self.tracks.items():
                if track['class_name'] != class_name or track['position'] is None:
                    continue
                distance = math.hypot(
                    map_position['x'] - track['position']['x'],
                    map_position['y'] - track['position']['y'])
                if distance < best_distance:
                    best_key, best_distance = key, distance
            if best_key is not None and best_distance <= self.track_match_distance:
                self.tracks[best_key] = {
                    'class_name': class_name,
                    'position': map_position, 'seen': now}
                return best_key
        self.track_counter += 1
        key = f'{class_name}-{self.track_counter:04d}'
        self.tracks[key] = {
            'class_name': class_name,
            'position': map_position, 'seen': now}
        return key

    def _publish_status(self, state, message, **extra):
        self.status_pub.publish(String(data=json.dumps({
            'state': state, 'message': message, **extra,
        }, ensure_ascii=False)))

    def destroy_node(self):
        self.inference_pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

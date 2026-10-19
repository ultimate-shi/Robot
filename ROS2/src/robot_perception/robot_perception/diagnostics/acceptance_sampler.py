#!/usr/bin/env python3
"""使用方法：通过 /perception/capture_sample 保存现场检测与视觉问答验收样本。"""

import csv
from datetime import datetime
import json
import os
import threading

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from robot_interfaces.msg import SemanticDetectionArray
from robot_interfaces.srv import CaptureSample
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class AcceptanceSampler(Node):
    """保存同一时刻附近的图像、深度、标定、TF和自动预标注."""

    def __init__(self):
        super().__init__('acceptance_sampler')
        defaults = {
            'output_directory': '/workspace/acceptance_dataset',
            'image_topic': '/stereo/right/image_rect',
            'depth_topic': '/stereo/depth/right/image',
            'camera_info_topic': '/stereo/right/camera_info',
            'detections_topic': '/perception/semantic_detections',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'jpeg_quality': 95,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.output_directory = str(
            self.get_parameter('output_directory').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.jpeg_quality = int(
            self.get_parameter('jpeg_quality').value)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.image = None
        self.depth = None
        self.info = None
        self.detections = {'detections': []}

        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._set_image, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value),
            self._set_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._set_info, qos_profile_sensor_data)
        self.create_subscription(
            SemanticDetectionArray,
            str(self.get_parameter('detections_topic').value),
            self._set_detections, 10)
        self.create_subscription(
            String, '/brain/sample_request', self._save_request, 10)
        self.status_pub = self.create_publisher(
            String, '/brain/sample_status', 10)
        self.create_service(
            CaptureSample, '/perception/capture_sample',
            self._capture_service)

    def _set_image(self, msg):
        with self.lock:
            self.image = msg

    def _set_depth(self, msg):
        with self.lock:
            self.depth = msg

    def _set_info(self, msg):
        with self.lock:
            self.info = msg

    def _set_detections(self, msg):
        payload = {'detections': []}
        for item in msg.detections:
            payload['detections'].append({
                'id': item.id, 'class_name': item.class_name,
                'label_zh': item.label_zh,
                'confidence': item.confidence,
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
        with self.lock:
            self.detections = payload

    def _save_request(self, msg):
        try:
            request = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            request = {}
        success, message, directory = self._save(
            str(request.get('scene_id', 'scene')))
        self._status(success, message, directory=directory)

    def _capture_service(self, request, response):
        """为前端提供有结果的类型化采样服务。"""
        success, message, directory = self._save(request.scene_id)
        response.success = success
        response.message = message
        response.output_path = directory
        return response

    def _save(self, scene_id):
        """保存一次原子快照并返回成功状态、说明和目录。"""
        with self.lock:
            image_msg, depth_msg, info_msg = self.image, self.depth, self.info
            detections = json.loads(json.dumps(self.detections))
        if image_msg is None or depth_msg is None or info_msg is None:
            return False, '图像、深度或 CameraInfo 尚未全部就绪', ''
        try:
            directory = self._write_sample(
                image_msg, depth_msg, info_msg, detections,
                str(scene_id or 'scene'))
        except (OSError, TypeError, ValueError) as exc:
            return False, f'保存验收样本失败: {exc}', ''
        return True, '验收样本已保存', directory

    def _write_sample(self, image_msg, depth_msg, info_msg, detections,
                      scene_id):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        safe_scene = ''.join(
            char for char in scene_id if char.isalnum() or char in '-_')
        sample_id = f'{safe_scene or "scene"}_{stamp}'
        directory = os.path.join(self.output_directory, 'samples', sample_id)
        os.makedirs(directory, exist_ok=False)
        image = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding='bgr8')
        if not cv2.imwrite(
                os.path.join(directory, 'right_rect.jpg'), image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]):
            raise OSError('写入右目图像失败')
        depth = self._depth_array(depth_msg)
        if depth is None:
            raise ValueError('深度图编码或尺寸无效')
        np.save(os.path.join(directory, 'depth_m.npy'), depth)
        metadata = {
            'sample_id': sample_id,
            'created_at': datetime.now().astimezone().isoformat(),
            'image_file': 'right_rect.jpg',
            'depth_file': 'depth_m.npy',
            'image_frame': image_msg.header.frame_id,
            'camera_info': {
                'width': info_msg.width, 'height': info_msg.height,
                'distortion_model': info_msg.distortion_model,
                'd': list(info_msg.d), 'k': list(info_msg.k),
                'r': list(info_msg.r), 'p': list(info_msg.p),
            },
            'robot_pose': self._robot_pose(),
            'auto_detections': detections.get('detections', []),
            'annotation_status': '人工未复核',
        }
        with open(os.path.join(directory, 'metadata.json'), 'w',
                  encoding='utf-8') as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2)
        self._append_manifest(sample_id, directory, detections)
        return directory

    def _append_manifest(self, sample_id, directory, detections):
        os.makedirs(self.output_directory, exist_ok=True)
        manifest = os.path.join(self.output_directory, 'annotation_manifest.csv')
        new_file = not os.path.exists(manifest)
        with open(manifest, 'a', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            if new_file:
                writer.writerow([
                    'sample_id', 'image_path', 'auto_detection_count',
                    'human_annotation_status', 'reviewer', 'notes'])
            writer.writerow([
                sample_id, os.path.join(directory, 'right_rect.jpg'),
                len(detections.get('detections', [])), '待人工标注', '', ''])
        qa_path = os.path.join(self.output_directory, 'vqa_template.jsonl')
        template = {
            'sample_id': sample_id,
            'image_path': os.path.join(directory, 'right_rect.jpg'),
            'question': '', 'reference_answer': '',
            'acceptable_synonyms': [], 'answerable_from_image': True,
            'human_score': '', 'notes': '请人工填写问题、标准答案并复核模型回答',
        }
        with open(qa_path, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(template, ensure_ascii=False) + '\n')

    def _robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        value = transform.transform
        return {
            'frame_id': self.map_frame,
            'position': {
                'x': value.translation.x, 'y': value.translation.y,
                'z': value.translation.z,
            },
            'orientation': {
                'x': value.rotation.x, 'y': value.rotation.y,
                'z': value.rotation.z, 'w': value.rotation.w,
            },
        }

    @staticmethod
    def _depth_array(msg):
        if msg.encoding != '32FC1' or msg.step < msg.width * 4:
            return None
        endian = '>' if msg.is_bigendian else '<'
        return np.ndarray(
            (msg.height, msg.width), dtype=np.dtype(endian + 'f4'),
            buffer=msg.data, strides=(msg.step, 4)).astype(
                np.float32, copy=True)

    def _status(self, success, message, **extra):
        self.status_pub.publish(String(data=json.dumps({
            'success': success, 'message': message, **extra,
        }, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = AcceptanceSampler()
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

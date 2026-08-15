#!/usr/bin/env python3
"""使用方法：由 stereo_camera.launch.py 把视差转换为米制深度和预览图。"""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from stereo_msgs.msg import DisparityImage


class StereoDepth(Node):
    """根据 Z=fT/d 计算深度，无效值保持为 NaN."""

    def __init__(self):
        super().__init__('stereo_depth')
        self.declare_parameter('disparity_topic', '/stereo/disparity')
        self.declare_parameter('depth_topic', '/stereo/depth/image')
        self.declare_parameter(
            'right_depth_topic', '/stereo/depth/right/image')
        self.declare_parameter(
            'right_depth_frame_id', 'stereo_right_optical_frame')
        self.declare_parameter(
            'visual_topic', '/stereo/depth/image_visual')
        self.declare_parameter('min_depth', 0.25)
        self.declare_parameter('max_depth', 4.0)
        self.declare_parameter('status_topic', '/stereo/depth/status')

        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.right_depth_frame = str(
            self.get_parameter('right_depth_frame_id').value)
        self.frames = 0
        self.last_status_frames = 0
        self.last_status_time = time.monotonic()
        self.processing_samples_ms = []
        self.age_samples_ms = []
        # 深度是高带宽传感器数据，BEST_EFFORT/KEEP_LAST(1) 避免远程监测反压计算链。
        output_qos = qos_profile_sensor_data
        self.depth_pub = self.create_publisher(
            Image, str(self.get_parameter('depth_topic').value),
            output_qos)
        self.right_depth_pub = self.create_publisher(
            Image, str(self.get_parameter('right_depth_topic').value),
            output_qos)
        self.visual_pub = self.create_publisher(
            Image, str(self.get_parameter('visual_topic').value),
            output_qos)
        self.create_subscription(
            DisparityImage,
            str(self.get_parameter('disparity_topic').value),
            self.disparity_callback,
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_timer(1.0, self._publish_status)

    def disparity_callback(self, msg):
        processing_started = time.perf_counter()
        image = msg.image
        if image.encoding != '32FC1':
            self.get_logger().error(
                f'视差图编码必须是 32FC1，当前为 {image.encoding}',
                throttle_duration_sec=2.0)
            return
        if image.step < image.width * 4:
            self.get_logger().error('视差图 step 小于一行 float32 数据长度')
            return

        endian = '>' if image.is_bigendian else '<'
        disparity = np.ndarray(
            shape=(image.height, image.width),
            dtype=np.dtype(endian + 'f4'),
            buffer=image.data,
            strides=(image.step, 4),
        ).astype(np.float32, copy=False)

        depth = np.full(disparity.shape, np.nan, dtype=np.float32)
        scale = float(msg.f) * float(msg.t)
        valid = np.isfinite(disparity) & (disparity > 0.0)
        if math.isfinite(scale) and scale > 0.0:
            depth[valid] = scale / disparity[valid]
        valid_depth = (
            np.isfinite(depth)
            & (depth >= self.min_depth)
            & (depth <= self.max_depth)
        )
        depth[~valid_depth] = np.nan

        depth_msg = Image()
        depth_msg.header = image.header
        depth_msg.height = image.height
        depth_msg.width = image.width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = image.width * 4
        depth_msg.data = np.ascontiguousarray(depth).tobytes()
        self.depth_pub.publish(depth_msg)

        # stereo_image_proc 的视差和默认深度都以左目为基准。右目检测框中的
        # 像素 x_r 对应 x_l-disparity，将有效深度前向映射到右目坐标；
        # 多个左目像素落到同一右目像素时保留最近表面，避免穿透障碍物。
        right_depth = StereoDepth._right_aligned_depth(disparity, depth)
        right_depth_msg = Image()
        right_depth_msg.header.stamp = image.header.stamp
        right_depth_msg.header.frame_id = self.right_depth_frame
        right_depth_msg.height = image.height
        right_depth_msg.width = image.width
        right_depth_msg.encoding = '32FC1'
        right_depth_msg.is_bigendian = False
        right_depth_msg.step = image.width * 4
        right_depth_msg.data = np.ascontiguousarray(right_depth).tobytes()
        self.right_depth_pub.publish(right_depth_msg)

        # 近处更亮、远处更暗；无效像素为 0，便于 Foxglove 直接显示。
        visual = np.zeros(depth.shape, dtype=np.uint8)
        if self.max_depth > self.min_depth:
            normalized = (
                (self.max_depth - depth[valid_depth])
                / (self.max_depth - self.min_depth)
            )
            visual[valid_depth] = np.clip(
                1.0 + normalized * 254.0, 1.0, 255.0).astype(np.uint8)
        visual_msg = Image()
        visual_msg.header = image.header
        visual_msg.height = image.height
        visual_msg.width = image.width
        visual_msg.encoding = 'mono8'
        visual_msg.is_bigendian = False
        visual_msg.step = image.width
        visual_msg.data = visual.tobytes()
        self.visual_pub.publish(visual_msg)
        self.frames += 1
        self.processing_samples_ms.append(
            (time.perf_counter() - processing_started) * 1000.0)
        stamp_ns = (
            int(image.header.stamp.sec) * 1_000_000_000
            + int(image.header.stamp.nanosec)
        )
        if stamp_ns > 0:
            age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
            self.age_samples_ms.append(max(0.0, age_ms))

    @staticmethod
    def _right_aligned_depth(disparity, depth):
        """把左目深度按 x_right=x_left-disparity 映射到右目像素坐标。"""
        height, width = depth.shape
        rows, columns = np.indices((height, width))
        valid = np.isfinite(depth) & np.isfinite(disparity) & (disparity > 0.0)
        coordinates = np.where(
            np.isfinite(disparity), columns - disparity, -1.0)
        right_columns = np.rint(coordinates).astype(np.int32)
        valid &= (right_columns >= 0) & (right_columns < width)
        output = np.full(height * width, np.inf, dtype=np.float32)
        indices = rows[valid] * width + right_columns[valid]
        np.minimum.at(output, indices, depth[valid])
        output[~np.isfinite(output)] = np.nan
        return output.reshape((height, width))

    @staticmethod
    def _percentile(values, ratio):
        if not values:
            return None
        ordered = sorted(values)
        index = min(len(ordered) - 1, round((len(ordered) - 1) * ratio))
        return round(float(ordered[index]), 2)

    def _publish_status(self):
        """每秒报告深度输出频率、处理时间和采集到深度的消息年龄."""
        now = time.monotonic()
        elapsed = max(now - self.last_status_time, 1e-6)
        payload = {
            'state': 'ok' if self.frames else 'waiting_disparity',
            'fps': round((self.frames - self.last_status_frames) / elapsed, 2),
            'frames': self.frames,
            'processing_p95_ms': self._percentile(
                self.processing_samples_ms, 0.95),
            'capture_to_depth_p95_ms': self._percentile(
                self.age_samples_ms, 0.95),
        }
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.last_status_time = now
        self.last_status_frames = self.frames
        self.processing_samples_ms.clear()
        self.age_samples_ms.clear()


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepth()
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

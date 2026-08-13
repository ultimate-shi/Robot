#!/usr/bin/env python3
"""从左右校正图中选取最新的同时间戳图像对，限制立体匹配频率。"""

from collections import OrderedDict
import copy
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def stamp_key(msg) -> int:
    """把 ROS 时间戳转换成可比较的纳秒整数."""
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(
        msg.header.stamp.nanosec)


class StereoPairThrottle(Node):
    """以固定频率发布最新完整图像对，不重复旧帧且不积压队列."""

    def __init__(self):
        super().__init__('stereo_pair_throttle')
        defaults = {
            'left_image_topic': '/stereo/left/image_rect',
            'right_image_topic': '/stereo/right/image_rect',
            'left_camera_info_topic': '/stereo/left/camera_info',
            'right_camera_info_topic': '/stereo/right/camera_info',
            'output_left_image_topic': '/stereo/navigation/left/image_rect',
            'output_right_image_topic': '/stereo/navigation/right/image_rect',
            'output_left_camera_info_topic':
                '/stereo/navigation/left/camera_info',
            'output_right_camera_info_topic':
                '/stereo/navigation/right/camera_info',
            'status_topic': '/stereo/pair_throttle/status',
            'navigation_rate': 4.0,
            'pair_cache_size': 4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.navigation_rate = max(
            0.1, float(self.get_parameter('navigation_rate').value))
        self.pair_cache_size = max(
            2, int(self.get_parameter('pair_cache_size').value))

        # 输入保持 BEST_EFFORT，慢 SGBM 不会反压上游 10 Hz 识别链；门后四路消息
        # 必须可靠送达，否则任一 Image/CameraInfo 丢失都会让精确同步整组作废。
        input_qos = qos_profile_sensor_data
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.left_pub = self.create_publisher(
            Image,
            str(self.get_parameter('output_left_image_topic').value),
            output_qos,
        )
        self.right_pub = self.create_publisher(
            Image,
            str(self.get_parameter('output_right_image_topic').value),
            output_qos,
        )
        self.left_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('output_left_camera_info_topic').value),
            output_qos,
        )
        self.right_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter('output_right_camera_info_topic').value),
            output_qos,
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)

        self.left_images = OrderedDict()
        self.right_images = OrderedDict()
        self.left_info = None
        self.right_info = None
        self.last_published_stamp = -1
        self.received_left = 0
        self.received_right = 0
        self.published_pairs = 0
        self.skipped_ticks = 0
        self.last_publish_monotonic = None

        self.create_subscription(
            Image,
            str(self.get_parameter('left_image_topic').value),
            self._left_callback,
            input_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('right_image_topic').value),
            self._right_callback,
            input_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('left_camera_info_topic').value),
            self._left_info_callback,
            input_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('right_camera_info_topic').value),
            self._right_info_callback,
            input_qos,
        )
        self.create_timer(1.0 / self.navigation_rate, self._publish_latest_pair)
        self.get_logger().info(
            f'导航双目最新帧调度已启动: {self.navigation_rate:.2f} Hz')

    def _left_callback(self, msg):
        self.received_left += 1
        self._store(self.left_images, msg)

    def _right_callback(self, msg):
        self.received_right += 1
        self._store(self.right_images, msg)

    def _left_info_callback(self, msg):
        self.left_info = msg

    def _right_info_callback(self, msg):
        self.right_info = msg

    def _store(self, cache, msg):
        key = stamp_key(msg)
        cache[key] = msg
        cache.move_to_end(key)
        while len(cache) > self.pair_cache_size:
            cache.popitem(last=False)

    def _publish_latest_pair(self):
        common = self.left_images.keys() & self.right_images.keys()
        candidates = [key for key in common if key > self.last_published_stamp]
        if not candidates or self.left_info is None or self.right_info is None:
            self.skipped_ticks += 1
            self._publish_status('waiting_pair')
            return

        key = max(candidates)
        left = self.left_images[key]
        right = self.right_images[key]
        self.left_pub.publish(left)
        self.right_pub.publish(right)
        self.left_info_pub.publish(self._info_for_image(self.left_info, left))
        self.right_info_pub.publish(self._info_for_image(self.right_info, right))
        self.last_published_stamp = key
        self.published_pairs += 1
        self._drop_older(key)
        self._publish_status('ok')

    @staticmethod
    def _info_for_image(template, image):
        info = copy.deepcopy(template)
        info.header.stamp = image.header.stamp
        info.header.frame_id = image.header.frame_id
        return info

    def _drop_older(self, published_key):
        for cache in (self.left_images, self.right_images):
            for key in list(cache):
                if key <= published_key:
                    del cache[key]

    def _publish_status(self, state):
        now = time.monotonic()
        fps = 0.0
        if state == 'ok':
            if self.last_publish_monotonic is not None:
                period = now - self.last_publish_monotonic
                if period > 0.0:
                    fps = 1.0 / period
            self.last_publish_monotonic = now
        payload = {
            'state': state,
            'target_fps': self.navigation_rate,
            'fps': round(fps, 2),
            'received_left': self.received_left,
            'received_right': self.received_right,
            'published_pairs': self.published_pairs,
            'skipped_ticks': self.skipped_ticks,
        }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = StereoPairThrottle()
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

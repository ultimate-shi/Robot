#!/usr/bin/env python3
"""把 stereo_image_proc 的视差图转换为米制深度图和 8 位预览图。"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage


class StereoDepth(Node):
    """根据 Z=fT/d 计算深度，无效值保持为 NaN。"""

    def __init__(self):
        super().__init__('stereo_depth')
        self.declare_parameter('disparity_topic', '/stereo/disparity')
        self.declare_parameter('depth_topic', '/stereo/depth/image')
        self.declare_parameter(
            'visual_topic', '/stereo/depth/image_visual')
        self.declare_parameter('min_depth', 0.25)
        self.declare_parameter('max_depth', 4.0)

        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.depth_pub = self.create_publisher(
            Image, str(self.get_parameter('depth_topic').value),
            qos_profile_sensor_data)
        self.visual_pub = self.create_publisher(
            Image, str(self.get_parameter('visual_topic').value),
            qos_profile_sensor_data)
        self.create_subscription(
            DisparityImage,
            str(self.get_parameter('disparity_topic').value),
            self.disparity_callback,
            qos_profile_sensor_data,
        )

    def disparity_callback(self, msg):
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

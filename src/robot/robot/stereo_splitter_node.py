#!/usr/bin/env python3
"""拆分横向拼接的 UVC 双目图像，并发布左右图像和标定信息。"""

import os

import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class StereoSplitter(Node):
    """保持同一采集时间戳，将一帧横向拼接图像拆成左右两帧。"""

    def __init__(self):
        super().__init__('stereo_splitter')
        self.declare_parameter('input_topic', '/stereo/image_raw')
        self.declare_parameter('left_image_topic', '/stereo/left/image_raw')
        self.declare_parameter('right_image_topic', '/stereo/right/image_raw')
        self.declare_parameter('left_camera_info_topic', '/stereo/left/camera_info')
        self.declare_parameter('right_camera_info_topic', '/stereo/right/camera_info')
        self.declare_parameter('left_frame_id', 'stereo_left_optical_frame')
        self.declare_parameter('right_frame_id', 'stereo_right_optical_frame')
        self.declare_parameter('left_first', True)
        self.declare_parameter('frame_skip', 0)
        self.declare_parameter('output_encoding', 'passthrough')
        self.declare_parameter('calibration_mode', False)
        self.declare_parameter('left_calibration_file', '')
        self.declare_parameter('right_calibration_file', '')

        self.left_first = bool(self.get_parameter('left_first').value)
        self.frame_skip = max(0, int(self.get_parameter('frame_skip').value))
        self.output_encoding = str(self.get_parameter('output_encoding').value)
        self.calibration_mode = bool(self.get_parameter('calibration_mode').value)
        self.left_frame = str(self.get_parameter('left_frame_id').value)
        self.right_frame = str(self.get_parameter('right_frame_id').value)
        self.frame_count = 0

        self.left_info = self._load_camera_info(
            str(self.get_parameter('left_calibration_file').value),
            self.left_frame,
            'left',
        )
        self.right_info = self._load_camera_info(
            str(self.get_parameter('right_calibration_file').value),
            self.right_frame,
            'right',
        )

        input_topic = str(self.get_parameter('input_topic').value)
        self.left_pub = self.create_publisher(
            Image, str(self.get_parameter('left_image_topic').value),
            qos_profile_sensor_data)
        self.right_pub = self.create_publisher(
            Image, str(self.get_parameter('right_image_topic').value),
            qos_profile_sensor_data)
        self.left_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('left_camera_info_topic').value),
            qos_profile_sensor_data)
        self.right_info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('right_camera_info_topic').value),
            qos_profile_sensor_data)
        self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'双目拆分已启动: {input_topic}, left_first={self.left_first}, '
            f'frame_skip={self.frame_skip}')

    def _load_camera_info(self, path, frame_id, side):
        """读取 camera_calibration 生成的 YAML；标定模式允许文件缺失。"""
        if not path or not os.path.isfile(path):
            message = f'{side} 标定文件不存在: {path or "(空)"}'
            if self.calibration_mode:
                self.get_logger().warn(message + '；标定模式仅发布原始图像')
                return None
            self.get_logger().error(message + '；请先完成双目标定')
            return None

        try:
            with open(path, 'r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream)
            info = CameraInfo()
            info.width = int(data['image_width'])
            info.height = int(data['image_height'])
            info.distortion_model = str(data.get(
                'distortion_model', 'plumb_bob'))
            info.d = [float(value) for value in
                      data['distortion_coefficients']['data']]
            info.k = [float(value) for value in data['camera_matrix']['data']]
            info.r = [float(value) for value in
                      data['rectification_matrix']['data']]
            info.p = [float(value) for value in
                      data['projection_matrix']['data']]
            info.header.frame_id = frame_id
            if not self.calibration_mode:
                if info.k[0] <= 0.0 or info.k[4] <= 0.0:
                    self.get_logger().error(
                        f'{side} 标定文件仍是模板，不能启动深度处理')
                    return None
                if side == 'right' and abs(info.p[3]) <= 1e-9:
                    self.get_logger().error(
                        'right 投影矩阵没有有效 Tx，不能用标称基线替代标定')
                    return None
            return info
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f'{side} 标定文件格式错误: {exc}')
            return None

    @staticmethod
    def _bytes_per_pixel(encoding):
        return {
            'mono8': 1,
            '8UC1': 1,
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
        }.get(encoding)

    def image_callback(self, msg):
        self.frame_count += 1
        if (self.frame_count - 1) % (self.frame_skip + 1):
            return
        if msg.width < 2 or msg.width % 2:
            self.get_logger().error(
                f'拼接图宽度必须是正偶数，当前为 {msg.width}',
                throttle_duration_sec=2.0)
            return

        bytes_per_pixel = self._bytes_per_pixel(msg.encoding)
        if bytes_per_pixel is None:
            self.get_logger().error(
                f'暂不支持输入编码 {msg.encoding}；请让 usb_cam 输出 '
                'mono8/rgb8/bgr8/rgba8/bgra8',
                throttle_duration_sec=2.0)
            return

        half_width = msg.width // 2
        row_bytes = msg.width * bytes_per_pixel
        if msg.step < row_bytes or len(msg.data) < msg.step * msg.height:
            self.get_logger().error('输入 Image 的 step 或 data 长度无效')
            return

        rows = np.ndarray(
            shape=(msg.height, msg.step),
            dtype=np.uint8,
            buffer=msg.data,
        )
        packed = rows[:, :row_bytes].reshape(
            msg.height, msg.width, bytes_per_pixel)
        first = np.ascontiguousarray(packed[:, :half_width])
        second = np.ascontiguousarray(packed[:, half_width:])
        left, right = (first, second) if self.left_first else (second, first)

        encoding = (msg.encoding if self.output_encoding == 'passthrough'
                    else self.output_encoding)
        if encoding != msg.encoding:
            self.get_logger().error(
                '当前节点不做颜色空间转换；output_encoding 应设为 passthrough '
                '或与输入编码一致',
                throttle_duration_sec=2.0)
            return

        left_msg = self._make_image(msg, left, half_width, self.left_frame)
        right_msg = self._make_image(msg, right, half_width, self.right_frame)
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self._publish_info(self.left_info_pub, self.left_info, left_msg)
        self._publish_info(self.right_info_pub, self.right_info, right_msg)

    @staticmethod
    def _make_image(source, array, width, frame_id):
        output = Image()
        output.header.stamp = source.header.stamp
        output.header.frame_id = frame_id
        output.height = source.height
        output.width = width
        output.encoding = source.encoding
        output.is_bigendian = source.is_bigendian
        output.step = width * array.shape[2]
        output.data = array.tobytes()
        return output

    @staticmethod
    def _publish_info(publisher, template, image):
        if template is None:
            return
        template.header.stamp = image.header.stamp
        template.header.frame_id = image.header.frame_id
        publisher.publish(template)


def main(args=None):
    rclpy.init(args=args)
    node = StereoSplitter()
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

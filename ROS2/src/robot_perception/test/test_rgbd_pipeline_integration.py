"""作用：验证 C++ RGB-D 时间/几何拒绝、限速及预览；用法：独立 DDS 域设 RUN_RGBD_INTEGRATION=1 后 pytest 本文件。"""
import copy
import os
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import Image, CameraInfo


@pytest.mark.skipif(os.environ.get('RUN_RGBD_INTEGRATION') != '1', reason='需独立 DDS 域显式启用')
def test_cpp_rgbd_throttle_rejects_invalid_input_and_preserves_stamps(tmp_path):
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 100
    rclpy.init()
    node = rclpy.create_node('rgbd_validation_probe')
    publisher = node.create_publisher(RGBDImage, '/probe/rgbd', 10)
    received, previews, depths, infos = [], [], [], []
    node.create_subscription(RGBDImage, '/probe/out', received.append, 10)
    node.create_subscription(Image, '/probe/preview', previews.append, 10)
    node.create_subscription(Image, '/probe/depth', depths.append, 10)
    node.create_subscription(CameraInfo, '/probe/info', infos.append, 10)
    log = open(tmp_path/'cpp.log', 'w')
    process = subprocess.Popen(['ros2', 'run', 'robot_stereo_components', 'rgbd_throttle_cpp',
        '--ros-args', '-r', 'rgbd_in:=/probe/rgbd', '-r', 'rgbd_out:=/probe/out',
        '-r', 'preview_out:=/probe/preview', '-r', 'depth_out:=/probe/depth',
        '-r', 'info_out:=/probe/info'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
    def sample():
        message = RGBDImage()
        for image in (message.rgb, message.depth):
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id, image.width, image.height = 'color_optical', 4, 4
        message.rgb.encoding, message.rgb.step = 'rgb8', 12
        message.rgb.data = bytes(48)
        message.depth.encoding, message.depth.step = '16UC1', 8
        depths = np.full((4, 4), 1000, dtype=np.uint16)
        depths[0, 0] = 0
        message.depth.data = depths.tobytes()
        message.rgb_camera_info.width = message.rgb_camera_info.height = 4
        message.rgb_camera_info.k = [100., 0., 2., 0., 100., 2., 0., 0., 1.]
        message.rgb_camera_info.p = [200., 0., 2., 0., 0., 200., 2., 0., 0., 0., 1., 0.]
        message.rgb_camera_info.d = [.1, 0., 0., 0., 0.]
        return message
    try:
        end = time.monotonic()+10
        while publisher.get_subscription_count() == 0:
            assert time.monotonic() < end
            spin(.05)
        valid = sample()
        publisher.publish(valid)
        spin(.3)
        assert len(received) == 1 and previews
        assert received[0].rgb.header == valid.rgb.header
        assert received[0].depth.header == valid.depth.header
        assert previews[0].encoding == 'mono8' and previews[0].data[0] == 0
        assert previews[0].data[1] > 0
        assert depths and depths[0].encoding == '32FC1'
        meters = np.frombuffer(depths[0].data, dtype=np.float32)
        assert np.isnan(meters[0]) and meters[1] == pytest.approx(1.)
        assert infos and infos[0].k[0] == 200. and not any(infos[0].d)
        for case in ('stale', 'skew', 'frame', 'size', 'uncalibrated', 'encoding'):
            invalid = sample()
            if case == 'stale':
                invalid.rgb.header.stamp.sec -= 3
            elif case == 'skew':
                invalid.depth.header.stamp.sec -= 1
            elif case == 'frame':
                invalid.depth.header.frame_id = 'native_unregistered_depth'
            elif case == 'size':
                invalid.depth.width = 2
            elif case == 'uncalibrated':
                invalid.rgb_camera_info.k[0] = 0.
            else:
                invalid.depth.encoding = '8UC1'
            publisher.publish(invalid)
            spin(.15)
            assert len(received) == 1, case
        for _ in range(10):
            publisher.publish(sample())
            spin(.01)
        spin(.1)
        assert 1 < len(received) <= 4
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()

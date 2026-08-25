"""使用方法：pytest 运行本文件，验证不依赖相机硬件的双目核心处理。"""

import threading

import numpy as np

from robot_perception.diagnostics.stereo_pipeline_benchmark import percentile
from robot_perception.semantic.semantic_perception import (
    InferencePaused, SemanticPerception)
from robot_perception.stereo.stereo_depth import StereoDepth
from robot_perception.stereo.stereo_pointcloud_filter import StereoPointCloudFilter
from robot_perception.stereo.stereo_splitter import StereoSplitter
from sensor_msgs.msg import Image, PointCloud2, PointField
from stereo_msgs.msg import DisparityImage


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def error(self, *args, **kwargs):
        raise AssertionError(args[0])


def test_splitter_preserves_stamp_and_left_right_order():
    """横向拼接图应按配置拆分，并让左右图保持同一时间戳."""
    fake = type('FakeSplitter', (), {})()
    fake.frame_count = 0
    fake.published_count = 0
    fake.frame_skip = 0
    fake.calibration_mode = True
    fake.left_first = True
    fake.output_encoding = 'passthrough'
    fake.left_frame = 'left_optical'
    fake.right_frame = 'right_optical'
    fake.left_info = None
    fake.right_info = None
    fake.left_pub = _Publisher()
    fake.right_pub = _Publisher()
    fake.left_info_pub = _Publisher()
    fake.right_info_pub = _Publisher()
    fake.get_logger = lambda: _Logger()
    fake._bytes_per_pixel = StereoSplitter._bytes_per_pixel
    fake._make_image = StereoSplitter._make_image
    fake._publish_info = StereoSplitter._publish_info

    image = Image()
    image.header.stamp.sec = 123
    image.height = 1
    image.width = 4
    image.encoding = 'mono8'
    image.step = 4
    image.data = bytes([10, 11, 20, 21])
    StereoSplitter.image_callback(fake, image)

    assert list(fake.left_pub.messages[0].data) == [10, 11]
    assert list(fake.right_pub.messages[0].data) == [20, 21]
    assert fake.left_pub.messages[0].header.stamp.sec == 123
    assert fake.right_pub.messages[0].header.stamp.sec == 123


def test_depth_uses_metric_scale_and_nan_for_invalid_values():
    """深度应使用 Z=fT/d，零视差与超范围结果必须为 NaN."""
    fake = type('FakeDepth', (), {})()
    fake.min_depth = 0.25
    fake.max_depth = 4.0
    fake.depth_pub = _Publisher()
    fake.right_depth_pub = _Publisher()
    fake.right_depth_frame = 'right_optical'
    fake.visual_pub = _Publisher()
    fake.frames = 0
    fake.processing_samples_ms = []
    fake.age_samples_ms = []
    fake.get_logger = lambda: _Logger()

    message = DisparityImage()
    message.f = 100.0
    message.t = 0.1
    message.image.height = 1
    message.image.width = 4
    message.image.encoding = '32FC1'
    message.image.step = 16
    message.image.data = np.array(
        [10.0, 0.0, 2.0, 40.0], dtype=np.float32).tobytes()
    StereoDepth.disparity_callback(fake, message)

    depth = np.frombuffer(
        fake.depth_pub.messages[0].data, dtype=np.float32)
    assert np.isclose(depth[0], 1.0)
    assert np.isnan(depth[1])
    assert np.isnan(depth[2])
    assert np.isclose(depth[3], 0.25)


def test_right_depth_is_forward_warped_from_left_disparity():
    """右目深度必须按 x_right=x_left-disparity 对齐并保留最近表面。"""
    disparity = np.array([[1.0, 1.0, 1.0, 2.0]], dtype=np.float32)
    depth = np.array([[1.0, 2.0, 3.0, 0.5]], dtype=np.float32)

    right = StereoDepth._right_aligned_depth(disparity, depth)

    assert np.isclose(right[0, 1], 0.5)
    assert np.isnan(right[0, 3])


def test_pointcloud_parser_handles_padding_without_unpack_loop():
    """结构数组解析应支持 point_step 中的额外 padding."""
    message = PointCloud2()
    message.height = 1
    message.width = 2
    message.point_step = 16
    message.row_step = 32
    message.fields = [
        PointField(
            name=name,
            offset=index * 4,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for index, name in enumerate(('x', 'y', 'z'))
    ]
    raw = np.zeros((2, 4), dtype=np.float32)
    raw[:, :3] = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    message.data = raw.tobytes()

    points = StereoPointCloudFilter._cloud_to_xyz(message)
    assert np.allclose(points, raw[:, :3])


def test_voxel_filter_keeps_nearest_point_for_navigation_safety():
    """同一体素存在多个点时必须保留离底盘最近的障碍边界."""
    fake = type('FakeFilter', (), {'voxel_size': 0.05})()
    points = np.array([
        [0.149, 0.01, 0.10],
        [0.101, 0.01, 0.10],
        [0.30, 0.01, 0.10],
    ], dtype=np.float32)

    filtered = StereoPointCloudFilter._voxel_downsample(fake, points)

    assert len(filtered) == 2
    assert np.any(np.isclose(filtered[:, 0], 0.101))
    assert not np.any(np.isclose(filtered[:, 0], 0.149))


def test_benchmark_percentile_is_stable_for_small_samples():
    """验收统计在小样本下也应给出确定的中位数和P95."""
    assert percentile([], 0.95) is None
    assert percentile([10.0, 20.0, 30.0], 0.5) == 20.0
    assert percentile([10.0, 20.0, 30.0], 0.95) == 30.0


def test_semantic_pause_does_not_publish_fake_empty_result():
    """NPU 被 Qwen 占用时只发布暂停状态，不能伪造 YOLO 零目标。"""
    fake = type('FakeSemantic', (), {})()
    fake.lock = threading.Lock()
    fake.in_flight = True
    statuses = []
    fake._publish_status = lambda *args, **kwargs: statuses.append(
        (args, kwargs))
    fake._publish_payload = lambda *_: (_ for _ in ()).throw(
        AssertionError('暂停时不应发布检测数组'))

    class Future:
        def result(self):
            raise InferencePaused('Qwen 正在占用 NPU')

    SemanticPerception._inference_finished(fake, Future())

    assert fake.in_flight is False
    assert statuses[0][0][0] == 'paused'
    assert statuses[0][1]['reason_code'] == 'NPU_BUSY_LLM'

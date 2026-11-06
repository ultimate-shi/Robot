"""作用：合成 RGB-D 验证静止观测合并及长期库转移；用法：独立 ROS 域设置 RUN_MAPPING_MEMORY_INTEGRATION=1 后 pytest -s。"""

import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import cv2
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rtabmap_msgs.msg import Info, RGBDImage
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


@pytest.mark.skipif(os.environ.get('RUN_MAPPING_MEMORY_INTEGRATION') != '1', reason='需显式启用隔离域内存回归')
@pytest.mark.parametrize('moving', [False, True])
def test_stationary_merge_and_working_memory_transfer(tmp_path, moving):
    assert os.environ.get('ROS_DOMAIN_ID', '0') != '0', '禁止接入实车 ROS 域'
    config = Path(__file__).parents[1] / 'config' / 'rtabmap_mapping.yaml'
    database = tmp_path / 'memory.db'
    rclpy.init()
    node = Node('mapping_memory_regression')
    static = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'camera_optical'
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.transform.translation.z = 0.2
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    static.sendTransform(transform)
    dynamic = TransformBroadcaster(node)
    publisher = node.create_publisher(RGBDImage, '/memory_test/rgbd', 1)
    observations = []
    node.create_subscription(Info, '/info', observations.append, 100)
    rng = np.random.default_rng(123)
    texture = cv2.GaussianBlur(rng.integers(0, 256, (120, 160), dtype=np.uint8), (3, 3), 0)
    logfile = open(tmp_path / 'mapping.log', 'w')
    process = subprocess.Popen(['/opt/ros/jazzy/lib/rtabmap_slam/rtabmap', '--ros-args',
        '--params-file', str(config), '-p', f'database_path:={database}',
        '-p', 'subscribe_stereo:=false', '-p', 'subscribe_rgbd:=true',
        '-p', "Rtabmap/DetectionRate:='10.0'", '-p', "Rtabmap/MemoryThr:='20'",
        '-p', "Mem/STMSize:='5'", '-r', 'rgbd_image:=/memory_test/rgbd'],
        stdout=logfile, stderr=subprocess.STDOUT)
    rss = []
    expected_observations = int(os.environ.get('MAPPING_MEMORY_TEST_FRAMES', '65'))
    try:
        deadline = time.monotonic() + max(65, expected_observations * 0.5)
        while len(observations) < expected_observations and time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'mapping.log').read_text()[-3000:]
            stamp = node.get_clock().now().to_msg()
            odom = TransformStamped()
            odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'odom', 'base_link', stamp
            odom.transform.rotation.w = 1.0
            odom.transform.translation.x = len(observations) * 0.15 if moving else 0.0
            dynamic.sendTransform(odom)
            if moving:
                texture = cv2.GaussianBlur(rng.integers(0, 256, (120, 160), dtype=np.uint8), (3, 3), 0)
            color = Image()
            color.header.frame_id, color.header.stamp = 'camera_optical', stamp
            color.width, color.height, color.encoding, color.step = 160, 120, 'mono8', 160
            color.data = texture.tobytes()
            depth = Image()
            depth.header, depth.width, depth.height = color.header, 160, 120
            depth.encoding, depth.step = '16UC1', 320
            depth.data = np.full((120, 160), 2000, dtype=np.uint16).tobytes()
            info = CameraInfo()
            info.header, info.width, info.height = color.header, 160, 120
            info.k = [100., 0., 80., 0., 100., 60., 0., 0., 1.]
            info.p = [100., 0., 80., 0., 0., 100., 60., 0., 0., 0., 1., 0.]
            bundle = RGBDImage()
            bundle.header, bundle.rgb, bundle.depth = color.header, color, depth
            bundle.rgb_camera_info, bundle.depth_camera_info = info, info
            publisher.publish(bundle)
            rclpy.spin_once(node, timeout_sec=0.08)
            status = Path(f'/proc/{process.pid}/status').read_text()
            rss.append(int(next(line.split()[1] for line in status.splitlines() if line.startswith('VmRSS:'))))
            time.sleep(0.1)
        assert len(observations) >= expected_observations, (tmp_path / 'mapping.log').read_text()[-4000:]
        statistics = [dict(zip(message.stats_keys, message.stats_values)) for message in observations]
        last = statistics[-1]
        working = last['Memory/Working_memory_size/']
        short = last['Memory/Short_time_memory_size/']
        if moving:
            # MemoryThr 是目标而非硬限；验证确实转出了显著比例的历史，而不假定严格等于 20。
            assert working + short < len(observations) * 0.8
            assert any(item['Memory/Signatures_removed/'] > 0 for item in statistics)
        else:
            assert working + short <= 5, '相同静止观察不应每帧新增永久节点'
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        with sqlite3.connect(database) as connection:
            count = connection.execute('SELECT count(*) FROM Node WHERE id>0').fetchone()[0]
            active_count = connection.execute('SELECT count(*) FROM Node WHERE id>0 AND weight>=0').fetchone()[0]
        # 更新 ID 的合并会保留已落盘历史（weight=-9），不能把它当成工作内存。
        assert count > 40 if moving else active_count <= 5
        report = {'moving': moving, 'observations': len(observations), 'working_nodes': working,
                  'short_nodes': short, 'database_nodes': count,
                  'active_database_nodes': active_count,
                  'rss_first_mib': round(rss[0] / 1024, 1), 'rss_last_mib': round(rss[-1] / 1024, 1)}
        print(json.dumps(report), flush=True)
        (tmp_path / 'memory_report.json').write_text(json.dumps(report, indent=2))
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        logfile.close()
        node.destroy_node()
        rclpy.shutdown()

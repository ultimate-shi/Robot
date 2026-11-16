"""作用：用双目/RGB-D 验证保存、强制退出恢复、定位重载及里程计；用法：独立 DDS 域设 RUN_RTABMAP_INTEGRATION=1 后 pytest。"""

import os
import json
from pathlib import Path
import signal
import subprocess
import time
import yaml

import cv2
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import RGBDImage
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from robot_navigation.mapping.snapshot_manager import SnapshotManager
from robot_navigation.mapping.database import validate_bundle, file_digest


@pytest.mark.skipif(os.environ.get('RUN_RTABMAP_INTEGRATION') != '1', reason='需显式启用隔离域合成双目集成测试')
@pytest.mark.parametrize('camera_source', ['stereo', 'astra'])
def test_real_rtabmap_database_roundtrip(tmp_path, camera_source):
    assert os.environ.get('ROS_DOMAIN_ID', '0') != '0', '必须使用独立 DDS 域，禁止接入实车'
    config = Path(__file__).parents[1] / 'config' / 'rtabmap_mapping.yaml'
    database = tmp_path / 'active.db'
    rclpy.init(args=['--ros-args', '-p', f'database_path:={database}',
                     '-p', f'save_directory:={tmp_path}', '-p', 'database_service_timeout:=20.0'])
    node = Node('synthetic_mapping_test')
    snapshots = SnapshotManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(snapshots)
    static = StaticTransformBroadcaster(node)
    camera_tf = TransformStamped()
    camera_tf.header.stamp = node.get_clock().now().to_msg()
    camera_tf.header.frame_id = 'base_link'
    camera_tf.child_frame_id = 'camera_optical'
    camera_tf.transform.translation.z = 0.2
    q = camera_tf.transform.rotation
    q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
    static.sendTransform(camera_tf)
    dynamic = TransformBroadcaster(node)
    rgbd_pub = node.create_publisher(RGBDImage, '/camera/rgbd_image', 10)
    visual_odometry = []
    node.create_subscription(Odometry, '/visual_odom_raw', visual_odometry.append, 10)
    publishers = {name: node.create_publisher(kind, '/stereo/' + name, 10)
                  for name, kind in [('left/image_rect', Image), ('right/image_rect', Image),
                                     ('left/camera_info', CameraInfo), ('right/camera_info', CameraInfo)]}
    rng = np.random.default_rng(12)
    left = cv2.GaussianBlur(rng.integers(0, 256, (240, 320), dtype=np.uint8), (3, 3), 0)
    right = np.roll(left, -10, axis=1)
    matches, permissions = [], []
    node.create_subscription(Bool, '/localization/ready', lambda msg: permissions.append(msg.data), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose', matches.append, 10)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/localization/initialpose', 1)
    logfile = open(tmp_path / 'rtabmap.log', 'w')
    processes = []

    def start(db, localization=False):
        args = ['/opt/ros/jazzy/lib/rtabmap_slam/rtabmap', '--ros-args', '--params-file', str(config),
                '-p', f'database_path:={db}', '-p', "Rtabmap/DetectionRate:='2.0'",
                '-r', 'left/image_rect:=/stereo/left/image_rect',
                '-r', 'right/image_rect:=/stereo/right/image_rect',
                '-r', 'left/camera_info:=/stereo/left/camera_info',
                '-r', 'right/camera_info:=/stereo/right/camera_info',
                '-r', 'cloud_map:=/mapping/cloud_map',
                '-r', 'localization_pose:=/rtabmap/localization_pose']
        if localization:
            args = ['ros2', 'launch', 'robot_navigation', 'map_localization.launch.py',
                    f'database_path:={db}', f'camera_source:={camera_source}']
        elif camera_source == 'astra':
            mapping_config = yaml.safe_load(config.read_text())
            mapping_config['rtabmap']['ros__parameters'].update(database_path=str(db),
                                                               **{'Rtabmap/DetectionRate': '2.0'})
            mapping_path = tmp_path/'rgbd_mapping.yaml'
            mapping_path.write_text(yaml.safe_dump(mapping_config))
            args = ['ros2', 'launch', 'robot_navigation', 'rtabmap_mapping.launch.py',
                    'camera_source:=astra', f'config_file:={mapping_path}',
                    f'save_directory:={tmp_path / "auto_recovered"}',
                    f'preview_directory:={tmp_path / "preview"}']
        process = subprocess.Popen(args, stdout=logfile, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def publish(images=True):
        stamp = node.get_clock().now().to_msg()
        odom = TransformStamped()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        odom.transform.rotation.w = 1.0
        dynamic.sendTransform(odom)
        if not images:
            return
        for side, pixels in [('left', left), ('right', right)]:
            image = Image()
            image.header.stamp, image.header.frame_id = stamp, 'camera_optical'
            image.width, image.height, image.encoding, image.step = 320, 240, 'mono8', 320
            image.data = pixels.tobytes()
            info = CameraInfo()
            info.header = image.header
            info.width, info.height = 320, 240
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            info.k = [200.0, 0.0, 160.0, 0.0, 200.0, 120.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [200.0, 0.0, 160.0, -20.0 if side == 'right' else 0.0,
                      0.0, 200.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            publishers[side + '/image_rect'].publish(image)
            publishers[side + '/camera_info'].publish(info)
            if camera_source == 'astra' and side == 'left':
                bundle = RGBDImage()
                bundle.header, bundle.rgb, bundle.rgb_camera_info = image.header, image, info
                depth = Image()
                depth.header, depth.width, depth.height = image.header, 320, 240
                depth.encoding, depth.step = '16UC1', 640
                depth.data = np.full((240, 320), 2000, dtype=np.uint16).tobytes()
                bundle.depth, bundle.depth_camera_info = depth, info
                rgbd_pub.publish(bundle)

    def until(predicate, timeout=35, check_process=True, images=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if check_process and processes and processes[-1].poll() is not None:
                logfile.flush()
                pytest.fail((tmp_path / 'rtabmap.log').read_text()[-6000:])
            publish(images)
            executor.spin_once(timeout_sec=0.1)
            if predicate():
                return
            time.sleep(0.1)
        logfile.flush()
        pytest.fail((tmp_path / 'rtabmap.log').read_text()[-6000:])

    try:
        if camera_source == 'astra':
            odometry = subprocess.Popen(['ros2', 'launch', 'robot_navigation', 'rgbd_odometry.launch.py'],
                stdout=logfile, stderr=subprocess.STDOUT)
            processes.append(odometry)
            until(lambda: len(visual_odometry) >= 3)
            assert visual_odometry[-1].header.frame_id == 'odom'
            assert visual_odometry[-1].child_frame_id == 'base_link'
        mapping = start(database)
        until(lambda: snapshots.latest_map is not None and snapshots.latest_cloud is not None)
        client = node.create_client(Trigger, '/mapping/save_snapshot')
        until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        until(future.done, timeout=35)
        response = future.result()
        assert response.success, response.message
        map_yaml = Path(response.message)
        validate_bundle(map_yaml, map_yaml.with_suffix('.db'))
        # 仅杀隔离域的合成建图进程，验证与 OOM 同样的无清理退出路径。
        if camera_source == 'astra':
            children = Path(f'/proc/{mapping.pid}/task/{mapping.pid}/children').read_text().split()
            slam_pid = next(int(pid) for pid in children if b'/rtabmap_slam/rtabmap' in
                            Path(f'/proc/{pid}/cmdline').read_bytes())
            os.kill(slam_pid, signal.SIGKILL)
            until(lambda: bool(list((tmp_path / 'auto_recovered').glob('map_*/map.yaml'))),
                  timeout=45, check_process=False, images=False)
            map_yaml = next((tmp_path / 'auto_recovered').glob('map_*/map.yaml'))
            mapping.wait(timeout=15)
        else:
            mapping.kill()
            mapping.wait(timeout=10)
            until(lambda: not any(service.service_is_ready() for service in (
                snapshots.pause_client, snapshots.resume_client, snapshots.map_client, snapshots.backup_client)),
                check_process=False, images=False)
            # 即使没有地图/点云订阅缓存，也要从数据库副本恢复，不再被缓存前置检查挡住。
            snapshots.latest_map = snapshots.latest_cloud = None
            future = client.call_async(Trigger.Request())
            until(future.done, timeout=45, check_process=False, images=False)
            assert future.result().success, future.result().message
            map_yaml = Path(future.result().message)
        assert json.loads(map_yaml.with_suffix('.json').read_text())['recovered']
        validate_bundle(map_yaml, map_yaml.with_suffix('.db'))
        matches.clear()
        digest = file_digest(map_yaml.with_suffix('.db'))
        localization = start(map_yaml.with_suffix('.db'), localization=True)
        until(lambda: len(matches) >= 3 and permissions and permissions[-1])
        assert matches[-1].pose.covariance[0] < 0.25
        # 相机断流而里程计仍更新时也必须撤销许可；恢复后重新等待地图匹配。
        until(lambda: permissions and not permissions[-1], timeout=12, images=False)
        hint = PoseWithCovarianceStamped()
        hint.header.frame_id = 'map'
        hint.header.stamp = node.get_clock().now().to_msg()
        hint.pose.pose.orientation.w = 1.0
        hint.pose.covariance[0] = hint.pose.covariance[7] = 0.25
        hint.pose.covariance[35] = 0.12
        initial_pub.publish(hint)
        retry_client = node.create_client(Trigger, '/localization/retry')
        until(retry_client.service_is_ready, images=False)
        retry = retry_client.call_async(Trigger.Request())
        until(retry.done, images=False)
        assert retry.result().success
        assert not permissions[-1], '人工初始估计和重试不能直接授予导航许可'
        until(lambda: permissions[-1])
        localization.send_signal(signal.SIGINT)
        localization.wait(timeout=15)
        assert file_digest(map_yaml.with_suffix('.db')) == digest
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        executor.shutdown()
        snapshots.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        logfile.close()

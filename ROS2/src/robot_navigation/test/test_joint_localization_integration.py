"""作用：真实 AMCL、可切换点云投影及全局融合隔离验证；使用方法：独立 DDS 域设 RUN_JOINT_LOCALIZATION_INTEGRATION=1 后 pytest -s 本文件。"""

import json
import math
import os
import signal
from pathlib import Path
import subprocess
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tf2_msgs.msg import TFMessage


@pytest.mark.skipif(os.environ.get('RUN_JOINT_LOCALIZATION_INTEGRATION') != '1',
                    reason='需独立 DDS 域显式启用')
@pytest.mark.parametrize('cloud_source', ['stereo', 'astra'])
def test_real_amcl_depth_projection_and_visual_priority(tmp_path, cloud_source):
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 100, '禁止接入实机 DDS 域'
    rclpy.init()
    node = rclpy.create_node('joint_localization_probe')
    configs = Path(__file__).parents[1]/'config'
    processes, logs, states, poses = [], [], [], []
    global_map_tfs, unexpected_map_tfs = [], []
    def observe_tf(message, output):
        output.extend(tf for tf in message.transforms if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom')
    node.create_subscription(TFMessage, '/tf', lambda msg: observe_tf(msg, unexpected_map_tfs), 100)
    node.create_subscription(TFMessage, '/probe/fusion_tf', lambda msg: observe_tf(msg, global_map_tfs), 100)
    # 将融合节点的动态 TF 通道单独观察，直接区分它与 AMCL 的实际地图 TF 输出。
    fusion_tf_pub = node.create_publisher(TFMessage, '/probe/fusion_tf', 100)
    flags = {'cloud': True, 'visual': True, 'tf': True, 'visual_x': 0., 'bad_visual': False}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, '/map', transient)
    cloud_pub = node.create_publisher(PointCloud2, '/probe/cloud', 10)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    visual_pub = node.create_publisher(PoseWithCovarianceStamped, '/rtabmap/localization_pose', 10)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/localization/initialpose', 1)
    node.create_subscription(String, '/localization/status', lambda msg: states.append(json.loads(msg.data)), 100)
    node.create_subscription(PoseWithCovarianceStamped, '/localization/fused_pose', poses.append, 100)
    # 模拟视觉定位器的初始估计订阅；视觉匹配输入由本测试明确控制。
    node.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda msg: None, 1)
    dynamic, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    camera = TransformStamped()
    camera.header.frame_id, camera.child_frame_id = 'base_link', 'probe_depth_sensor'
    camera.header.stamp = node.get_clock().now().to_msg()
    camera.transform.translation.x, camera.transform.translation.z = .1, .25
    camera.transform.rotation.w = 1.
    static.sendTransform(camera)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution, grid.info.width, grid.info.height = .05, 160, 128
    grid.info.origin.position.x, grid.info.origin.position.y = -4., -3.2
    grid.info.origin.orientation.w = 1.
    cells = np.zeros((128, 160), dtype=np.int8)
    cells[34:95, 40] = cells[34:95, 120] = 100
    cells[34, 40:121] = cells[94, 40:121] = 100
    grid.data = cells.reshape(-1).tolist()
    # Astra 接口用例故意让二维地图晚于视觉启动，覆盖整车错峰启动的播种竞态。
    map_publish_after = time.monotonic()+(4. if cloud_source == 'astra' else 0.)
    map_sent = False
    # 从真实深度光心 (.1,0,.25) 向矩形房间投射前向射线。
    points = []
    for angle in np.linspace(-math.pi/3, math.pi/3, 480):
        distance = min(1.9/math.cos(angle), 1.5/max(abs(math.sin(angle)), 1e-9))
        for offset in np.linspace(0., .025, 6):
            points.append([distance*math.cos(angle), distance*math.sin(angle), offset])
    points = np.asarray(points, dtype=np.float32)
    last_cloud = last_visual = last_odom = -math.inf

    def start(command):
        log = open(tmp_path/f'process-{len(logs)}.log', 'w')
        logs.append(log)
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def pump(seconds):
        nonlocal last_cloud, last_visual, last_odom, map_sent
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            now = node.get_clock().now()
            if not map_sent and time.monotonic() >= map_publish_after:
                grid.header.stamp = now.to_msg()
                map_pub.publish(grid)
                map_sent = True
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = now.to_msg(), 'odom', 'base_link'
            tf.transform.rotation.w = 1.
            publish_odom = time.monotonic()-last_odom >= .05
            if publish_odom and flags['tf']:
                dynamic.sendTransform(tf)
                fusion_tf_pub.publish(TFMessage(transforms=[tf]))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now.to_msg(), 'odom', 'base_link'
            odom.pose.pose.orientation.w = 1.
            for index in (0, 7, 35):
                odom.pose.covariance[index] = .005
            if publish_odom:
                odom_pub.publish(odom)
                last_odom = time.monotonic()
            stamp = rclpy.time.Time(nanoseconds=now.nanoseconds-800_000_000).to_msg()
            if flags['cloud'] and time.monotonic()-last_cloud >= .26:
                last_cloud = time.monotonic()
                cloud = PointCloud2()
                cloud.header.stamp, cloud.header.frame_id = stamp, 'probe_depth_sensor'
                cloud.height, cloud.width = 1, len(points)
                cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(('x', 'y', 'z'))]
                cloud.point_step, cloud.row_step, cloud.is_dense = 12, len(points)*12, True
                cloud.data = points.tobytes()
                cloud_pub.publish(cloud)
            if flags['visual'] and time.monotonic()-last_visual >= .3:
                last_visual = time.monotonic()
                visual = PoseWithCovarianceStamped()
                visual.header.stamp, visual.header.frame_id = stamp, 'map'
                visual.pose.pose.position.x = flags['visual_x']
                visual.pose.pose.orientation.w = 1.
                for index in (0, 7, 35):
                    visual.pose.covariance[index] = 10. if flags['bad_visual'] else .01
                visual_pub.publish(visual)
            rclpy.spin_once(node, timeout_sec=.02)

    def until(predicate, timeout=18.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            pump(.1)
            if states and predicate(states[-1]):
                return states[-1]
            assert all(process.poll() is None for process in processes), '测试节点意外退出'
        for log in logs:
            log.flush()
        (tmp_path/'states.json').write_text(json.dumps(states, ensure_ascii=False, indent=2))
        pytest.fail(json.dumps(states[-3:], ensure_ascii=False)+ '\n' + '\n'.join(
            path.read_text()[-2200:] for path in tmp_path.glob('process-*.log')))

    try:
        start(['ros2', 'launch', 'robot_perception', 'localization_scan.launch.py',
               f'cloud_source:={cloud_source}', 'cloud_topic:=/probe/cloud', 'sensor_frame:=probe_depth_sensor'])
        start(['/opt/ros/jazzy/lib/nav2_amcl/amcl', '--ros-args', '--params-file',
               str(configs/'amcl_localization.yaml'), '-r', 'initialpose:=/localization/amcl_initialpose'])
        start(['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager', '--ros-args',
               '-r', '__node:=probe_lifecycle', '-p', 'autostart:=true', '-p', 'node_names:=[amcl]'])
        start(['ros2', 'run', 'robot_navigation', 'global_localization', '--ros-args', '--params-file',
               str(configs/'global_localization.yaml'), '-p', 'tracking_timeout:=3.0',
               '-p', 'input_dropout_timeout:=2.5', '-r', '/tf:=/probe/fusion_tf'])
        normal = until(lambda state: state['ready'] and state['source'] == 'fused')
        assert normal['scan_score'] >= .55
        flags['visual_x'] = .8
        conflict = until(lambda state: state['ready'] and state['source'] == 'visual' and state['conflict'])
        assert poses[-1].pose.pose.position.x == pytest.approx(.8, abs=.05)
        assert any(not state['ready'] for state in states), '初始确认/重定位必须禁止导航'
        before_fallback = len(states)
        flags['bad_visual'] = True
        fallback = until(lambda state: state['ready'] and state['source'] == 'amcl')
        assert any(not state['ready'] and state['state'] == 'verifying' for state in states[before_fallback:]), '明显跳变的来源切换须让网页观察到持续撤销许可'
        assert fallback['sources']['visual']['rejection'] == '定位协方差超限'
        assert abs(poses[-1].pose.pose.position.x) < .15
        flags['cloud'], flags['visual'] = False, False
        lost = until(lambda state: not state['ready'] and state['state'] == 'lost', timeout=8.)
        assert lost['odom_ready'], '测试里程计持续正常，断流不能靠输出心跳伪装定位有效'
        pump(.4)
        assert states[-1]['state'] == 'lost' and not states[-1]['ready'], '丢失状态必须持续到重新确认，不能只闪现一个周期'
        flags.update(cloud=True, visual=True, bad_visual=False, visual_x=0.)
        recovered = until(lambda state: state['ready'] and state['source'] == 'fused')
        assert recovered['matches'] >= 3
        flags['tf'] = False
        tf_lost = until(lambda state: not state['ready'] and not state['tf_ready'], timeout=5.)
        assert tf_lost['odom_ready'], '里程计消息正常不能代替实际 TF 链健康'
        flags['tf'] = True
        until(lambda state: state['ready'] and state['source'] == 'fused')
        # 人工估计必须清除两路旧确认，不能直接放行。
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id, initial.header.stamp = 'map', node.get_clock().now().to_msg()
        initial.pose.pose.orientation.w = 1.
        initial.pose.covariance[0] = initial.pose.covariance[7] = .25
        initial.pose.covariance[35] = .12
        before = len(states)
        initial_pub.publish(initial)
        pump(.3)
        assert any(not state['ready'] for state in states[before:])
        assert global_map_tfs and not unexpected_map_tfs
        assert {info.node_name for info in node.get_publishers_info_by_topic('/odom')} == {'joint_localization_probe'}
        print(json.dumps({'normal': normal, 'conflict': conflict, 'fallback': fallback,
                          'lost': lost, 'recovered': recovered}, ensure_ascii=False))
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()

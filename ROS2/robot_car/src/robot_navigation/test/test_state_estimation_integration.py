"""作用：真实 EKF 对照验证迟到视觉、三路断流与同向打滑；使用方法：独立 DDS 域设 RUN_EKF_INTEGRATION=1 后 pytest -s 本文件。"""

import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml


@pytest.mark.skipif(os.environ.get('RUN_EKF_INTEGRATION') != '1',
                    reason='需独立 DDS 域显式启用')
def test_real_visual_guard_preserves_stamp_and_drops_bad_messages():
    """真实 ROS 回调验证年龄/姿态保护，避免仅测试纯函数而遗漏节点接线。"""
    from rclpy.executors import SingleThreadedExecutor
    from robot_navigation.odometry_covariance_guard import OdometryCovarianceGuard

    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 100
    rclpy.init(args=['--ros-args', '-p', 'input_topic:=/probe/raw',
                     '-p', 'output_topic:=/probe/protected'])
    guard = OdometryCovarianceGuard()
    probe = rclpy.create_node('visual_guard_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(guard)
    executor.add_node(probe)
    received = []
    probe.create_subscription(Odometry, '/probe/protected', received.append, 10)
    publisher = probe.create_publisher(Odometry, '/probe/raw', 10)
    try:
        deadline = time.monotonic()+10
        while publisher.get_subscription_count() == 0 or guard.publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=.02)
        good_stamp = None
        for age, invalid_quaternion in [(0.8, False), (1.8, False), (0., True), (-.5, False)]:
            msg = Odometry()
            msg.header.stamp = rclpy.time.Time(nanoseconds=
                probe.get_clock().now().nanoseconds-round(age*1e9)).to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.orientation.w = 0. if invalid_quaternion else 1.
            if good_stamp is None:
                good_stamp = msg.header.stamp
            publisher.publish(msg)
            deadline = time.monotonic()+.25
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
        assert len(received) == 1
        assert received[0].header.stamp == good_stamp
        assert received[0].pose.covariance[0] == .02
        assert received[0].pose.covariance[14] == 1e6
    finally:
        executor.shutdown()
        probe.destroy_node()
        guard.destroy_node()
        rclpy.shutdown()


@pytest.mark.skipif(os.environ.get('RUN_EKF_INTEGRATION') != '1',
                    reason='需独立 DDS 域显式启用')
def test_delayed_visual_and_dropout_against_real_ekf(tmp_path):
    """无实车话题/控制输出；旧配置和新配置接收相同的三维合成观测。"""
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) >= 100, '禁止接入实车 DDS 域'
    config = yaml.safe_load((Path(__file__).parents[1]/'config/state_estimation.yaml').read_text())[
        'ekf_filter_node']['ros__parameters']
    rclpy.init()
    node = rclpy.create_node('ekf_integration_probe')
    pubs = {key: node.create_publisher(kind, '/probe/'+key, 10)
            for key, kind in [('visual', Odometry), ('wheel', Odometry), ('imu', Imu)]}
    samples = {'old': [], 'fixed': []}
    processes = []
    logs = []
    start = None

    def receive(key, msg):
        if start is not None:
            samples[key].append((time.monotonic()-start, msg))

    try:
        for key in samples:
            params = dict(config)
            params.update(odom0='/probe/visual', odom1='/probe/wheel', imu0='/probe/imu',
                          publish_tf=False, print_diagnostics=False)
            if key == 'old':
                for name in ('smooth_lagged_data', 'history_length',
                             'predict_to_current_time', 'permit_corrected_publication'):
                    params.pop(name, None)
            path = tmp_path / f'{key}.yaml'
            path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
            log = open(tmp_path/f'{key}.log', 'w')
            logs.append(log)
            processes.append(subprocess.Popen([
                '/opt/ros/jazzy/lib/robot_localization/ekf_node', '--ros-args',
                '--params-file', str(path), '-r', '__node:=probe_'+key,
                '-r', 'odometry/filtered:=/probe/'+key, '--log-level', 'error'],
                stdout=log, stderr=subprocess.STDOUT))
            node.create_subscription(Odometry, '/probe/'+key,
                                     lambda msg, key=key: receive(key, msg), 100)
        deadline = time.monotonic()+15
        while not all(pub.get_subscription_count() >= 2 for pub in pubs.values()):
            assert time.monotonic() < deadline, 'EKF 未完成发现'
            rclpy.spin_once(node, timeout_sec=0.05)
        start = time.monotonic()
        last_visual = -1.0
        next_publish = start
        # 前 3 秒静止，之后真实速度为 0.5m/s；最后 2 秒真实停车但全部观测消失。
        while (elapsed := time.monotonic()-start) < 23:
            if time.monotonic() >= next_publish:
                next_publish += 0.04
                now = node.get_clock().now()
                if not 14 <= elapsed < 17 and elapsed < 21:
                    wheel = Odometry()
                    wheel.header.stamp = now.to_msg()
                    wheel.header.frame_id = 'odom'
                    wheel.child_frame_id = 'base_link'
                    wheel.pose.pose.orientation.w = 1.
                    wheel.twist.twist.linear.x = (0. if elapsed < 3 else
                                                 0.8 if 18 <= elapsed < 20 else 0.5)
                    for index, value in [(0, .02), (7, .02), (35, .03)]:
                        wheel.twist.covariance[index] = value
                    pubs['wheel'].publish(wheel)
                if not 11 <= elapsed < 14 and elapsed < 21:
                    imu = Imu()
                    imu.header.stamp = now.to_msg()
                    imu.header.frame_id = 'base_link'
                    imu.orientation.w = 1.
                    for index in (0, 4, 8):
                        imu.orientation_covariance[index] = .03
                        imu.angular_velocity_covariance[index] = .0025
                    pubs['imu'].publish(imu)
                if elapsed-last_visual >= .25 and not 8 <= elapsed < 11 and elapsed < 21:
                    last_visual = elapsed
                    visual = Odometry()
                    visual.header.stamp = rclpy.time.Time(
                        nanoseconds=now.nanoseconds-800_000_000).to_msg()
                    visual.header.frame_id = 'odom'
                    visual.child_frame_id = 'base_link'
                    visual.pose.pose.orientation.w = 1.
                    visual.pose.pose.position.x = .5*max(0., elapsed-.8-3)
                    for index, value in [(0, .02), (7, .02), (14, .1),
                                         (21, .1), (28, .1), (35, .03)]:
                        visual.pose.covariance[index] = value
                    pubs['visual'].publish(visual)
            rclpy.spin_once(node, timeout_sec=.005)

        report = {}
        for key, rows in samples.items():
            assert len(rows) > 150
            positions = [msg.pose.pose.position.x for _, msg in rows]
            stamps = [msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9 for _, msg in rows]
            assert all(math.isfinite(x) for x in positions)
            assert all(b > a for a, b in zip(stamps, stamps[1:])), key
            errors = [abs(msg.pose.pose.position.x-.5*(t-3))
                      for t, msg in rows if 5 <= t <= 7.8]
            stages = {}
            for stage, lower, upper in [('visual_off', 8.5, 10.8), ('imu_off', 11.5, 13.8),
                                        ('wheel_off', 14.5, 16.8), ('slip', 18.5, 19.8),
                                        ('all_off', 21.5, 22.8)]:
                selected = [(t, msg) for t, msg in rows if lower <= t <= upper]
                assert len(selected) >= (8 if key == 'fixed' else 4), (key, stage)
                stages[stage] = {
                    'max_x_error_m': round(max(abs(msg.pose.pose.position.x-
                        .5*(min(t, 21)-3)) for t, msg in selected), 4),
                    'last_vx': round(selected[-1][1].twist.twist.linear.x, 4),
                    'last_x_variance': round(selected[-1][1].pose.covariance[0], 4)}
            report[key] = {'steady_mean_error_m': statistics.mean(errors),
                           'max_step_m': max(abs(b-a) for a, b in zip(positions, positions[1:])),
                           'stages': stages}
        print(json.dumps(report, indent=2))
        (tmp_path/'report.json').write_text(json.dumps(report, indent=2))
        assert report['fixed']['steady_mean_error_m'] < .08
        assert report['fixed']['steady_mean_error_m'] < report['old']['steady_mean_error_m']*.5
        assert report['fixed']['max_step_m'] < .3
        # 只验证短时平直运动的有界连续性，不宣称所有运动/长时间断流都可保持精度。
        for stage in ('visual_off', 'imu_off', 'wheel_off'):
            assert report['fixed']['stages'][stage]['max_x_error_m'] < .3
        # 全断流时仍会外推，不能把输出新鲜误判为传感器健康或自动停车。
        assert report['fixed']['stages']['all_off']['last_vx'] > .1
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()

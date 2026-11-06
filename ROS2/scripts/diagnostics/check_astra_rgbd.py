#!/usr/bin/env python3
"""作用：仅采样 Astra RGB-D/点云并可测试相机自身里程计；用法：独立 ROS_DOMAIN_ID>=100 下 python3 check_astra_rgbd.py --start-camera --test-odometry，不启动底盘。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from rtabmap_msgs.msg import RGBDImage
from ament_index_python.packages import get_package_share_directory
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=25.)
    parser.add_argument('--output', default='/tmp/astra-rgbd-report.json')
    parser.add_argument('--start-camera', action='store_true')
    parser.add_argument('--test-odometry', action='store_true')
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) < 100:
        parser.error('请使用独立 ROS_DOMAIN_ID>=100，禁止接入整车通信域')
    rclpy.init()
    node = rclpy.create_node('astra_camera_validation')
    report = {'counts': {}, 'details': {}, 'valid_visual_odometry': 0,
              'note': '相机自身里程计以 astra_link 为机身，仅验证传感器链路，不代表整车安装已标定'}
    def receive(topic, message):
        report['counts'][topic] = report['counts'].get(topic, 0)+1
        if isinstance(message, RGBDImage):
            stamp = lambda image: image.header.stamp.sec+image.header.stamp.nanosec*1e-9
            report['details'][topic] = {'rgb_frame': message.rgb.header.frame_id,
                'depth_frame': message.depth.header.frame_id,
                'rgb_depth_skew_s': abs(stamp(message.rgb)-stamp(message.depth))}
        elif isinstance(message, Image):
            report['details'][topic] = {'frame': message.header.frame_id, 'encoding': message.encoding}
        elif isinstance(message, PointCloud2):
            report['details'][topic] = {'frame': message.header.frame_id, 'points': message.width*message.height}
        else:
            q = message.pose.pose.orientation
            if (abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .01 and
                    math.isfinite(message.pose.covariance[0]) and message.pose.covariance[0] < 1000.):
                report['valid_visual_odometry'] += 1
            report['details'][topic] = {'frame': message.header.frame_id, 'child': message.child_frame_id,
                                      'x_variance': message.pose.covariance[0]}
    for topic, kind in [('/camera/rgbd_image', RGBDImage), ('/camera/points', PointCloud2),
                        ('/camera/depth/image_rect', Image), ('/visual_odom_raw', Odometry)]:
        node.create_subscription(kind, topic, lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data)
    processes = []
    with tempfile.TemporaryDirectory(prefix='astra-check-') as temporary:
        log_path = Path(args.output).with_suffix('.log')
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log = log_path.open('w')
        def start(command):
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        try:
            if args.start_camera:
                start(['ros2', 'launch', 'robot_perception', 'astra_camera.launch.py'])
            if args.test_odometry:
                config = yaml.safe_load((Path(get_package_share_directory('robot_navigation'))/'config/rgbd_odometry.yaml').read_text())
                config['rgbd_odometry']['ros__parameters']['frame_id'] = 'astra_link'
                path = Path(temporary)/'rgbd_odometry.yaml'
                path.write_text(yaml.safe_dump(config))
                start(['ros2', 'launch', 'robot_navigation', 'rgbd_odometry.launch.py', f'config_file:={path}'])
            deadline = time.monotonic()+args.duration
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
        finally:
            for process in processes:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.close()
            node.destroy_node()
            rclpy.shutdown()
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report['counts'].get('/camera/points', 0) < 3 or report['counts'].get('/camera/rgbd_image', 0) < 3:
        raise SystemExit('RGB-D/点云未充分到达，检查报告同名 .log')
    if args.test_odometry and report['valid_visual_odometry'] < 3:
        raise SystemExit('相机视觉里程计有效帧不足，检查纹理、配准和同名 .log')
    if report['details'].get('/camera/depth/image_rect', {}).get('encoding') != '32FC1':
        raise SystemExit('统一深度未转换为语义链要求的米制 32FC1')


if __name__ == '__main__':
    main()

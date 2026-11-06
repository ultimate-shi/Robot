"""作用：融合视觉与 AMCL 地图修正并监督导航许可；使用方法：map_localization.launch.py localization_backend:=fused|amcl。"""

from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from robot_navigation.global_localization_core import (
    CorrectionFusion, MapLikelihood, compose, composed_covariance,
    correction_with_covariance, planar_covariance)
from robot_navigation.localization_guard import LocalizationState, initial_pose_message


def stamp_seconds(header):
    return header.stamp.sec+header.stamp.nanosec*1e-9


def pose_vector(pose):
    q = pose.orientation
    values = (pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w)
    if (not all(math.isfinite(v) for v in values) or
            abs(sum(v*v for v in (q.x, q.y, q.z, q.w))-1.) > .01):
        raise ValueError('位姿非有限或四元数无效')
    return np.array([pose.position.x, pose.position.y,
                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


def transform_vector(transform):
    q = transform.rotation
    return np.array([transform.translation.x, transform.translation.y,
                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


class GlobalLocalization(Node):
    def __init__(self):
        super().__init__('global_localization')
        defaults = {
            'backend': 'fused', 'visual_topic': '/rtabmap/localization_pose',
            'amcl_topic': '/amcl_pose', 'scan_topic': '/localization/scan', 'map_topic': '/map',
            'odom_topic': '/odom', 'info_topic': '/info',
            'output_topic': '/localization/fused_pose', 'map_frame': 'map',
            'odom_frame': 'odom', 'base_frame': 'base_link',
            'pose_max_age': 1.5, 'source_timeout': 3., 'tf_timeout': .02,
            'odom_timeout': 1., 'history_tolerance': .15, 'scan_pose_tolerance': .15,
            'max_xy_variance': .25, 'max_yaw_variance': .12,
            'position_variance_floor': .005, 'yaw_variance_floor': .01,
            'unobserved_variance': 1e6,
            'required_matches': 3, 'confirm_distance': .25, 'confirm_angle': .174533,
            'conflict_distance': .5, 'conflict_angle': .349066,
            'scan_min_beams': 30, 'scan_min_span': .523599, 'scan_min_score': .55,
            'scan_match_distance': .2, 'map_occupied_threshold': 65,
            'localization_timeout': 60., 'tracking_timeout': 120.,
            'tracking_max_distance': 5., 'tracking_max_angle': 6.28,
            'input_dropout_timeout': 20., 'update_rate': 10.,
            'nomotion_update_interval': 1.,
            'relocalization_hold': 1.,
            'initial_seed_retry': 5.,
            'initial_pose_enabled': False, 'initial_x': 0., 'initial_y': 0., 'initial_yaw': 0.,
            'initial_pose_xy_variance': .25, 'initial_pose_yaw_variance': .12,
        }
        for key, default in defaults.items():
            self.declare_parameter(key, default)
            setattr(self, key, self.get_parameter(key).value)
        if self.backend not in ('fused', 'amcl'):
            raise ValueError('联合定位节点 backend 只能为 fused 或 amcl')
        positive = [key for key, value in defaults.items() if isinstance(value, (float, int))
                    and not isinstance(value, bool) and key not in ('initial_x', 'initial_y', 'initial_yaw')]
        if (not all(math.isfinite(getattr(self, key)) and getattr(self, key) > 0 for key in positive) or
                self.scan_min_score > 1 or self.map_occupied_threshold > 100):
            raise ValueError('联合定位阈值必须为有限正数且匹配比例不大于 1')
        self.floor = [self.position_variance_floor]*2+[self.yaw_variance_floor]
        self.fusion = CorrectionFusion(self.required_matches, self.source_timeout,
                                       self.confirm_distance, self.confirm_angle,
                                       self.conflict_distance, self.conflict_angle)
        self.state = LocalizationState(self.localization_timeout, self.source_timeout,
            self.required_matches, self.max_xy_variance, self.max_yaw_variance,
            tracking_timeout=self.tracking_timeout, tracking_max_distance=self.tracking_max_distance,
            tracking_max_angle=self.tracking_max_angle)
        self.buffer = Buffer(cache_time=Duration(seconds=10.))
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.ready_pub = self.create_publisher(Bool, '/localization/ready', 1)
        self.status_pub = self.create_publisher(String, '/localization/status', 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.output_topic, 10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.amcl_initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/amcl_initialpose', 1)
        self.create_subscription(PoseWithCovarianceStamped, self.visual_topic,
                                 lambda msg: self.on_pose('visual', msg), 10)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_topic,
                                 lambda msg: self.on_pose('amcl', msg), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/initialpose', self.on_initial, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_external_initial, 1)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Info, self.info_topic, self.on_info, 10)
        self.create_service(Trigger, '/localization/retry', self.retry)
        self.nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.nomotion_future, self.last_nomotion = None, -math.inf
        self.history, self.scans = deque(maxlen=300), deque(maxlen=40)
        self.likelihood, self.anchor, self.last_odom = None, None, None
        self.last_update_stamp, self.last_odom_stamp = -1., -1.
        self.input_stamps = {'visual': -math.inf, 'amcl': -math.inf}
        self.rejections = {'visual': '等待有效视觉定位', 'amcl': '等待有效轮廓定位'}
        self.scan_score = None
        self.quality_covariances = {}
        self.last_selection = None
        self.relocalizing = None
        self.hold_until = 0.
        self.lost_latched = False
        self.amcl_seeded = False
        self.last_seed_time, self.amcl_seed_payload = -math.inf, None
        self.reset_stamp = self.now_seconds()
        self.internal_initial = None
        self.initial_hint = (initial_pose_message(self.initial_x, self.initial_y, self.initial_yaw,
            self.initial_pose_xy_variance, self.initial_pose_yaw_variance) if self.initial_pose_enabled else None)
        if self.initial_hint is not None:
            self.initial_hint.header.frame_id = self.map_frame
        self.pending_initial = self.initial_hint
        self.last_source, self.conflict = 'waiting', False
        self.create_timer(1/self.update_rate, self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds/1e9

    def fresh(self, stamp, age):
        return 0 <= self.now_seconds()-stamp <= age

    def on_map(self, msg):
        if (msg.header.frame_id != self.map_frame or msg.info.resolution <= 0 or
                len(msg.data) != msg.info.width*msg.info.height or not msg.data):
            self.likelihood = None
            return
        try:
            self.likelihood = MapLikelihood(np.asarray(msg.data).reshape(msg.info.height, msg.info.width),
                msg.info.resolution, pose_vector(msg.info.origin), self.map_occupied_threshold)
        except ValueError:
            self.likelihood = None

    def on_info(self, msg):
        stamp = stamp_seconds(msg.header)
        if self.fresh(stamp, self.pose_max_age):
            self.input_stamps['visual'] = stamp

    def on_scan(self, msg):
        stamp = stamp_seconds(msg.header)
        if not self.fresh(stamp, self.pose_max_age) or (self.scans and stamp <= stamp_seconds(self.scans[-1].header)):
            return
        self.scans.append(msg)
        if self.scan_endpoints(msg) is not None:
            self.input_stamps['amcl'] = stamp

    def scan_endpoints(self, scan):
        ranges = np.asarray(scan.ranges)
        if (not math.isfinite(scan.angle_increment) or scan.angle_increment <= 0 or
                not math.isfinite(scan.angle_min) or not 0 < scan.range_min < scan.range_max):
            return None
        indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max))
        if (len(indices) < self.scan_min_beams or
                (indices[-1]-indices[0])*scan.angle_increment < self.scan_min_span):
            return None
        angles = scan.angle_min+indices*scan.angle_increment
        return np.column_stack((ranges[indices]*np.cos(angles), ranges[indices]*np.sin(angles)))

    def on_odom(self, msg):
        stamp = stamp_seconds(msg.header)
        if stamp <= self.last_odom_stamp:
            return
        self.last_odom_stamp = stamp
        try:
            if (msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame or
                    not self.fresh(stamp, self.odom_timeout)):
                raise ValueError('里程计坐标或时间无效')
            pose = pose_vector(msg.pose.pose)
            covariance = planar_covariance(msg.pose.covariance, self.floor)
            if max(np.diag(covariance)[:2]) > self.max_xy_variance or covariance[2, 2] > self.max_yaw_variance:
                raise ValueError('里程计协方差超限')
            self.history.append((stamp, covariance))
            self.last_odom = (stamp, pose)
            self.state.observe_odometry(*pose)
        except ValueError:
            self.last_odom = None

    def historical(self, stamp):
        if not self.history:
            raise ValueError('没有历史里程计协方差')
        sample = min(self.history, key=lambda item: abs(item[0]-stamp))
        if abs(sample[0]-stamp) > self.history_tolerance:
            raise ValueError('采集时刻里程计协方差缺失')
        transform = self.buffer.lookup_transform(self.odom_frame, self.base_frame,
            Time(nanoseconds=round(stamp*1e9)), timeout=Duration(seconds=self.tf_timeout)).transform
        return transform_vector(transform), sample[1]

    def on_pose(self, source, msg):
        if source == 'visual' and self.backend == 'amcl':
            return
        stamp = stamp_seconds(msg.header)
        try:
            if (msg.header.frame_id != self.map_frame or stamp <= self.reset_stamp or
                    not self.fresh(stamp, self.pose_max_age)):
                raise ValueError('定位时间或坐标系无效')
            mapped = pose_vector(msg.pose.pose)
            covariance = planar_covariance(msg.pose.covariance, self.floor)
            if max(np.diag(covariance)[:2]) > self.max_xy_variance or covariance[2, 2] > self.max_yaw_variance:
                raise ValueError('定位协方差超限')
            if source == 'amcl':
                if not self.scans or self.likelihood is None:
                    raise ValueError('缺少定位扫描或静态地图')
                scan = min(self.scans, key=lambda value: abs(stamp_seconds(value.header)-stamp))
                endpoints = self.scan_endpoints(scan)
                if abs(stamp_seconds(scan.header)-stamp) > self.scan_pose_tolerance or endpoints is None:
                    raise ValueError('扫描时间未对齐或有效覆盖不足')
                sensor = self.buffer.lookup_transform(self.base_frame, scan.header.frame_id,
                    Time.from_msg(scan.header.stamp), timeout=Duration(seconds=self.tf_timeout)).transform
                self.scan_score = self.likelihood.score(endpoints, compose(mapped, transform_vector(sensor)),
                                                         self.scan_match_distance)
                if self.scan_score < self.scan_min_score:
                    raise ValueError('轮廓与地图吻合度不足')
            odom, odom_cov = self.historical(stamp)
            value, correction_cov = correction_with_covariance(mapped, covariance, odom, odom_cov)
            if source == 'amcl':
                self.amcl_seeded = True
            old = self.fusion.accepted.get(source)
            jumped = old is not None and not self.fusion.close(value, old.value,
                self.conflict_distance, self.conflict_angle)
            if jumped and (source == 'visual' or self.last_source == 'amcl'):
                self.relocalizing = source
                self.ready_pub.publish(Bool(data=False))
            accepted = self.fusion.observe(source, value, correction_cov, stamp)
            self.rejections[source] = '' if accepted else '等待连续一致的地图修正确认'
            if accepted:
                self.quality_covariances[source] = covariance
                self.update_anchor()
                if self.relocalizing == source:
                    self.relocalizing = None
        except Exception as exc:
            self.rejections[source] = str(exc)

    def update_anchor(self):
        estimate, source, conflict = self.fusion.select(self.now_seconds())
        selection = (source, tuple((key, value.stamp) for key, value in self.fusion.accepted.items()))
        if estimate is None or selection == self.last_selection:
            return
        if self.anchor is not None and not self.fusion.close(estimate.value, self.anchor,
                self.conflict_distance, self.conflict_angle):
            self.state.reset()
            self.hold_until = self.now_seconds()+self.relocalization_hold
            self.ready_pub.publish(Bool(data=False))
        self.anchor, self.last_source, self.conflict = estimate.value, source, conflict
        self.last_selection = selection
        covariance = [0.]*36
        for i, row in enumerate((0, 1, 5)):
            for j, col in enumerate((0, 1, 5)):
                covariance[row*6+col] = float(estimate.covariance[i, j])
        # 来源已分别完成三次确认，不能再把输出频率当成确认次数。
        # 修正量在远离 odom 原点时有较大的角度杠杆方差，不等同机器人地图位姿方差。
        # 导航确认使用已经检查过的来源位姿协方差，输出仍保留保守修正协方差。
        quality = self.quality_covariances.get('visual' if source in ('visual', 'fused') else 'amcl')
        quality_values = [0.]*36
        for index, value in zip((0, 7, 35), np.diag(quality)):
            quality_values[index] = float(value)
        new_measurement = round(estimate.stamp*1e9) > self.state.last_stamp
        self.state.observe(quality_values, round(estimate.stamp*1e9))
        if new_measurement and self.state.last_match is not None:
            self.state.last_match -= max(0., self.now_seconds()-estimate.stamp)
        self.state.count = self.required_matches
        if estimate.stamp <= self.last_update_stamp:
            return
        self.last_update_stamp = estimate.stamp
        odom, odom_cov = self.historical(estimate.stamp)
        mapped = compose(self.anchor, odom)
        output_covariance = composed_covariance(self.anchor, estimate.covariance, odom, odom_cov)
        for i, row in enumerate((0, 1, 5)):
            for j, col in enumerate((0, 1, 5)):
                covariance[row*6+col] = float(output_covariance[i, j])
        message = initial_pose_message(*mapped, self.position_variance_floor, self.yaw_variance_floor)
        message.header.frame_id = self.map_frame
        message.header.stamp = Time(nanoseconds=round(estimate.stamp*1e9)).to_msg()
        message.pose.covariance = covariance
        for index in (14, 21, 28):
            message.pose.covariance[index] = self.unobserved_variance
        self.pose_pub.publish(message)

    def reset(self):
        self.fusion.reset()
        self.state.reset()
        self.anchor = None
        self.reset_stamp = self.now_seconds()
        self.last_update_stamp = -1.
        self.last_selection, self.relocalizing = None, None
        self.hold_until = 0.
        self.lost_latched = False
        self.quality_covariances.clear()
        self.amcl_seeded = False
        self.last_seed_time, self.amcl_seed_payload = -math.inf, None
        self.last_source, self.conflict = 'waiting', False
        self.ready_pub.publish(Bool(data=False))

    def on_initial(self, msg):
        try:
            pose_vector(msg.pose.pose)
            planar_covariance(msg.pose.covariance, self.floor)
            if msg.header.frame_id != self.map_frame:
                raise ValueError('初始位置必须使用地图坐标')
        except ValueError as exc:
            self.get_logger().warning(f'拒绝初始估计：{exc}')
            return False
        self.initial_hint, self.pending_initial = msg, msg
        self.reset()
        self.amcl_seed_payload = msg
        return True

    def on_external_initial(self, msg):
        if self.internal_initial is not None and msg == self.internal_initial:
            return
        if not self.on_initial(msg):
            return
        # 标准 /initialpose 已直接送达两个定位器，不再重复转发。
        self.pending_initial = None
        self.amcl_initial_pub.publish(msg)
        self.last_seed_time = self.now_seconds()

    def retry(self, request, response):
        del request
        self.reset()
        self.pending_initial = self.initial_hint
        response.success, response.message = True, '已清除旧确认，等待新的地图匹配'
        return response

    def tick(self):
        now = self.now_seconds()
        odom_ready = self.last_odom is not None and self.fresh(self.last_odom[0], self.odom_timeout)
        input_ready = any(self.fresh(stamp, self.source_timeout) for stamp in self.input_stamps.values())
        if (self.pending_initial is not None and odom_ready and input_ready and
                self.initial_pub.get_subscription_count() >= (2 if self.backend == 'fused' else 1) and
                self.amcl_initial_pub.get_subscription_count()):
            self.pending_initial.header.stamp = self.get_clock().now().to_msg()
            self.internal_initial = self.pending_initial
            self.initial_pub.publish(self.internal_initial)
            self.amcl_initial_pub.publish(self.internal_initial)
            self.amcl_seed_payload = self.internal_initial
            self.last_seed_time = now
            self.pending_initial = None
        # 静止时 AMCL 默认不更新；定期请求下一帧更新，才能连续确认和检测环境变化。
        if (self.fresh(self.input_stamps['amcl'], self.source_timeout) and now-self.last_nomotion >= self.nomotion_update_interval and
                (self.nomotion_future is None or self.nomotion_future.done()) and self.nomotion.service_is_ready()):
            self.nomotion_future = self.nomotion.call_async(Empty.Request())
            self.last_nomotion = now
        estimate, source, conflict = self.fusion.select(now)
        self.conflict = conflict
        if estimate is not None:
            self.update_anchor()
            self.last_source = source
        # 地图服务可能比视觉晚启动；首次播种丢失时重试，收到合格 AMCL 位姿才停止。
        # 仅播种 AMCL 私有入口，不回送视觉定位器、不把初始化当作匹配确认。
        if (not self.amcl_seeded and self.likelihood is not None and odom_ready and
                self.fresh(self.input_stamps['amcl'], self.source_timeout) and
                now-self.last_seed_time >= self.initial_seed_retry and self.amcl_initial_pub.get_subscription_count()):
            seed = self.amcl_seed_payload
            if estimate is not None and source in ('visual', 'fused') and self.anchor is not None:
                current = compose(self.anchor, self.last_odom[1])
                seed = initial_pose_message(*current, self.initial_pose_xy_variance, self.initial_pose_yaw_variance)
            if seed is not None:
                seed.header.frame_id, seed.header.stamp = self.map_frame, self.get_clock().now().to_msg()
                self.amcl_initial_pub.publish(seed)
                self.last_seed_time = now
        try:
            body_tf = self.buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            body_tf_ready = self.fresh(stamp_seconds(body_tf.header), self.odom_timeout)
        except Exception:
            body_tf_ready = False
        tf_ready = self.anchor is not None and odom_ready and body_tf_ready
        if tf_ready:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = self.map_frame, self.odom_frame
            transform.transform.translation.x, transform.transform.translation.y = map(float, self.anchor[:2])
            transform.transform.rotation.z, transform.transform.rotation.w = math.sin(self.anchor[2]/2), math.cos(self.anchor[2]/2)
            self.broadcaster.sendTransform(transform)
        tolerate = any(self.fresh(stamp, self.input_dropout_timeout) for stamp in self.input_stamps.values())
        status = self.state.snapshot(tf_ready, tracking_ready=odom_ready and tolerate)
        pending_jump = (self.relocalizing is not None and self.relocalizing in self.fusion.pending and
                        self.fresh(self.fusion.pending[self.relocalizing].stamp, self.source_timeout))
        holding = now < self.hold_until
        if pending_jump or holding:
            status.update(ready=False, state='verifying', message='定位出现明显跳变，等待连续匹配重新确认')
        if not odom_ready or (not tolerate and self.state.was_ready):
            status.update(ready=False, state='lost' if self.state.was_ready else 'waiting',
                          message='里程计失效或定位输入持续中断，已禁止自主运动')
        status['message'] = status['message'].replace('视觉', '地图').replace('地图地图', '地图')
        if status['ready']:
            label = {'visual': '视觉定位', 'amcl': '轮廓定位', 'fused': '视觉与轮廓联合定位'}
            if estimate is not None:
                status['message'] = label[source]+('；轮廓结果冲突，采用通过质量检查的视觉结果' if conflict else '有效')
        status.update(backend=self.backend, source=source if estimate is not None else ('tracking' if self.anchor is not None else 'waiting'),
            conflict=conflict, scan_score=self.scan_score, odom_ready=odom_ready,
            input_fresh=input_ready, tf_ready=tf_ready,
            sources={key: {'accepted_age_seconds': now-value.stamp if (value := self.fusion.accepted.get(key)) else None,
                          'confirmations': self.fusion.counts.get(key, 0), 'rejection': self.rejections[key]}
                     for key in ('visual', 'amcl')})
        if self.initial_hint is not None:
            pose = pose_vector(self.initial_hint.pose.pose)
            status['initial_pose'] = dict(zip(('x', 'y', 'yaw'), map(float, pose)))
        if not status['ready'] and self.state.was_ready and not pending_jump and not holding:
            self.lost_latched = True
            self.state.reset()
            self.fusion.reset()
            self.last_selection = None
            self.reset_stamp = now
        if status['ready']:
            self.lost_latched = False
        elif self.lost_latched and not pending_jump and not holding:
            status.update(state='lost', message='定位丢失，已禁止自主运动；等待新的有效地图匹配重新确认')
        self.ready_pub.publish(Bool(data=status['ready']))
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

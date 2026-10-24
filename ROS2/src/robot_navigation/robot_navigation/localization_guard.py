"""作用：监督已有地图的视觉定位并发布导航许可；使用方法：由 map_localization.launch.py 启动。"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def initial_pose_message(x, y, yaw, xy_variance, yaw_variance):
    """构造地图中的平面初始估计；它不代表已成功定位。"""
    if not all(math.isfinite(v) for v in (x, y, yaw, xy_variance, yaw_variance)):
        raise ValueError('启动初始估计必须为有限数值')
    if xy_variance <= 0 or yaw_variance <= 0:
        raise ValueError('初始估计协方差必须为正数')
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.pose.covariance[0] = msg.pose.covariance[7] = xy_variance
    msg.pose.covariance[35] = yaw_variance
    return msg


class LocalizationState:
    """只认可新鲜、连续且协方差合格的地图匹配，人工初始估计不能直接放行。"""

    def __init__(self, timeout, match_timeout, required_matches, max_xy_variance,
                 max_yaw_variance, clock=time.monotonic):
        self.clock = clock
        self.timeout = timeout
        self.match_timeout = match_timeout
        self.required_matches = required_matches
        self.max_xy_variance = max_xy_variance
        self.max_yaw_variance = max_yaw_variance
        self.reset()

    def reset(self):
        self.started = self.clock()
        self.last_match = None
        self.count = 0
        self.was_ready = False
        self.last_stamp = -1

    def observe(self, covariance, stamp):
        now = self.clock()
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        values = [covariance[i] for i in (0, 7, 35)]
        if (not all(math.isfinite(v) and v >= 0 for v in values) or
                max(values[:2]) > self.max_xy_variance or
                values[2] > self.max_yaw_variance):
            self.count = 0
            self.last_match = None
            return
        if self.last_match is None or now - self.last_match > self.match_timeout:
            self.count = 0
        self.count = min(self.required_matches, self.count + 1)
        self.last_match = now

    def snapshot(self, tf_ready):
        now = self.clock()
        fresh = self.last_match is not None and now - self.last_match <= self.match_timeout
        ready = fresh and self.count >= self.required_matches and tf_ready
        if ready:
            self.was_ready = True
            state, message = 'localized', '定位成功，可以规划并确认导航'
        elif self.was_ready:
            state, message = 'lost', '定位丢失，已禁止自主运动；请检查相机并重新设置初始位姿'
        elif now - self.started >= self.timeout:
            state, message = 'failed', '定位超时；请确认地图正确、相机正常，将车移至建图时有纹理的区域并按原朝向重试'
        elif fresh and self.count:
            state, message = 'verifying', '正在连续确认地图匹配及坐标变换'
        else:
            state, message = 'waiting', '等待视觉地图匹配；可在地图上设置机器人位置和朝向'
        return {'state': state, 'ready': bool(ready), 'message': message,
                'matches': self.count, 'required_matches': self.required_matches,
                'elapsed_seconds': round(now - self.started, 1),
                'match_age_seconds': round(now - self.last_match, 1) if self.last_match is not None else None}


class LocalizationGuard(Node):
    def __init__(self):
        super().__init__('localization_guard')
        defaults = {'localization_timeout': 60.0, 'match_timeout': 5.0,
                    'required_matches': 3, 'max_xy_variance': 0.25,
                    'max_yaw_variance': 0.12, 'tf_timeout': 1.0,
                    'update_rate': 5.0,
                    'pose_topic': '/rtabmap/localization_pose',
                    'initial_pose_topic': '/localization/initialpose',
                    'info_topic': '/info', 'input_ready_timeout': 120.0,
                    'initial_pose_enabled': False,
                    'initial_x': 0.0, 'initial_y': 0.0, 'initial_yaw': 0.0,
                    'initial_pose_xy_variance': 0.25, 'initial_pose_yaw_variance': 0.12}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        values = {key: self.get_parameter(key).value for key in defaults}
        for key in ('localization_timeout', 'match_timeout', 'required_matches',
                    'max_xy_variance', 'max_yaw_variance', 'tf_timeout', 'update_rate',
                    'input_ready_timeout'):
            if not math.isfinite(values[key]) or values[key] <= 0:
                raise ValueError(f'{key} 必须为正有限值')
        self.state = LocalizationState(*(values[key] for key in (
            'localization_timeout', 'match_timeout', 'required_matches',
            'max_xy_variance', 'max_yaw_variance')))
        self.tf_timeout = values['tf_timeout']
        self.reset_stamp = self.get_clock().now().nanoseconds
        self.input_started = time.monotonic()
        self.input_ready = False
        self.last_info = None
        self.input_ready_timeout = values['input_ready_timeout']
        self.startup_seed = (initial_pose_message(*(values[key] for key in (
            'initial_x', 'initial_y', 'initial_yaw', 'initial_pose_xy_variance',
            'initial_pose_yaw_variance'))) if values['initial_pose_enabled'] else None)
        self.initial_hint = self.startup_seed
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.ready_pub = self.create_publisher(Bool, '/localization/ready', 1)
        self.status_pub = self.create_publisher(String, '/localization/status', 1)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(PoseWithCovarianceStamped, values['pose_topic'], self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, values['initial_pose_topic'], self.on_initial, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_external_initial, 1)
        self.create_subscription(Info, values['info_topic'], self.on_info, 10)
        self.create_service(Trigger, '/localization/retry', self.retry)
        self.create_timer(1.0 / values['update_rate'], self.publish_status)

    def reset(self):
        self.state.reset()
        self.reset_stamp = self.get_clock().now().nanoseconds
        self.ready_pub.publish(Bool(data=False))
        self.publish_status()

    def on_initial(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        if (msg.header.frame_id != 'map' or
                not all(math.isfinite(v) for v in (pose.position.x, pose.position.y,
                                                    q.x, q.y, q.z, q.w)) or
                abs(sum(v*v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.01):
            self.get_logger().warning('拒绝无效初始位姿')
            return
        self.startup_seed = msg  # 最新人工估计覆盖启动估计，处理链未就绪时排队等待。
        self.initial_hint = msg
        self.reset()

    def retry(self, request, response):
        del request
        self.startup_seed = self.initial_hint
        self.reset()
        response.success = True
        response.message = '已重新开始定位确认；仍需相机匹配成功，请勿将人工估计当作定位结果'
        return response

    def on_external_initial(self, msg):
        # 兼容 RViz 标准入口；外部初始估计同样必须撤销旧许可。
        if self.initial_hint is None or msg != self.initial_hint:
            self.startup_seed = msg if not self.input_ready else None
            self.initial_hint = msg
            self.reset()

    def on_pose(self, msg):
        if not self.input_ready:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        pose = msg.pose.pose
        if (msg.header.frame_id != 'map' or stamp <= self.reset_stamp or
                not 0 <= age <= self.state.match_timeout or
                not all(math.isfinite(v) for v in (pose.position.x, pose.position.y,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))):
            return
        self.state.observe(msg.pose.covariance, stamp)
        self.publish_status()

    def on_info(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0 <= age <= self.state.match_timeout:
            return
        self.last_info = time.monotonic()
        if not self.input_ready:
            # Info 由 RTAB-Map 真正处理过双目帧后发布，不能仅凭订阅者已发现判断已就绪。
            self.input_ready = True
            self.state.reset()
            self.reset_stamp = self.get_clock().now().nanoseconds
            self.publish_status()

    def try_startup_seed(self):
        if (self.startup_seed is None or not self.input_ready or self.last_info is None or
                time.monotonic()-self.last_info > self.state.match_timeout or
                self.initial_pub.get_subscription_count() == 0):
            return
        try:
            transform = self.buffer.lookup_transform('odom', 'base_link', Time())
            stamp = transform.header.stamp
            age = (self.get_clock().now().nanoseconds - stamp.sec*1_000_000_000 - stamp.nanosec) / 1e9
            if not 0 <= age <= self.tf_timeout:
                return
        except Exception:
            return
        msg = self.startup_seed
        self.startup_seed = None
        msg.header.stamp = self.get_clock().now().to_msg()
        self.state.reset()
        self.reset_stamp = self.get_clock().now().nanoseconds
        self.initial_pub.publish(msg)
        self.get_logger().info('双目定位处理已就绪，已向 RTAB-Map 发送初始估计，等待匹配确认')

    def publish_status(self):
        self.try_startup_seed()
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp
            age = (self.get_clock().now().nanoseconds - stamp.sec*1_000_000_000 - stamp.nanosec) / 1e9
            tf_ready = 0 <= age <= self.tf_timeout
        except Exception:
            tf_ready = False
        status = self.state.snapshot(tf_ready)
        if not self.input_ready:
            elapsed = time.monotonic()-self.input_started
            status.update(ready=False, state='initializing',
                          message=('正在加载视觉地图或等待双目处理就绪' +
                                   ('，初始估计将在就绪后发送' if self.startup_seed is not None else '')),
                          elapsed_seconds=round(elapsed, 1))
            if elapsed >= self.input_ready_timeout:
                status.update(state='failed', message='视觉定位输入未就绪，请检查相机同步、数据库加载和处理节点')
        status['input_ready'] = self.input_ready
        status['initial_pose_pending'] = self.startup_seed is not None
        hint = self.initial_hint.pose.pose if self.initial_hint is not None else None
        status['initial_pose'] = ({'x': hint.position.x, 'y': hint.position.y,
                                  'yaw': 2 * math.atan2(hint.orientation.z, hint.orientation.w)}
                                 if hint is not None else None)
        self.ready_pub.publish(Bool(data=status['ready']))
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

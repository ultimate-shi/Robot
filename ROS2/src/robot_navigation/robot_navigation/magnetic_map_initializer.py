"""使用方法：静态地图磁航向模式下运行本节点；校准磁姿态与 EKF TF 就绪后只发布一次 map→odom。"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def yaw_of(quaternion):
    """从单位四元数读取 ENU 偏航角。"""
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def multiply(a, b):
    """组合两个 xyzw 四元数。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_of(message):
    q = message
    components = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(value) for value in components):
        return None
    norm = math.sqrt(sum(value * value for value in components))
    if norm < 0.5 or norm > 1.5:
        return None
    return tuple(value / norm for value in components)


def map_to_odom_yaw(magnetic_east_yaw_in_map, magnetic_imu, imu_base,
                    odom_base):
    """将磁 ENU 姿态换到 base_link，再抵消 EKF 的 odom 姿态。"""
    map_base = magnetic_east_yaw_in_map + yaw_of(multiply(magnetic_imu, imu_base))
    return math.atan2(math.sin(map_base - yaw_of(odom_base)),
                      math.cos(map_base - yaw_of(odom_base)))


def map_to_odom_translation(initial_x, initial_y, yaw, odom_x, odom_y):
    """让当前 base_link 位于地图指定初始点，而非假定 odom 原点。"""
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (initial_x - cosine * odom_x + sine * odom_y,
            initial_y - sine * odom_x - cosine * odom_y)


class MagneticMapInitializer(Node):
    """仅在所有定位前提成立时锁存静态地图航向。"""

    def __init__(self):
        super().__init__('magnetic_map_initializer')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('magnetic_east_yaw_in_map', float('nan'))
        self.declare_parameter('startup_timeout_sec', 20.0)
        self.declare_parameter('max_imu_age_sec', 0.5)
        self.declare_parameter('max_odom_tf_age_sec', 0.5)
        self.east_yaw = float(self.get_parameter('magnetic_east_yaw_in_map').value)
        if not math.isfinite(self.east_yaw):
            raise ValueError('magnetic_east_yaw_in_map 必须显式设置为有限角度')
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.max_imu_age = float(self.get_parameter('max_imu_age_sec').value)
        self.max_odom_age = float(self.get_parameter('max_odom_tf_age_sec').value)
        if (self.timeout <= 0.0 or self.max_imu_age <= 0.0 or
                self.max_odom_age <= 0.0 or
                not all(map(math.isfinite, (
                    self.initial_x, self.initial_y, self.timeout,
                    self.max_imu_age, self.max_odom_age)))):
            raise ValueError('初始化位置、时限和数据有效期参数不合法')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.sample = None
        self.ready = False
        self.failed = False
        self.started = time.monotonic()
        self.create_subscription(Imu, '/sensors/imu/vendor_data', self.on_imu, 10)
        self.create_timer(0.05, self.try_initialize)

    def on_imu(self, message):
        """厂家姿态仅在驱动标记校准有效且时间戳新鲜时接受。"""
        covariance = message.orientation_covariance[0]
        if (not math.isfinite(covariance) or covariance < 0.0 or
                not message.header.frame_id):
            return
        quaternion = quaternion_of(message.orientation)
        if quaternion is None:
            return
        stamp = Time.from_msg(message.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if 0.0 <= age <= self.max_imu_age:
            self.sample = (message.header.frame_id, quaternion, stamp)

    def try_initialize(self):
        """TF 不齐备时持续等待；超时明确失败且不发布变换。"""
        if self.ready or self.failed:
            return
        if time.monotonic() - self.started > self.timeout:
            self.get_logger().error('磁航向初始化超时；Nav2 导航不会启动')
            self.failed = True
            return
        if self.sample is None:
            return
        imu_frame, magnetic_imu, stamp = self.sample
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age < 0.0 or age > self.max_imu_age:
            self.sample = None
            return
        try:
            imu_base_tf = self.buffer.lookup_transform(imu_frame, 'base_link', Time())
            odom_base_tf = self.buffer.lookup_transform('odom', 'base_link', Time())
        except Exception:
            return
        odom_stamp = Time.from_msg(odom_base_tf.header.stamp)
        odom_age = (self.get_clock().now() - odom_stamp).nanoseconds * 1e-9
        if odom_age < 0.0 or odom_age > self.max_odom_age:
            return
        imu_base = quaternion_of(imu_base_tf.transform.rotation)
        odom_base = quaternion_of(odom_base_tf.transform.rotation)
        if imu_base is None or odom_base is None:
            return
        yaw = map_to_odom_yaw(self.east_yaw, magnetic_imu, imu_base, odom_base)
        odom_position = odom_base_tf.transform.translation
        if not all(map(math.isfinite, (odom_position.x, odom_position.y))):
            return
        map_odom_x, map_odom_y = map_to_odom_translation(
            self.initial_x, self.initial_y, yaw,
            odom_position.x, odom_position.y)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = map_odom_x
        transform.transform.translation.y = map_odom_y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(transform)
        self.ready = True
        self.get_logger().info('磁航向初始化完成：map→odom yaw=%.4f rad' % yaw)
        print('MAGNETIC_MAP_READY', flush=True)


def main(args=None):
    """保持静态 TF 发布者存活，直到 launch 结束。"""
    rclpy.init(args=args)
    node = None
    try:
        node = MagneticMapInitializer()
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.failed:
            raise SystemExit(2)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

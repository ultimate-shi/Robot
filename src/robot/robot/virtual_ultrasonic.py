#!/home/shijiahao/ros2_pythonenv/bin/python
# -*- coding: utf-8 -*-

"""
ROS2 Jazzy 虚拟超声波传感器模拟节点（最终版）
---------------------------------------------------
功能：
1. 读取 PLY 点云地图（真实环境）
2. 使用 scipy cKDTree 做高速邻域搜索
3. 根据 TF 获取 8 个超声波传感器在 map 下位置姿态
4. 实时计算距离
5. 发布标准 sensor_msgs/Range 消息
6. 支持 RViz / Foxglove / Nav2 联调

依赖安装：
pip install scipy plyfile numpy transforms3d

运行：
ros2 run robot virtual_ultrasonic.py

---------------------------------------------------
你只需修改：
1. PLY_PATH
2. 8个传感器名字（如果和URDF不一致）
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

import tf2_ros
from rclpy.executors import ExternalShutdownException

from scipy.spatial import cKDTree
from plyfile import PlyData


# =========================================================
# 8个超声波传感器配置（与你URDF一致）
# =========================================================
SENSORS = [
    {"link": "radar-front_fl", "topic": "/ultrasonic/front_fl"},
    {"link": "radar-front_fr", "topic": "/ultrasonic/front_fr"},
    {"link": "radar-front_rl", "topic": "/ultrasonic/front_rl"},
    {"link": "radar-front_rr", "topic": "/ultrasonic/front_rr"},
    {"link": "radar-side_fl",  "topic": "/ultrasonic/side_fl"},
    {"link": "radar-side_fr",  "topic": "/ultrasonic/side_fr"},
    {"link": "radar-side_rl",  "topic": "/ultrasonic/side_rl"},
    {"link": "radar-side_rr",  "topic": "/ultrasonic/side_rr"},
]


# =========================================================
# 主节点
# =========================================================
class VirtualUltrasonic(Node):

    def __init__(self):
        super().__init__("virtual_ultrasonic")

        # =================================================
        # 参数
        # =================================================
        pkg_path = get_package_share_directory("robot")

        self.PLY_PATH = os.path.join(
            pkg_path,
            "map",
            "studyroom.ply"
        )

        self.PLY_PATH = os.path.abspath(self.PLY_PATH)

        self.MAX_RANGE = 4.0
        self.MIN_RANGE = 0.02

        # 超声波水平半角（15°）
        self.FOV_HALF = math.radians(15.0)

        # 检测高度范围（地面以上）
        self.MIN_HEIGHT = 0.03
        self.MAX_HEIGHT = 0.60

        # 发布频率
        self.TIMER_PERIOD = 0.2   # 10Hz

        # =================================================
        # 加载点云
        # =================================================
        self.load_point_cloud()

        # =================================================
        # TF
        # =================================================
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=Duration(seconds=10.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =================================================
        # 发布器
        # =================================================
        self.sensor_publishers = {}

        for sensor in SENSORS:
            pub = self.create_publisher(
                Range,
                sensor["topic"],
                10
            )
            self.sensor_publishers[sensor["link"]] = pub

        self.get_logger().info("8路超声波发布器已创建")

        # =================================================
        # 定时器
        # =================================================
        self.create_timer(
            self.TIMER_PERIOD,
            self.publish_all
        )

        self.get_logger().info("Virtual Ultrasonic Node Started")

    # =====================================================
    # 加载PLY地图
    # =====================================================
    def load_point_cloud(self):

        self.get_logger().info(f"读取PLY地图: {self.PLY_PATH}")

        ply = PlyData.read(self.PLY_PATH)
        vertex = ply["vertex"]

        self.points = np.vstack([
            vertex["x"],
            vertex["y"],
            vertex["z"]
        ]).T.astype(np.float32)

        self.get_logger().info(
            f"原始点数: {len(self.points)}"
        )

        # -------------------------------------------------
        # 简单降采样（体素化）
        # -------------------------------------------------
        voxel = 0.03  # 3cm

        grid = np.floor(self.points / voxel).astype(np.int32)

        _, idx = np.unique(grid, axis=0, return_index=True)

        self.points = self.points[idx]

        self.get_logger().info(
            f"降采样后点数: {len(self.points)}"
        )

        # KDTree
        self.kdtree = cKDTree(self.points)

        self.get_logger().info("KDTree建立完成")


    # =====================================================
    # 获取单个传感器距离
    # =====================================================
    def get_sensor_distance(self, link_name):

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                link_name,
                rclpy.time.Time()
            )

        except Exception:
            return self.MAX_RANGE

        # -------------------------------------------------
        # 传感器位置
        # -------------------------------------------------
        sx = tf.transform.translation.x
        sy = tf.transform.translation.y
        sz = tf.transform.translation.z

        sensor_pos = np.array([sx, sy, sz])

        # -------------------------------------------------
        # 四元数 -> 朝向
        # transforms3d顺序是 [w x y z]
        # -------------------------------------------------
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w

        rot = self.quat_to_rotmat(qw, qx, qy, qz)

        # 本体x轴 = 前方方向
        front = rot[:, 0]

        fx = front[0]
        fy = front[1]

        front_2d = np.array([fx, fy])

        norm = np.linalg.norm(front_2d)

        if norm < 1e-6:
            return self.MAX_RANGE

        front_2d /= norm

        # -------------------------------------------------
        # 搜索半径内点
        # -------------------------------------------------
        ids = self.kdtree.query_ball_point(
            sensor_pos,
            self.MAX_RANGE
        )

        if len(ids) == 0:
            return self.MAX_RANGE

        near_points = self.points[ids]

        min_dist = self.MAX_RANGE

        # -------------------------------------------------
        # 遍历候选点
        # -------------------------------------------------
        for p in near_points:

            px, py, pz = p

            # 高度过滤
            if pz < self.MIN_HEIGHT or pz > self.MAX_HEIGHT:
                continue

            dx = px - sx
            dy = py - sy

            horizontal_dist = math.sqrt(dx * dx + dy * dy)

            if horizontal_dist < self.MIN_RANGE:
                continue

            if horizontal_dist > self.MAX_RANGE:
                continue

            # 方向角判断
            vec = np.array([dx, dy])

            vnorm = np.linalg.norm(vec)

            if vnorm < 1e-6:
                continue

            vec /= vnorm

            dot = np.dot(front_2d, vec)
            dot = np.clip(dot, -1.0, 1.0)

            angle = math.acos(dot)

            if angle > self.FOV_HALF:
                continue

            if horizontal_dist < min_dist:
                min_dist = horizontal_dist

        return round(min_dist, 3)

    # =====================================================
    # 计算四元数旋转矩阵
    # =====================================================
    def quat_to_rotmat(self, w, x, y, z):
        return np.array([
            [1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ], dtype=np.float64)
    

    # =====================================================
    # 发布所有传感器
    # =====================================================
    def publish_all(self):

        stamp = self.get_clock().now().to_msg()

        for sensor in SENSORS:

            link = sensor["link"]

            dist = self.get_sensor_distance(link)

            msg = Range()

            msg.header.stamp = stamp
            msg.header.frame_id = link

            msg.radiation_type = Range.ULTRASOUND

            msg.field_of_view = self.FOV_HALF * 2.0

            msg.min_range = self.MIN_RANGE
            msg.max_range = self.MAX_RANGE

            msg.range = dist

            self.sensor_publishers[link].publish(msg)


# =========================================================
# main
# =========================================================
def main(args=None):

    rclpy.init(args=args)
    node = VirtualUltrasonic()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # 正常退出，不打印traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
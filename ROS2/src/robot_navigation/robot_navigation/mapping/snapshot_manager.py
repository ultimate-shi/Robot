#!/usr/bin/env python3
"""作用：成套保存地图、点云及已落盘的 SLAM 数据库；使用方法：调用 /mapping/save_snapshot。"""

import json
import math
import os
import shutil
import sqlite3
import tempfile
import threading
import time
from datetime import datetime
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from robot_navigation.mapping.database import copy_database, file_digest


def snapshot_identity(directory, owner_uid=-1, owner_gid=-1):
    """普通进程使用当前用户；容器 root 默认沿保存路径向上查找工作区用户。"""
    uid, gid = os.getuid(), os.getgid()
    if uid == 0:
        candidate = os.path.abspath(directory)
        while True:
            if os.path.exists(candidate):
                info = os.stat(candidate)
                if info.st_uid != 0:
                    uid, gid = info.st_uid, info.st_gid
                    break
            parent = os.path.dirname(candidate)
            if parent == candidate:
                break
            candidate = parent
    return (uid if owner_uid < 0 else owner_uid,
            gid if owner_gid < 0 else owner_gid)


def apply_snapshot_permissions(directory, uid, gid):
    """成套地图归目标用户所有，目录可遍历和写入，文件可读写，避免临时目录保留 0700。"""
    for root, directories, files in os.walk(directory, topdown=False, followlinks=False):
        for name in directories + files:
            path = os.path.join(root, name)
            if os.path.islink(path):
                raise ValueError('地图快照目录内不能含符号链接')
        for name in files:
            path = os.path.join(root, name)
            os.chown(path, uid, gid)
            os.chmod(path, 0o644)
        os.chown(root, uid, gid)
        os.chmod(root, 0o755)


def snapshot_time(timezone_name, now=None):
    """返回快照配置时区中的时间，避免目录名受容器本地时区影响。"""
    zone = ZoneInfo(timezone_name)
    if now is None:
        return datetime.now(zone)
    if now.tzinfo is None:
        raise ValueError('测试或调用方提供的时间必须包含时区')
    return now.astimezone(zone)


def occupancy_to_pgm(grid):
    """将 ROS OccupancyGrid 转成 map_server 使用的 PGM 字节。"""
    width = int(grid.info.width)
    height = int(grid.info.height)
    values = np.asarray(grid.data, dtype=np.int16)
    if width <= 0 or height <= 0 or values.size != width * height:
        raise ValueError('OccupancyGrid 尺寸或数据长度无效')
    values = values.reshape((height, width))
    image = np.full((height, width), 205, dtype=np.uint8)
    # ROS 使用 -1 表示未知区域，不能因其小于空闲阈值而误标为空闲。
    image[(values >= 0) & (values <= 25)] = 254
    image[values >= 65] = 0
    # OccupancyGrid 从左下角开始，PGM 从左上角开始，写盘前需上下翻转。
    image = np.flipud(image)
    header = f'P5\n# robot mapping snapshot\n{width} {height}\n255\n'
    return header.encode('ascii') + image.tobytes()


def occupancy_yaml(grid, image_name):
    """生成兼容 nav2_map_server 的地图 YAML。"""
    origin = grid.info.origin
    q = origin.orientation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return (
        f'image: {image_name}\n'
        'mode: trinary\n'
        f'resolution: {float(grid.info.resolution):.9g}\n'
        f'origin: [{origin.position.x:.9g}, {origin.position.y:.9g}, '
        f'{yaw:.9g}]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    )


def cloud_to_xyz(msg):
    """按 PointCloud2 字段偏移提取有限的 XYZ 点。"""
    fields = {field.name: field for field in msg.fields}
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('PointCloud2 缺少 x/y/z 字段')
    type_map = {
        PointField.INT8: 'i1', PointField.UINT8: 'u1',
        PointField.INT16: 'i2', PointField.UINT16: 'u2',
        PointField.INT32: 'i4', PointField.UINT32: 'u4',
        PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
    }
    endian = '>' if msg.is_bigendian else '<'
    names, formats, offsets = [], [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in type_map or field.count != 1:
            raise ValueError(f'PointCloud2 字段 {name} 类型不受支持')
        names.append(name)
        formats.append(np.dtype(endian + type_map[field.datatype]))
        offsets.append(field.offset)
    dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg.point_step,
    })
    structured = np.ndarray(
        shape=(msg.height, msg.width), dtype=dtype, buffer=msg.data,
        strides=(msg.row_step, msg.point_step),
    )
    points = np.column_stack([
        structured[name].reshape(-1) for name in ('x', 'y', 'z')
    ]).astype(np.float32, copy=False)
    return np.ascontiguousarray(points[np.isfinite(points).all(axis=1)])


def write_binary_ply(path, points):
    """以 binary_little_endian PLY 写出 XYZ 点，减少快照体积。"""
    little = np.asarray(points, dtype='<f4')
    header = (
        'ply\nformat binary_little_endian 1.0\n'
        f'element vertex {len(little)}\n'
        'property float x\nproperty float y\nproperty float z\n'
        'end_header\n'
    ).encode('ascii')
    with open(path, 'wb') as stream:
        stream.write(header)
        stream.write(little.tobytes())


class SnapshotManager(Node):
    """缓存最近地图和点云，并由显式服务调用创建快照。"""

    def __init__(self):
        super().__init__('mapping_snapshot_manager')
        defaults = {
            'map_topic': '/map',
            'cloud_topic': '/mapping/cloud_map',
            'preview_directory': '/tmp/robot_preview',
            'save_directory': '/workspace/maps',
            'snapshot_basename': 'current',
            'snapshot_timezone': 'UTC',
            'database_path': '/tmp/robot_rtabmap_mapping.db',
            'database_service_timeout': 60.0,
            'resume_service_timeout': 5.0,
            'backup_service': '/rtabmap/backup',
            'pause_service': '/rtabmap/pause',
            'resume_service': '/rtabmap/resume',
            'map_service': '/rtabmap/get_map',
            'snapshot_owner_uid': -1,
            'snapshot_owner_gid': -1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.preview_directory = str(
            self.get_parameter('preview_directory').value)
        self.save_directory = str(self.get_parameter('save_directory').value)
        self.snapshot_basename = str(
            self.get_parameter('snapshot_basename').value)
        self.snapshot_timezone = str(
            self.get_parameter('snapshot_timezone').value)
        owner_uid = int(self.get_parameter('snapshot_owner_uid').value)
        owner_gid = int(self.get_parameter('snapshot_owner_gid').value)
        if owner_uid < -1 or owner_gid < -1:
            raise ValueError('快照属主 UID/GID 必须是 -1 或非负整数')
        self.snapshot_uid, self.snapshot_gid = snapshot_identity(
            self.save_directory, owner_uid, owner_gid)
        # 启动时立即校验，避免保存请求到来后才暴露无效时区配置。
        snapshot_time(self.snapshot_timezone)
        self.latest_map = None
        self.latest_cloud = None
        self.database_path = str(self.get_parameter('database_path').value)
        self.database_timeout = float(self.get_parameter('database_service_timeout').value)
        self.resume_timeout = float(self.get_parameter('resume_service_timeout').value)
        self.backup_future = None
        self.client_group = ReentrantCallbackGroup()
        self.backup_client = self.create_client(
            Empty, str(self.get_parameter('backup_service').value), callback_group=self.client_group)
        self.pause_client = self.create_client(Empty, str(self.get_parameter('pause_service').value),
                                               callback_group=self.client_group)
        self.resume_client = self.create_client(Empty, str(self.get_parameter('resume_service').value),
                                                callback_group=self.client_group)
        self.map_client = self.create_client(GetMap, str(self.get_parameter('map_service').value),
                                             callback_group=self.client_group)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._map_callback, map_qos)
        self.create_subscription(
            PointCloud2, str(self.get_parameter('cloud_topic').value),
            self._cloud_callback, 1)
        self.status_pub = self.create_publisher(
            String, '/mapping/snapshot_status', 10)
        self.create_service(
            Trigger, '/mapping/create_preview_snapshot',
            self._create_preview)
        self.create_service(
            Trigger, '/mapping/save_snapshot', self._save_snapshot)
        self.get_logger().info('地图快照管理节点已启动，默认不会自动写盘')

    def _map_callback(self, msg):
        self.latest_map = msg

    def _cloud_callback(self, msg):
        self.latest_cloud = msg

    def _create_preview(self, request, response):
        del request
        return self._handle_snapshot(
            self.preview_directory, self.snapshot_basename, response)

    def _save_snapshot(self, request, response):
        del request
        session = snapshot_time(self.snapshot_timezone).strftime(
            'map_%Y%m%d_%H%M%S_%f')
        directory = os.path.join(self.save_directory, session)
        return self._handle_snapshot(directory, 'map', response, save_database=True)

    def _request(self, client, request, deadline):
        if not client.service_is_ready():
            raise ValueError(f'建图服务尚未就绪：{client.srv_name}')
        future = client.call_async(request)
        self.backup_future = future
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(max(0.0, deadline - time.monotonic())):
            raise ValueError('数据库保存服务超时，本次未生成可重载地图，请稍后重试')
        return future.result()

    def _backup_database(self):
        """暂停图优化后读取最终栅格并落盘，确保二维地图与数据库使用同一图坐标。"""
        if self.backup_future is not None and not self.backup_future.done():
            raise ValueError('上一次数据库保存仍在执行，请稍后重试')
        for client in (self.pause_client, self.resume_client, self.map_client, self.backup_client):
            if not client.service_is_ready():
                raise ValueError(f'建图服务尚未就绪：{client.srv_name}')
        deadline = time.monotonic() + self.database_timeout
        try:
            self._request(self.pause_client, Empty.Request(), deadline)
            grid = self._request(self.map_client, GetMap.Request(), deadline).map
            if not grid.info.width or not grid.info.height:
                raise ValueError('暂停建图后未获取到有效栅格，请稍后重试')
            self.latest_map = grid
            backup = self.database_path + '.back'
            if os.path.exists(backup):
                os.unlink(backup)
            self._request(self.backup_client, Empty.Request(), deadline)
            return backup
        finally:
            # 即使保存失败也请求恢复；ROS 超时请求无法撤销，若恢复也失败会明确报告。
            self._request(self.resume_client, Empty.Request(),
                          time.monotonic() + self.resume_timeout)

    def _handle_snapshot(self, directory, basename, response, save_database=False):
        if self.latest_map is None or self.latest_cloud is None:
            missing = []
            if self.latest_map is None:
                missing.append('/map')
            if self.latest_cloud is None:
                missing.append('/mapping/cloud_map')
            response.success = False
            response.message = '尚未收到: ' + ', '.join(missing)
            self._publish_status('waiting', response.message, '')
            return response
        try:
            # RTAB-Map backup 会先将内存节点写盘，再生成独立 .back；不能只复制活跃 db。
            if save_database:
                backup = self._backup_database()
                # 整包写入成功后才公开目录，失败不留下看似可用的地图。
                os.makedirs(os.path.dirname(directory), exist_ok=True)
                staging = tempfile.mkdtemp(prefix='.mapping-', dir=os.path.dirname(directory))
                try:
                    paths = self._write_snapshot(staging, basename)
                    copy_database(backup, os.path.join(staging, basename + '.db'))
                    with open(paths['metadata'], encoding='utf-8') as stream:
                        metadata = json.load(stream)
                    metadata['database'] = basename + '.db'
                    metadata['files_sha256'] = {
                        basename + extension: file_digest(os.path.join(staging, basename + extension))
                        for extension in ('.db', '.yaml', '.pgm')}
                    with open(paths['metadata'], 'w', encoding='utf-8') as stream:
                        json.dump(metadata, stream, ensure_ascii=False, indent=2)
                    apply_snapshot_permissions(staging, self.snapshot_uid, self.snapshot_gid)
                    os.rename(staging, directory)
                    paths = {key: os.path.join(directory, os.path.basename(path))
                             for key, path in paths.items()}
                finally:
                    if os.path.isdir(staging):
                        shutil.rmtree(staging)
            else:
                paths = self._write_snapshot(directory, basename)
        except (OSError, TypeError, ValueError, RuntimeError, sqlite3.DatabaseError) as exc:
            response.success = False
            response.message = f'创建地图快照失败: {exc}'
            self._publish_status('error', response.message, directory)
            return response
        response.success = True
        response.message = paths['yaml']
        self._publish_status('saved', '地图快照创建成功', directory)
        return response

    def _write_snapshot(self, directory, basename):
        os.makedirs(directory, exist_ok=True)
        image_name = basename + '.pgm'
        paths = {
            'pgm': os.path.join(directory, image_name),
            'yaml': os.path.join(directory, basename + '.yaml'),
            'ply': os.path.join(directory, basename + '.ply'),
            'metadata': os.path.join(directory, basename + '.json'),
        }
        points = cloud_to_xyz(self.latest_cloud)
        if len(points) == 0:
            raise ValueError('三维点云没有有限坐标点')
        with open(paths['pgm'], 'wb') as stream:
            stream.write(occupancy_to_pgm(self.latest_map))
        with open(paths['yaml'], 'w', encoding='utf-8') as stream:
            stream.write(occupancy_yaml(self.latest_map, image_name))
        write_binary_ply(paths['ply'], points)
        metadata = {
            'created_at': snapshot_time(self.snapshot_timezone).isoformat(),
            'map_frame': self.latest_map.header.frame_id,
            'cloud_frame': self.latest_cloud.header.frame_id,
            'map_width': self.latest_map.info.width,
            'map_height': self.latest_map.info.height,
            'resolution': self.latest_map.info.resolution,
            'point_count': int(len(points)),
        }
        with open(paths['metadata'], 'w', encoding='utf-8') as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2)
        return paths

    def _publish_status(self, state, message, directory):
        payload = {
            'state': state,
            'message': message,
            'directory': directory,
            'has_map': self.latest_map is not None,
            'has_cloud': self.latest_cloud is not None,
        }
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = SnapshotManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

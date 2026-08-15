#!/usr/bin/env python3
"""使用方法：由 stereo_mapping.launch.py 保存在线 SLAM 二维地图和三维点云快照。"""

import json
import math
import os
from datetime import datetime
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger


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
        # 启动时立即校验，避免保存请求到来后才暴露无效时区配置。
        snapshot_time(self.snapshot_timezone)
        self.latest_map = None
        self.latest_cloud = None

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
            'map_%Y%m%d_%H%M%S')
        directory = os.path.join(self.save_directory, session)
        return self._handle_snapshot(directory, 'map', response)

    def _handle_snapshot(self, directory, basename, response):
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
            paths = self._write_snapshot(directory, basename)
        except (OSError, TypeError, ValueError) as exc:
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

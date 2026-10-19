"""使用方法：感知节点调用 cloud_to_xyz 解析 PointCloud2，不跨 Package 导入导航实现。"""

import numpy as np
from sensor_msgs.msg import PointField


def cloud_to_xyz(msg):
    """按 PointCloud2 字段偏移提取有限 XYZ 点，并兼容行填充。"""
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
        'names': names, 'formats': formats, 'offsets': offsets,
        'itemsize': msg.point_step,
    })
    rows = []
    for row in range(msg.height):
        start = row * msg.row_step
        rows.append(np.frombuffer(
            msg.data, dtype=dtype, count=msg.width, offset=start))
    if not rows:
        return np.empty((0, 3), dtype=np.float32)
    structured = np.concatenate(rows)
    points = np.column_stack([
        structured['x'], structured['y'], structured['z']]).astype(
            np.float32, copy=False)
    return points[np.isfinite(points).all(axis=1)]

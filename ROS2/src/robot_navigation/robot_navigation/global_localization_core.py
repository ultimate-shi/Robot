"""作用：提供平面地图修正、相关测量保守融合及来源确认；使用方法：由 global_localization 节点调用，可独立 pytest。"""

from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], wrap(a[2]+b[2])])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2]])


def correction_with_covariance(mapped, map_cov, odom, odom_cov):
    """把同一采集时刻的地图/里程计位姿变为地图修正，传播完整平面协方差。"""
    value = compose(mapped, inverse(odom))
    c, s = math.cos(value[2]), math.sin(value[2])
    # t = p_map - R(yaw_map-yaw_odom) p_odom。
    d = np.array([s*odom[0]+c*odom[1], -c*odom[0]+s*odom[1]])
    jm = np.eye(3)
    jm[:2, 2] = d
    jo = np.array([[-c, s, -d[0]], [-s, -c, -d[1]], [0., 0., -1.]])
    # 两者可能共享视觉信息；2(A+B) 是未知交叉相关性下的保守协方差上界。
    return value, 2*(jm @ map_cov @ jm.T + jo @ odom_cov @ jo.T)


def composed_covariance(a, pa, b, pb):
    """组合位姿的完整平面协方差；不假定修正量与局部 odom 独立。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    ja = np.eye(3)
    ja[:2, 2] = [-s*b[0]-c*b[1], c*b[0]-s*b[1]]
    jb = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    return 2*(ja @ pa @ ja.T+jb @ pb @ jb.T)


def covariance_intersection(a, pa, b, pb):
    """未知相关性采用等权协方差交集；角度先在同一局部分支展开。"""
    b = np.asarray(b).copy()
    b[2] = a[2]+wrap(b[2]-a[2])
    ia, ib = np.linalg.inv(pa), np.linalg.inv(pb)
    covariance = np.linalg.inv(.5*ia+.5*ib)
    value = covariance @ (.5*ia @ a+.5*ib @ b)
    value[2] = wrap(value[2])
    return value, covariance


def planar_covariance(values, floor):
    matrix = np.asarray(values, dtype=float).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
    if not np.isfinite(matrix).all() or not np.allclose(matrix, matrix.T, atol=1e-8):
        raise ValueError('平面协方差非有限或不对称')
    if np.linalg.eigvalsh(matrix).min() < -1e-8:
        raise ValueError('平面协方差不是半正定矩阵')
    return matrix + np.diag(np.maximum(0., np.asarray(floor)-np.diag(matrix)))


@dataclass
class Estimate:
    value: np.ndarray
    covariance: np.ndarray
    stamp: float


class CorrectionFusion:
    """每路独立确认稳定地图修正；冲突优先有效视觉，重复/旧数据不能刷新锚点。"""

    def __init__(self, confirmations=3, freshness=2., confirm_distance=.25,
                 confirm_angle=.174533, conflict_distance=.5, conflict_angle=.349066):
        self.confirmations = confirmations
        self.freshness = freshness
        self.confirm_distance, self.confirm_angle = confirm_distance, confirm_angle
        self.conflict_distance, self.conflict_angle = conflict_distance, conflict_angle
        self.reset()

    def reset(self):
        self.accepted, self.pending, self.counts, self.last_stamps = {}, {}, {}, {}

    def close(self, a, b, distance, angle):
        return (np.linalg.norm(a[:2]-b[:2]) <= distance and
                abs(wrap(a[2]-b[2])) <= angle)

    def observe(self, source, value, covariance, stamp):
        if stamp <= self.last_stamps.get(source, -math.inf):
            return False
        self.last_stamps[source] = stamp
        previous = self.pending.get(source)
        consistent = (previous is not None and stamp-previous.stamp <= self.freshness and
                      self.close(value, previous.value, self.confirm_distance, self.confirm_angle))
        self.counts[source] = min(self.confirmations, self.counts.get(source, 0)+1) if consistent else 1
        estimate = Estimate(np.asarray(value), covariance, stamp)
        self.pending[source] = estimate
        # 已确认来源的小修正直接接收；大修正/恢复必须重新连续确认。
        old = self.accepted.get(source)
        continuous = (old is not None and stamp-old.stamp <= self.freshness and
                      self.close(value, old.value, self.conflict_distance, self.conflict_angle))
        if self.counts[source] >= self.confirmations or continuous:
            self.accepted[source] = estimate
            return True
        return False

    def select(self, now):
        fresh = {key: value for key, value in self.accepted.items()
                 if 0 <= now-value.stamp <= self.freshness}
        visual, amcl = fresh.get('visual'), fresh.get('amcl')
        if visual is not None and amcl is not None:
            if self.close(visual.value, amcl.value, self.conflict_distance, self.conflict_angle):
                value, covariance = covariance_intersection(
                    visual.value, visual.covariance, amcl.value, amcl.covariance)
                return Estimate(value, covariance, max(visual.stamp, amcl.stamp)), 'fused', False
            return visual, 'visual', True
        if visual is not None:
            return visual, 'visual', False
        if amcl is not None:
            return amcl, 'amcl', False
        return None, 'tracking', False


class MapLikelihood:
    """用静态地图距离场检查扫描端点；未知区域和越界点不能成为匹配证据。"""

    def __init__(self, cells, resolution, origin, occupied_threshold=65):
        self.cells = np.asarray(cells)
        self.resolution, self.origin = resolution, origin
        occupied = self.cells >= occupied_threshold
        self.distance = (distance_transform_edt(~occupied)*resolution if occupied.any()
                         else np.full(self.cells.shape, math.inf))

    def score(self, endpoints, map_sensor_pose, max_distance):
        if len(endpoints) == 0:
            return 0.
        pose = compose(inverse(self.origin), map_sensor_pose)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world = endpoints @ np.array([[c, s], [-s, c]]) + pose[:2]
        indices = np.floor(world/self.resolution).astype(int)
        x, y = indices.T
        inside = (x >= 0) & (y >= 0) & (x < self.cells.shape[1]) & (y < self.cells.shape[0])
        x, y = x[inside], y[inside]
        hits = (self.cells[y, x] >= 0) & (self.distance[y, x] <= max_distance)
        return float(hits.sum()/len(endpoints))

"""
robot.launch 使用说明：
本模块不会被 launch 直接启动，而是被 perception/terrain_analyzer.py 使用。
它不再直接读取 PLY 文件，而是接收已经转换好的 PointCloud2 点数组。

输入：
- /perception/points 解析后的 Nx3 numpy 点数组，仿真来自 publish_ply，现实可来自双目摄像头。
- grid_resolution、ground_tolerance 等地形参数。

输出/能力：
- 查询机器人四个轮子位置处的地面高度。
- 计算 body_z、roll、pitch，帮助 /terrain_status 表达 3D 地形状态。
- 沿前进方向 lookahead，检查台阶和坑洼。

为什么不能删除：
terrain_analyzer_node.py 的地形感知依赖它。
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


@dataclass
class WheelTerrainInfo:
    """Result of querying terrain at 4 wheel positions."""
    body_z: float
    roll: float
    pitch: float
    wheel_z: list


@dataclass
class LookaheadResult:
    """Result of looking ahead along heading direction."""
    height_diff: float
    max_step_up: float
    max_step_down: float
    is_step: bool
    is_dropoff: bool


class TerrainHeightmap:
    """Build a 2D terrain height grid from an Nx3 point cloud array."""

    def __init__(self, points: np.ndarray, resolution: float = 0.02,
                 ground_tolerance: float = 0.05, voxel_size: float = 0.03):
        self.resolution = resolution
        self.ground_tolerance = ground_tolerance
        self.voxel_size = voxel_size

        self.height_grid = None
        self.normal_grid = None
        self.valid_mask = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.grid_w = 0
        self.grid_h = 0
        self.ground_z_base = 0.0

        self._build(points)

    def _build(self, points: np.ndarray):
        """Build height grid from supplied point cloud points."""
        if points.size == 0:
            raise ValueError('terrain point cloud is empty')
        points = points[np.isfinite(points).all(axis=1)].astype(np.float32)
        if len(points) < 10:
            raise ValueError('terrain point cloud has too few valid points')

        points = self._voxel_downsample(points)
        self.ground_z_base = self._detect_ground_level(points)
        self._build_height_grid(points)
        self._fill_holes()
        self._compute_normals()

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Downsample points with a voxel grid."""
        if self.voxel_size <= 0.0:
            return points
        grid = np.floor(points / self.voxel_size).astype(np.int32)
        _, idx = np.unique(grid, axis=0, return_index=True)
        return points[idx]

    def _detect_ground_level(self, points: np.ndarray) -> float:
        """Detect ground z level using histogram of z values."""
        z_values = points[:, 2]
        z_min, z_max = z_values.min(), z_values.max()
        bin_size = 0.01
        num_bins = max(1, int((z_max - z_min) / bin_size))
        hist, bin_edges = np.histogram(z_values, bins=num_bins)

        threshold = hist.max() * 0.05
        for i in range(len(hist)):
            if hist[i] > threshold:
                return float((bin_edges[i] + bin_edges[i + 1]) / 2.0)

        sorted_z = np.sort(z_values)
        return float(np.median(sorted_z[:max(1, len(sorted_z) // 5)]))

    def _build_height_grid(self, points: np.ndarray):
        """Build 2D height grid from ground-level points."""
        z_values = points[:, 2]
        ground_mask = np.abs(z_values - self.ground_z_base) < self.ground_tolerance
        ground_points = points[ground_mask]

        if len(ground_points) < 10:
            low_mask = z_values < (self.ground_z_base + self.ground_tolerance * 2)
            ground_points = points[low_mask] if np.any(low_mask) else points

        x_min, y_min = ground_points[:, 0].min(), ground_points[:, 1].min()
        x_max, y_max = ground_points[:, 0].max(), ground_points[:, 1].max()

        pad = self.resolution * 2
        self.origin_x = x_min - pad
        self.origin_y = y_min - pad
        self.grid_w = int(math.ceil((x_max - x_min + 2 * pad) / self.resolution))
        self.grid_h = int(math.ceil((y_max - y_min + 2 * pad) / self.resolution))
        self.grid_w = min(max(self.grid_w, 2), 1000)
        self.grid_h = min(max(self.grid_h, 2), 1000)

        self.height_grid = np.full((self.grid_h, self.grid_w), np.nan, dtype=np.float32)
        count_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int32)

        for p in ground_points:
            col = int((p[0] - self.origin_x) / self.resolution)
            row = int((p[1] - self.origin_y) / self.resolution)
            if 0 <= row < self.grid_h and 0 <= col < self.grid_w:
                if np.isnan(self.height_grid[row, col]):
                    self.height_grid[row, col] = p[2]
                    count_grid[row, col] = 1
                else:
                    self.height_grid[row, col] += p[2]
                    count_grid[row, col] += 1

        valid = count_grid > 0
        self.height_grid[valid] /= count_grid[valid]
        self.valid_mask = valid

    def _fill_holes(self):
        """Fill NaN cells using nearest-neighbor interpolation."""
        if cKDTree is None or self.valid_mask is None:
            nan_mask = np.isnan(self.height_grid)
            self.height_grid[nan_mask] = self.ground_z_base
            self.valid_mask = np.ones_like(self.height_grid, dtype=bool)
            return

        nan_mask = np.isnan(self.height_grid)
        if not np.any(nan_mask):
            return

        valid_coords = np.argwhere(self.valid_mask)
        invalid_coords = np.argwhere(nan_mask)
        if len(valid_coords) == 0:
            self.height_grid[nan_mask] = self.ground_z_base
            self.valid_mask = np.ones_like(self.height_grid, dtype=bool)
            return

        tree = cKDTree(valid_coords)
        _, indices = tree.query(invalid_coords, k=1)
        for i, inv_coord in enumerate(invalid_coords):
            nearest_valid = valid_coords[indices[i]]
            self.height_grid[inv_coord[0], inv_coord[1]] = \
                self.height_grid[nearest_valid[0], nearest_valid[1]]
        self.valid_mask[:] = True

    def _compute_normals(self):
        """Compute surface normals using finite differences on height grid."""
        self.normal_grid = np.zeros((self.grid_h, self.grid_w, 3), dtype=np.float32)
        dz_dx = np.zeros_like(self.height_grid)
        dz_dx[:, 1:-1] = (self.height_grid[:, 2:] - self.height_grid[:, :-2]) / (2 * self.resolution)
        dz_dx[:, 0] = (self.height_grid[:, 1] - self.height_grid[:, 0]) / self.resolution
        dz_dx[:, -1] = (self.height_grid[:, -1] - self.height_grid[:, -2]) / self.resolution

        dz_dy = np.zeros_like(self.height_grid)
        dz_dy[1:-1, :] = (self.height_grid[2:, :] - self.height_grid[:-2, :]) / (2 * self.resolution)
        dz_dy[0, :] = (self.height_grid[1, :] - self.height_grid[0, :]) / self.resolution
        dz_dy[-1, :] = (self.height_grid[-1, :] - self.height_grid[-2, :]) / self.resolution

        self.normal_grid[:, :, 0] = -dz_dx
        self.normal_grid[:, :, 1] = -dz_dy
        self.normal_grid[:, :, 2] = 1.0
        norms = np.linalg.norm(self.normal_grid, axis=2, keepdims=True)
        self.normal_grid /= np.maximum(norms, 1e-8)

    def _world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coords to grid row/col floats."""
        return (y - self.origin_y) / self.resolution, (x - self.origin_x) / self.resolution

    def _in_bounds(self, row: float, col: float) -> bool:
        """Check if row/col are inside interpolation bounds."""
        return 0 <= row < self.grid_h - 1 and 0 <= col < self.grid_w - 1

    def _bilinear(self, grid: np.ndarray, row: float, col: float) -> Optional[float]:
        """Bilinear interpolation on 2D grid."""
        if not self._in_bounds(row, col):
            return None
        r0, c0 = int(row), int(col)
        r1, c1 = min(r0 + 1, self.grid_h - 1), min(c0 + 1, self.grid_w - 1)
        fr, fc = row - r0, col - c0
        v00, v01 = grid[r0, c0], grid[r0, c1]
        v10, v11 = grid[r1, c0], grid[r1, c1]
        return float(v00 * (1 - fr) * (1 - fc) + v01 * (1 - fr) * fc +
                     v10 * fr * (1 - fc) + v11 * fr * fc)

    def query_height(self, x: float, y: float) -> float:
        """Query terrain height at world position."""
        row, col = self._world_to_grid(x, y)
        result = self._bilinear(self.height_grid, row, col)
        return self.ground_z_base if result is None else result

    def query_normal(self, x: float, y: float) -> np.ndarray:
        """Query surface normal at world position."""
        row, col = self._world_to_grid(x, y)
        if not self._in_bounds(row, col):
            return np.array([0.0, 0.0, 1.0], dtype=np.float32)
        return self.normal_grid[int(row), int(col)].copy()

    def query_4wheels(self, cx: float, cy: float, yaw: float,
                      wheelbase: float, track: float,
                      ground_to_base: float = 0.15) -> WheelTerrainInfo:
        """Query terrain at 4 wheel positions and compute body pose."""
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        half_wb, half_tr = wheelbase / 2.0, track / 2.0
        wheels = [
            (cx + half_wb * cos_y - half_tr * sin_y, cy + half_wb * sin_y + half_tr * cos_y),
            (cx + half_wb * cos_y + half_tr * sin_y, cy + half_wb * sin_y - half_tr * cos_y),
            (cx - half_wb * cos_y - half_tr * sin_y, cy - half_wb * sin_y + half_tr * cos_y),
            (cx - half_wb * cos_y + half_tr * sin_y, cy - half_wb * sin_y - half_tr * cos_y),
        ]
        wheel_z = [self.query_height(wx, wy) for wx, wy in wheels]
        z_fl, z_fr, z_rl, z_rr = wheel_z
        roll = math.atan2((z_fl + z_rl) - (z_fr + z_rr), 2.0 * track)
        pitch = math.atan2((z_fl + z_fr) - (z_rl + z_rr), 2.0 * wheelbase)
        body_z = sum(wheel_z) / 4.0 + ground_to_base
        return WheelTerrainInfo(body_z=body_z, roll=roll, pitch=pitch, wheel_z=wheel_z)

    def query_lookahead(self, x: float, y: float, heading: float,
                        distance: float = 0.10,
                        num_samples: int = 5) -> LookaheadResult:
        """Look ahead along heading direction and detect steps/drop-offs."""
        current_z = self.query_height(x, y)
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        max_step_up, max_step_down = 0.0, 0.0
        prev_z = current_z
        for i in range(1, num_samples + 1):
            d = distance * i / num_samples
            sample_z = self.query_height(x + d * cos_h, y + d * sin_h)
            diff_from_prev = sample_z - prev_z
            max_step_up = max(max_step_up, diff_from_prev)
            max_step_down = min(max_step_down, diff_from_prev)
            prev_z = sample_z
        return LookaheadResult(
            height_diff=prev_z - current_z,
            max_step_up=max_step_up,
            max_step_down=max_step_down,
            is_step=False,
            is_dropoff=False,
        )

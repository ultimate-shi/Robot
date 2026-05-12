"""
TerrainHeightmap - PLY point cloud to 2D height grid with O(1) query.

Builds a terrain height map from iPhone-scanned PLY data.
Provides fast queries for ground height, surface normals, and traversability.
Used by chassis_controller_3d for 6DOF pose estimation and obstacle detection.
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Optional

try:
    from plyfile import PlyData
except ImportError:
    PlyData = None

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


@dataclass
class WheelTerrainInfo:
    """Result of querying terrain at 4 wheel positions."""
    body_z: float       # Body center height above ground reference
    roll: float         # Roll angle (rad) from left-right wheel height diff
    pitch: float        # Pitch angle (rad) from front-rear wheel height diff
    wheel_z: list       # [z_fl, z_fr, z_rl, z_rr]


@dataclass
class LookaheadResult:
    """Result of looking ahead along heading direction."""
    height_diff: float      # Max height difference ahead
    max_step_up: float      # Max positive height change (step up)
    max_step_down: float    # Max negative height change (drop off)
    is_step: bool           # Step up exceeds threshold
    is_dropoff: bool        # Drop off exceeds threshold


class TerrainHeightmap:
    """
    Builds a 2D height grid from PLY point cloud data.
    Provides O(1) terrain queries via bilinear interpolation.
    """

    def __init__(self, ply_path: str, resolution: float = 0.02,
                 ground_tolerance: float = 0.05, voxel_size: float = 0.03):
        """
        Args:
            ply_path: Path to PLY point cloud file
            resolution: Height grid resolution in meters (default 2cm)
            ground_tolerance: Max z deviation from detected ground level
            voxel_size: Voxel size for downsampling PLY data
        """
        self.ply_path = ply_path
        self.resolution = resolution
        self.ground_tolerance = ground_tolerance
        self.voxel_size = voxel_size

        # Grid data (populated by build())
        self.height_grid = None      # 2D array of ground z values
        self.normal_grid = None      # 3D array (H, W, 3) of surface normals
        self.valid_mask = None       # Boolean mask of cells with data
        self.origin_x = 0.0         # World X of grid[0,0]
        self.origin_y = 0.0         # World Y of grid[0,0]
        self.grid_w = 0             # Grid width (columns)
        self.grid_h = 0             # Grid height (rows)
        self.ground_z_base = 0.0    # Detected ground level

        self._build()

    def _build(self):
        """Load PLY and build height grid."""
        points = self._load_ply()
        points = self._voxel_downsample(points)
        self.ground_z_base = self._detect_ground_level(points)
        self._build_height_grid(points)
        self._fill_holes()
        self._compute_normals()

    def _load_ply(self) -> np.ndarray:
        """Load PLY file and return Nx3 point array."""
        if PlyData is None:
            raise ImportError("plyfile not installed: pip install plyfile")

        ply = PlyData.read(self.ply_path)
        vertex = ply["vertex"]
        points = np.vstack([vertex["x"], vertex["y"], vertex["z"]]).T.astype(np.float32)
        return points

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Voxel grid downsampling."""
        grid = np.floor(points / self.voxel_size).astype(np.int32)
        _, idx = np.unique(grid, axis=0, return_index=True)
        return points[idx]

    def _detect_ground_level(self, points: np.ndarray) -> float:
        """Detect ground z level using histogram of z values."""
        z_values = points[:, 2]
        # Create histogram with 1cm bins
        z_min, z_max = z_values.min(), z_values.max()
        bin_size = 0.01
        num_bins = max(1, int((z_max - z_min) / bin_size))
        hist, bin_edges = np.histogram(z_values, bins=num_bins)

        # Ground is the lowest significant peak (>5% of max bin count)
        threshold = hist.max() * 0.05
        for i in range(len(hist)):
            if hist[i] > threshold:
                ground_z = (bin_edges[i] + bin_edges[i + 1]) / 2.0
                return float(ground_z)

        # Fallback: use median of lowest 20% points
        sorted_z = np.sort(z_values)
        return float(np.median(sorted_z[:len(sorted_z) // 5]))

    def _build_height_grid(self, points: np.ndarray):
        """Build 2D height grid from ground points."""
        # Filter to ground-level points
        z_values = points[:, 2]
        ground_mask = np.abs(z_values - self.ground_z_base) < self.ground_tolerance
        ground_points = points[ground_mask]

        if len(ground_points) < 10:
            # Fallback: use all low points
            low_mask = z_values < (self.ground_z_base + self.ground_tolerance * 2)
            ground_points = points[low_mask] if np.any(low_mask) else points

        # Determine grid bounds
        x_min, y_min = ground_points[:, 0].min(), ground_points[:, 1].min()
        x_max, y_max = ground_points[:, 0].max(), ground_points[:, 1].max()

        # Add small padding
        pad = self.resolution * 2
        self.origin_x = x_min - pad
        self.origin_y = y_min - pad

        self.grid_w = int(math.ceil((x_max - x_min + 2 * pad) / self.resolution))
        self.grid_h = int(math.ceil((y_max - y_min + 2 * pad) / self.resolution))

        # Clamp grid size to reasonable limits
        max_grid = 1000
        self.grid_w = min(self.grid_w, max_grid)
        self.grid_h = min(self.grid_h, max_grid)

        # Initialize grids
        self.height_grid = np.full((self.grid_h, self.grid_w), np.nan, dtype=np.float32)
        count_grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int32)

        # Populate grid cells with average z
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

        # Average
        valid = count_grid > 0
        self.height_grid[valid] /= count_grid[valid]
        self.valid_mask = valid

    def _fill_holes(self):
        """Fill NaN cells using nearest-neighbor interpolation."""
        if cKDTree is None or self.valid_mask is None:
            # Simple fallback: fill with ground_z_base
            nan_mask = np.isnan(self.height_grid)
            self.height_grid[nan_mask] = self.ground_z_base
            self.valid_mask = np.ones_like(self.valid_mask, dtype=bool)
            return

        nan_mask = np.isnan(self.height_grid)
        if not np.any(nan_mask):
            return

        # Get valid and invalid cell coordinates
        valid_coords = np.argwhere(self.valid_mask)  # (N, 2) [row, col]
        invalid_coords = np.argwhere(nan_mask)

        if len(valid_coords) == 0:
            self.height_grid[nan_mask] = self.ground_z_base
            self.valid_mask[:] = True
            return

        # Build KDTree from valid cells and query nearest for invalid
        tree = cKDTree(valid_coords)
        _, indices = tree.query(invalid_coords, k=1)

        # Fill with nearest valid height
        for i, inv_coord in enumerate(invalid_coords):
            nearest_valid = valid_coords[indices[i]]
            self.height_grid[inv_coord[0], inv_coord[1]] = \
                self.height_grid[nearest_valid[0], nearest_valid[1]]

        self.valid_mask[:] = True

    def _compute_normals(self):
        """Compute surface normals using finite differences on height grid."""
        self.normal_grid = np.zeros((self.grid_h, self.grid_w, 3), dtype=np.float32)

        # Compute gradients
        # dz/dx (along columns)
        dz_dx = np.zeros_like(self.height_grid)
        dz_dx[:, 1:-1] = (self.height_grid[:, 2:] - self.height_grid[:, :-2]) / (2 * self.resolution)
        dz_dx[:, 0] = (self.height_grid[:, 1] - self.height_grid[:, 0]) / self.resolution
        dz_dx[:, -1] = (self.height_grid[:, -1] - self.height_grid[:, -2]) / self.resolution

        # dz/dy (along rows)
        dz_dy = np.zeros_like(self.height_grid)
        dz_dy[1:-1, :] = (self.height_grid[2:, :] - self.height_grid[:-2, :]) / (2 * self.resolution)
        dz_dy[0, :] = (self.height_grid[1, :] - self.height_grid[0, :]) / self.resolution
        dz_dy[-1, :] = (self.height_grid[-1, :] - self.height_grid[-2, :]) / self.resolution

        # Normal = (-dz/dx, -dz/dy, 1) normalized
        self.normal_grid[:, :, 0] = -dz_dx
        self.normal_grid[:, :, 1] = -dz_dy
        self.normal_grid[:, :, 2] = 1.0

        # Normalize
        norms = np.linalg.norm(self.normal_grid, axis=2, keepdims=True)
        norms = np.maximum(norms, 1e-8)
        self.normal_grid /= norms

    def _world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coords to grid (row, col) as floats."""
        col = (x - self.origin_x) / self.resolution
        row = (y - self.origin_y) / self.resolution
        return row, col

    def _in_bounds(self, row: float, col: float) -> bool:
        """Check if grid coordinates are within bounds."""
        return 0 <= row < self.grid_h - 1 and 0 <= col < self.grid_w - 1

    def _bilinear(self, grid: np.ndarray, row: float, col: float) -> Optional[float]:
        """Bilinear interpolation on 2D grid."""
        if not self._in_bounds(row, col):
            return None

        r0, c0 = int(row), int(col)
        r1, c1 = r0 + 1, c0 + 1

        # Clamp
        r1 = min(r1, self.grid_h - 1)
        c1 = min(c1, self.grid_w - 1)

        fr = row - r0
        fc = col - c0

        v00 = grid[r0, c0]
        v01 = grid[r0, c1]
        v10 = grid[r1, c0]
        v11 = grid[r1, c1]

        val = (v00 * (1 - fr) * (1 - fc) +
               v01 * (1 - fr) * fc +
               v10 * fr * (1 - fc) +
               v11 * fr * fc)
        return float(val)

    def query_height(self, x: float, y: float) -> float:
        """Query terrain height at world position (x, y)."""
        row, col = self._world_to_grid(x, y)
        result = self._bilinear(self.height_grid, row, col)
        if result is None:
            return self.ground_z_base
        return result

    def query_normal(self, x: float, y: float) -> np.ndarray:
        """Query surface normal at world position (x, y)."""
        row, col = self._world_to_grid(x, y)
        if not self._in_bounds(row, col):
            return np.array([0.0, 0.0, 1.0], dtype=np.float32)

        r0, c0 = int(row), int(col)
        return self.normal_grid[r0, c0].copy()

    def query_4wheels(self, cx: float, cy: float, yaw: float,
                      wheelbase: float, track: float,
                      ground_to_base: float = 0.15) -> WheelTerrainInfo:
        """
        Query terrain at 4 wheel positions and compute body pose.

        Args:
            cx, cy: Body center position in world frame
            yaw: Heading angle (rad)
            wheelbase: Front-rear axle distance
            track: Left-right wheel distance
            ground_to_base: Vertical offset from ground to base_link
        """
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        half_wb = wheelbase / 2.0
        half_tr = track / 2.0

        # Wheel positions in world frame
        # FL: front-left, FR: front-right, RL: rear-left, RR: rear-right
        wheels = [
            (cx + half_wb * cos_y - half_tr * sin_y,
             cy + half_wb * sin_y + half_tr * cos_y),   # FL
            (cx + half_wb * cos_y + half_tr * sin_y,
             cy + half_wb * sin_y - half_tr * cos_y),   # FR
            (cx - half_wb * cos_y - half_tr * sin_y,
             cy - half_wb * sin_y + half_tr * cos_y),   # RL
            (cx - half_wb * cos_y + half_tr * sin_y,
             cy - half_wb * sin_y - half_tr * cos_y),   # RR
        ]

        wheel_z = [self.query_height(wx, wy) for wx, wy in wheels]
        z_fl, z_fr, z_rl, z_rr = wheel_z

        # Body pose from wheel heights
        # Roll: positive = left side higher
        roll = math.atan2((z_fl + z_rl) - (z_fr + z_rr), 2.0 * track)
        # Pitch: positive = front higher (nose up)
        pitch = math.atan2((z_fl + z_fr) - (z_rl + z_rr), 2.0 * wheelbase)
        # Body Z: mean of wheels + clearance
        body_z = sum(wheel_z) / 4.0 + ground_to_base

        return WheelTerrainInfo(
            body_z=body_z,
            roll=roll,
            pitch=pitch,
            wheel_z=wheel_z
        )

    def query_lookahead(self, x: float, y: float, heading: float,
                        distance: float = 0.10,
                        num_samples: int = 5) -> LookaheadResult:
        """
        Look ahead along heading direction and detect steps/drop-offs.

        Args:
            x, y: Current position
            heading: Direction to look (rad)
            distance: How far ahead to look (m)
            num_samples: Number of sample points along lookahead
        """
        current_z = self.query_height(x, y)
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)

        max_step_up = 0.0
        max_step_down = 0.0

        prev_z = current_z
        for i in range(1, num_samples + 1):
            d = distance * i / num_samples
            sample_x = x + d * cos_h
            sample_y = y + d * sin_h
            sample_z = self.query_height(sample_x, sample_y)

            # Height difference relative to current position
            diff_from_current = sample_z - current_z
            # Height difference from previous sample (local step)
            diff_from_prev = sample_z - prev_z

            if diff_from_prev > max_step_up:
                max_step_up = diff_from_prev
            if diff_from_prev < max_step_down:
                max_step_down = diff_from_prev

            prev_z = sample_z

        # Overall height difference
        height_diff = prev_z - current_z

        return LookaheadResult(
            height_diff=height_diff,
            max_step_up=max_step_up,
            max_step_down=max_step_down,
            is_step=False,      # Set by TerrainPhysics
            is_dropoff=False    # Set by TerrainPhysics
        )

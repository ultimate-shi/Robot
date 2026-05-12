"""
TerrainPhysics - Physical constraint engine for terrain traversal.

Evaluates slope, step, and drop-off constraints to determine
if the robot can safely traverse terrain at a given position.
"""

import math
from dataclasses import dataclass

from robot.terrain_heightmap import TerrainHeightmap, WheelTerrainInfo, LookaheadResult


@dataclass
class TerrainConstraint:
    """Result of terrain physics evaluation."""
    is_blocked: bool        # Cannot proceed
    block_reason: str       # "" / "slope" / "step" / "dropoff"
    slip_factor: float      # 0.0~1.0, velocity multiplier (1=full grip)
    traversability: float   # 0.0~1.0 overall score
    body_z: float           # Body height
    roll: float             # Body roll (rad)
    pitch: float            # Body pitch (rad)


class TerrainPhysics:
    """
    Evaluates physical constraints for robot terrain traversal.
    Determines slip, blockage, and traversability.
    """

    def __init__(self, max_grade_deg: float = 35.0,
                 step_threshold: float = 0.03,
                 dropoff_threshold: float = 0.05,
                 look_ahead_distance: float = 0.10,
                 look_ahead_samples: int = 5,
                 ground_to_base_height: float = 0.15,
                 wheelbase: float = 0.4,
                 track: float = 0.2):
        """
        Args:
            max_grade_deg: Maximum traversable slope angle in degrees
            step_threshold: Step height that blocks traversal (meters)
            dropoff_threshold: Drop height that blocks traversal (meters)
            look_ahead_distance: Distance to look ahead for obstacles (meters)
            look_ahead_samples: Number of sample points for lookahead
            ground_to_base_height: Vertical offset ground to base_link
            wheelbase: Front-rear axle distance
            track: Left-right wheel distance
        """
        self.max_grade_rad = math.radians(max_grade_deg)
        self.slip_start_rad = self.max_grade_rad * 0.6  # Slip starts at 60% of max
        self.step_threshold = step_threshold
        self.dropoff_threshold = dropoff_threshold
        self.look_ahead_distance = look_ahead_distance
        self.look_ahead_samples = look_ahead_samples
        self.ground_to_base_height = ground_to_base_height
        self.wheelbase = wheelbase
        self.track = track

    def evaluate(self, heightmap: TerrainHeightmap,
                 x: float, y: float, heading: float,
                 vx: float = 0.0) -> TerrainConstraint:
        """
        Evaluate terrain constraints at current position.

        Args:
            heightmap: The terrain height map
            x, y: Current robot position in world frame
            heading: Current heading (yaw) in radians
            vx: Current forward velocity (used to determine direction)

        Returns:
            TerrainConstraint with all constraint results
        """
        # Get body pose from wheel positions
        wheel_info = heightmap.query_4wheels(
            x, y, heading,
            self.wheelbase, self.track,
            self.ground_to_base_height
        )

        # Check slope
        slope_angle = self._compute_slope_angle(wheel_info)
        slip_factor = self._compute_slip_factor(slope_angle)
        slope_blocked = slope_angle > self.max_grade_rad

        # Check ahead only if moving forward
        step_blocked = False
        dropoff_blocked = False

        if vx > 0.001:  # Only check forward direction when moving forward
            lookahead = heightmap.query_lookahead(
                x, y, heading,
                self.look_ahead_distance,
                self.look_ahead_samples
            )
            step_blocked = lookahead.max_step_up > self.step_threshold
            dropoff_blocked = abs(lookahead.max_step_down) > self.dropoff_threshold
        elif vx < -0.001:  # Check backward when reversing
            rear_heading = heading + math.pi
            lookahead = heightmap.query_lookahead(
                x, y, rear_heading,
                self.look_ahead_distance,
                self.look_ahead_samples
            )
            step_blocked = lookahead.max_step_up > self.step_threshold
            dropoff_blocked = abs(lookahead.max_step_down) > self.dropoff_threshold

        # Determine overall blockage
        is_blocked = slope_blocked or step_blocked or dropoff_blocked
        block_reason = ""
        if slope_blocked:
            block_reason = "slope"
        elif step_blocked:
            block_reason = "step"
        elif dropoff_blocked:
            block_reason = "dropoff"

        # Traversability score
        slope_score = max(0.0, 1.0 - slope_angle / self.max_grade_rad)
        traversability = slope_score * (0.0 if is_blocked else 1.0)

        return TerrainConstraint(
            is_blocked=is_blocked,
            block_reason=block_reason,
            slip_factor=slip_factor,
            traversability=traversability,
            body_z=wheel_info.body_z,
            roll=wheel_info.roll,
            pitch=wheel_info.pitch
        )

    def _compute_slope_angle(self, wheel_info: WheelTerrainInfo) -> float:
        """Compute overall slope angle from roll and pitch."""
        # Combined slope angle (magnitude of tilt)
        return math.sqrt(wheel_info.roll ** 2 + wheel_info.pitch ** 2)

    def _compute_slip_factor(self, slope_angle: float) -> float:
        """
        Compute slip factor based on slope angle.
        Returns 1.0 (full grip) to 0.0 (complete slip).
        Linear decay between slip_start and max_grade.
        """
        if slope_angle <= self.slip_start_rad:
            return 1.0
        elif slope_angle >= self.max_grade_rad:
            return 0.0
        else:
            # Linear interpolation between slip_start and max_grade
            t = (slope_angle - self.slip_start_rad) / (self.max_grade_rad - self.slip_start_rad)
            return 1.0 - t

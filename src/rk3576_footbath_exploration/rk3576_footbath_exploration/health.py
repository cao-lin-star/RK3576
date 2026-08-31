"""Pure helper functions shared by the exploration supervisor and tests."""

from datetime import datetime
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple


ValidationResult = Tuple[bool, str]
MAX_OCCUPANCY_GRID_CELLS = 100_000_000


def evaluate_freshness(
    now_s: float,
    last_seen_s: Dict[str, Optional[float]],
    maximum_age_s: Dict[str, float],
    required: Iterable[str],
) -> Tuple[bool, Dict[str, float]]:
    """Return aggregate health and the age of every required input."""
    ages: Dict[str, float] = {}
    healthy = True
    for name in required:
        stamp = last_seen_s.get(name)
        age = float("inf") if stamp is None else max(0.0, now_s - stamp)
        ages[name] = age
        if age > maximum_age_s[name]:
            healthy = False
    return healthy, ages


def validate_laser_scan(
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    ranges: Sequence[float],
) -> ValidationResult:
    """Validate LaserScan metadata and require one usable finite return."""
    metadata = (
        angle_min,
        angle_max,
        angle_increment,
        range_min,
        range_max,
    )
    try:
        metadata_finite = all(math.isfinite(value) for value in metadata)
    except TypeError:
        metadata_finite = False
    if not metadata_finite:
        return False, "scan metadata contains non-finite values"
    if angle_max <= angle_min:
        return False, "scan angle_max must be greater than angle_min"
    if angle_increment <= 0.0:
        return False, "scan angle_increment must be positive"
    if angle_increment > angle_max - angle_min:
        return False, "scan angle_increment exceeds angular span"
    if range_min < 0.0 or range_max <= range_min:
        return False, "scan range limits are invalid"
    if len(ranges) == 0:
        return False, "scan ranges are empty"
    try:
        has_usable_return = any(
            math.isfinite(value) and range_min <= value <= range_max
            for value in ranges
        )
    except TypeError:
        has_usable_return = False
    if not has_usable_return:
        return False, "scan has no finite return inside declared range limits"
    return True, "ok"


def validate_occupancy_grid(
    width: int,
    height: int,
    resolution: float,
    data_length: int,
) -> ValidationResult:
    """Validate OccupancyGrid dimensions, resolution and storage length."""
    if width <= 0 or height <= 0:
        return False, "map width and height must be positive"
    if not math.isfinite(resolution) or resolution <= 0.0:
        return False, "map resolution must be finite and positive"
    cells = width * height
    if cells > MAX_OCCUPANCY_GRID_CELLS:
        return False, "map dimensions exceed the configured sanity limit"
    if data_length != cells:
        return False, "map data length does not match width times height"
    return True, "ok"


def _finite_sequence(
    values: Sequence[float], expected_length: int
) -> bool:
    """Return whether a fixed-size numeric sequence contains finite values."""
    if len(values) != expected_length:
        return False
    try:
        return all(math.isfinite(value) for value in values)
    except TypeError:
        return False


def validate_odometry(
    position_xyz: Sequence[float],
    orientation_xyzw: Sequence[float],
    linear_twist_xyz: Sequence[float],
    angular_twist_xyz: Sequence[float],
) -> ValidationResult:
    """Validate finite odometry pose/twist and a non-zero quaternion."""
    if not _finite_sequence(position_xyz, 3):
        return False, "odometry position is malformed or non-finite"
    if not _finite_sequence(orientation_xyzw, 4):
        return False, "odometry orientation is malformed or non-finite"
    if math.hypot(*orientation_xyzw) <= 1.0e-12:
        return False, "odometry orientation quaternion is zero"
    if not _finite_sequence(linear_twist_xyz, 3):
        return False, "odometry linear twist is malformed or non-finite"
    if not _finite_sequence(angular_twist_xyz, 3):
        return False, "odometry angular twist is malformed or non-finite"
    return True, "ok"


def auto_motion_lease_value(state: str, running_state: str) -> bool:
    """Authorize automatic motion only for the exact running state."""
    return state == running_state


def timestamped_prefix(pattern: str, when: datetime) -> str:
    """Expand strftime tokens in a configured absolute map prefix."""
    return when.strftime(pattern)

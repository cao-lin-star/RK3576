"""Pure safety calculations shared by ROS nodes and host tests."""

import math
from typing import Optional, Tuple


Command = Tuple[float, float]


def normalize_angle(angle: float) -> float:
    """Return angle in [-pi, pi], including scans encoded as 0..2*pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp_diff_drive(
    linear_x: float,
    angular_z: float,
    max_linear: float,
    max_angular: float,
) -> Command:
    """Validate and independently clamp a differential-drive command."""
    if not all(math.isfinite(value) for value in (
        linear_x, angular_z, max_linear, max_angular
    )):
        raise ValueError("velocity and limits must be finite")
    if max_linear <= 0.0 or max_angular <= 0.0:
        raise ValueError("velocity limits must be positive")
    return (
        max(-max_linear, min(max_linear, linear_x)),
        max(-max_angular, min(max_angular, angular_z)),
    )


def lease_is_active(
    now_s: float,
    lease_level: bool,
    last_lease_s: Optional[float],
    lease_timeout_s: float,
) -> bool:
    """Return true only for a positive, fresh short-lived motion lease."""
    if last_lease_s is None or lease_timeout_s <= 0.0:
        return False
    return lease_level and 0.0 <= now_s - last_lease_s <= lease_timeout_s


def motion_lease_allows(
    now_s: float,
    require_lease: bool,
    lease_level: bool,
    last_lease_s: Optional[float],
    lease_timeout_s: float,
) -> bool:
    """Allow unleased navigation only when explicitly configured."""
    return not require_lease or lease_is_active(
        now_s, lease_level, last_lease_s, lease_timeout_s)


def command_is_fresh(
    now_s: float,
    last_command_s: Optional[float],
    timeout_s: float,
) -> bool:
    """Return whether a command is recent enough to remain selectable."""
    if last_command_s is None or timeout_s <= 0.0:
        return False
    return 0.0 <= now_s - last_command_s <= timeout_s


def select_fresh_command(
    now_s: float,
    manual_command: Command,
    manual_stamp_s: Optional[float],
    manual_timeout_s: float,
    auto_command: Command,
    auto_stamp_s: Optional[float],
    auto_timeout_s: float,
    manual_publisher_count: int = 1,
) -> Tuple[str, Command]:
    """Fail closed on multiple manual publishers, then select by freshness."""
    if manual_publisher_count < 0:
        raise ValueError("manual publisher count must be non-negative")
    if manual_publisher_count > 1:
        return "manual_conflict", (0.0, 0.0)
    if command_is_fresh(now_s, manual_stamp_s, manual_timeout_s):
        return "manual", manual_command
    if command_is_fresh(now_s, auto_stamp_s, auto_timeout_s):
        return "auto", auto_command
    return "none", (0.0, 0.0)

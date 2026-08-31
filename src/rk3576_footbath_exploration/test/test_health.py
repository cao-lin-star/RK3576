# Copyright 2026 sky
# Licensed under the Apache License, Version 2.0

from datetime import datetime
import math

import pytest

from rk3576_footbath_exploration.health import auto_motion_lease_value
from rk3576_footbath_exploration.health import evaluate_freshness
from rk3576_footbath_exploration.health import timestamped_prefix
from rk3576_footbath_exploration.health import validate_laser_scan
from rk3576_footbath_exploration.health import validate_occupancy_grid
from rk3576_footbath_exploration.health import validate_odometry


def test_freshness_requires_every_input():
    healthy, ages = evaluate_freshness(
        10.0,
        {"scan": 9.8, "map": 7.0, "odom": None},
        {"scan": 1.0, "map": 5.0, "odom": 1.0},
        ("scan", "map", "odom"),
    )
    assert not healthy
    assert math.isinf(ages["odom"])
    assert ages["scan"] == pytest.approx(0.2)


def test_freshness_accepts_recent_inputs():
    healthy, ages = evaluate_freshness(
        10.0,
        {"scan": 9.8, "map": 7.0, "odom": 9.5},
        {"scan": 1.0, "map": 5.0, "odom": 1.0},
        ("scan", "map", "odom"),
    )
    assert healthy
    assert ages["map"] == 3.0


def test_laser_scan_accepts_one_usable_return():
    valid, reason = validate_laser_scan(
        -math.pi, math.pi, 0.01, 0.1, 12.0,
        (math.inf, math.nan, 0.75),
    )
    assert valid
    assert reason == "ok"


@pytest.mark.parametrize(
    "angle_min,angle_max,increment,range_min,range_max,ranges,reason_part",
    (
        (-math.pi, math.pi, 0.0, 0.1, 12.0, (1.0,), "increment"),
        (math.pi, -math.pi, 0.01, 0.1, 12.0, (1.0,), "angle_max"),
        (-math.pi, math.pi, 0.01, 12.0, 0.1, (1.0,), "range limits"),
        (-math.pi, math.pi, 0.01, 0.1, 12.0, (), "empty"),
        (
            -math.pi,
            math.pi,
            0.01,
            0.1,
            12.0,
            (math.inf, math.nan, 0.05, 20.0),
            "no finite return",
        ),
    ),
)
def test_laser_scan_rejects_invalid_content(
    angle_min,
    angle_max,
    increment,
    range_min,
    range_max,
    ranges,
    reason_part,
):
    valid, reason = validate_laser_scan(
        angle_min,
        angle_max,
        increment,
        range_min,
        range_max,
        ranges,
    )
    assert not valid
    assert reason_part in reason


def test_occupancy_grid_accepts_matching_nonempty_data():
    assert validate_occupancy_grid(100, 80, 0.05, 8000) == (True, "ok")


@pytest.mark.parametrize(
    "width,height,resolution,data_length,reason_part",
    (
        (0, 10, 0.05, 0, "positive"),
        (10, 10, math.nan, 100, "resolution"),
        (10, 10, 0.05, 99, "data length"),
        (10001, 10001, 0.05, 100020001, "sanity limit"),
    ),
)
def test_occupancy_grid_rejects_invalid_content(
    width, height, resolution, data_length, reason_part
):
    valid, reason = validate_occupancy_grid(
        width, height, resolution, data_length)
    assert not valid
    assert reason_part in reason


def test_odometry_accepts_finite_pose_and_twist():
    assert validate_odometry(
        (1.0, 2.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        (0.1, 0.0, 0.0),
        (0.0, 0.0, 0.2),
    ) == (True, "ok")


@pytest.mark.parametrize(
    "position,orientation,linear,angular,reason_part",
    (
        (
            (math.nan, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            "position",
        ),
        (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            "quaternion is zero",
        ),
        (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (math.inf, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            "linear twist",
        ),
        (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, math.nan),
            "angular twist",
        ),
    ),
)
def test_odometry_rejects_invalid_content(
    position, orientation, linear, angular, reason_part
):
    valid, reason = validate_odometry(
        position, orientation, linear, angular)
    assert not valid
    assert reason_part in reason


def test_map_prefix_expands_timestamp():
    value = timestamped_prefix(
        "/maps/footbath_%Y%m%d_%H%M%S",
        datetime(2026, 8, 26, 9, 7, 5),
    )
    assert value == "/maps/footbath_20260826_090705"


def test_auto_motion_lease_is_true_only_while_running():
    assert auto_motion_lease_value("running", "running")
    for state in (
        "waiting_for_health",
        "paused_fault",
        "paused_operator",
        "startup_timeout",
        "exploration_timeout",
        "complete",
    ):
        assert not auto_motion_lease_value(state, "running")

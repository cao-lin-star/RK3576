import math

import pytest

from rk3576_footbath_safety.logic import clamp_diff_drive
from rk3576_footbath_safety.logic import front_sector_minimum
from rk3576_footbath_safety.logic import held_detection_active
from rk3576_footbath_safety.logic import motion_lease_allows
from rk3576_footbath_safety.logic import select_fresh_command
from rk3576_footbath_safety.logic import suspected_glass


def test_auto_speed_is_clamped_to_point_two():
    linear, angular = clamp_diff_drive(1.0, -2.0, 0.20, 0.80)
    assert linear == pytest.approx(0.20)
    assert angular == pytest.approx(-0.80)


def test_non_finite_velocity_is_rejected():
    with pytest.raises(ValueError):
        clamp_diff_drive(math.nan, 0.0, 0.20, 0.80)


def test_front_sector_supports_zero_to_two_pi_scan():
    ranges = [math.inf] * 8
    ranges[0] = 0.6
    ranges[7] = 0.7
    ranges[4] = 0.2
    nearest = front_sector_minimum(
        ranges, 0.0, math.pi / 4.0, 0.05, 10.0, math.pi / 4.0)
    assert nearest == pytest.approx(0.6)


def test_detection_hold_has_bounded_duration():
    assert not held_detection_active(10.0, None, 5.0)
    assert held_detection_active(10.0, 5.0, 5.0)
    assert not held_detection_active(10.01, 5.0, 5.0)
    assert not held_detection_active(4.0, 5.0, 5.0)
    with pytest.raises(ValueError):
        held_detection_active(10.0, 9.0, -1.0)


def test_glass_requires_distance_correspondence_from_either_lidar():
    assert suspected_glass(0.8, math.inf, math.inf, 1.5, 0.15)
    assert not suspected_glass(0.8, 0.9, math.inf, 1.5, 0.15)
    assert suspected_glass(0.8, 0.5, math.inf, 1.5, 0.15)
    assert not suspected_glass(2.0, math.inf, math.inf, 1.5, 0.15)


def test_required_lease_must_be_positive_and_fresh():
    assert motion_lease_allows(10.0, True, True, 9.6, 0.5)
    assert not motion_lease_allows(10.0, True, True, 9.4, 0.5)
    assert not motion_lease_allows(10.0, True, False, 9.9, 0.5)
    assert not motion_lease_allows(10.0, True, True, None, 0.5)


def test_optional_lease_keeps_limiter_available_for_navigation():
    assert motion_lease_allows(10.0, False, False, None, 0.5)


def test_mux_fails_closed_on_multiple_manual_publishers():
    source, command = select_fresh_command(
        10.0, (0.5, 0.1), 9.9, 0.3, (0.2, 0.2), 9.95, 0.3, 2
    )
    assert source == "manual_conflict"
    assert command == (0.0, 0.0)


def test_mux_rejects_negative_manual_publisher_count():
    with pytest.raises(ValueError):
        select_fresh_command(
            10.0, (0.0, 0.0), None, 0.3, (0.0, 0.0), None, 0.3, -1
        )


def test_mux_prefers_fresh_manual_then_fresh_auto():
    source, command = select_fresh_command(
        10.0, (0.5, 0.1), 9.9, 0.3, (0.2, 0.2), 9.95, 0.3)
    assert source == "manual"
    assert command == (0.5, 0.1)
    source, command = select_fresh_command(
        10.4, (0.5, 0.1), 9.9, 0.3, (0.2, 0.2), 10.2, 0.3)
    assert source == "auto"
    assert command == (0.2, 0.2)


def test_mux_fails_to_zero_when_both_sources_expire():
    source, command = select_fresh_command(
        10.0, (0.5, 0.1), 9.0, 0.3, (0.2, 0.2), None, 0.3)
    assert source == "none"
    assert command == (0.0, 0.0)

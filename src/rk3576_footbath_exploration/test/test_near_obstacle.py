import math
import pytest
from rk3576_footbath_exploration.near_obstacle import (
    near_echo, StationaryConfirmation, merge_session_obstacles)


def test_exact_side_threshold_and_front_stable_threshold():
    for key in ('side_left', 'side_right'):
        assert near_echo(key, .12)
        assert not near_echo(key, .120001)
        assert not near_echo(key, .5)
    assert near_echo('front', .20)
    assert not near_echo('front', .200001)
    for d in (math.nan, math.inf, -.1, .019):
        assert not near_echo('front', d)


def test_three_independent_echoes_not_timer_ticks():
    c = StationaryConfirmation()
    assert c.observe('left', 1., (1., 2.)) is None
    for _ in range(20):
        assert c.observe('left', 1., (1., 2.)) is None
    assert c.observe('left', 1.32, (1.005, 2.)) is None
    assert c.observe('left', 1.64, (1., 2.)) == pytest.approx((1.005/3+2/3, 2.))


def test_rotation_drift_and_sample_gap_reset_confirmation():
    c = StationaryConfirmation()
    c.observe('left', 1., (1., 0.))
    c.observe('left', 1.32, (1., 0.))
    assert c.observe('left', 1.64, (0., 1.)) is None
    assert c.observe('left', 2.64, (0., 1.)) is None
    assert len(c.echoes['left']) == 1


def test_session_points_do_not_expand_and_repeat_budget_is_bounded():
    p = dict(x=1., y=2., radius=.04, expires=0., kind='near')
    first = merge_session_obstacles([], [p])
    second = merge_session_obstacles(first, [dict(p, x=1.02)])
    assert len(second) == 1 and second[0]['x'] == 1. and second[0]['radius'] == .04
    assert second[0]['expires'] == 0. and second[0]['encounters'] == 2
    with pytest.raises(ValueError):
        merge_session_obstacles(second, [p])
    assert first[0]['encounters'] == 1  # no partial in-place mutation


def test_two_probes_same_obstacle_count_as_one_encounter():
    a = dict(x=1., y=2., radius=.04, expires=0., kind='near')
    zones = merge_session_obstacles([], [a, dict(a, x=1.02)])
    assert len(zones) == 1 and zones[0]['encounters'] == 1

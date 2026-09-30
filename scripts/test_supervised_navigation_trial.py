import importlib.util
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('supervised_navigation_trial',
    Path(__file__).with_name('supervised_navigation_trial.py'))
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


def state():
    return dict(mode='navigation', initialized=True, transitioning=False,
                home_status_fresh=True, healthy=dict(scan=True, map=True, odom=True),
                pose=dict(x=0, y=0, yaw=0), nav_state='idle', home=dict(phase='ready', available=True))


def test_mapping_or_unlocalized_refused():
    for key, value in [('mode', 'auto_mapping'), ('initialized', False),
                       ('manual_active', True), ('home_status_fresh', False)]:
        current = state(); current[key] = value
        with pytest.raises(RuntimeError):
            trial.check_state(current, initial=True)


def test_stale_sensor_refused():
    current = state(); current['healthy']['scan'] = False
    with pytest.raises(RuntimeError):
        trial.check_state(current)


def test_initial_active_goal_refused_but_running_goal_allowed():
    current = state(); current['nav_state'] = 'active'
    with pytest.raises(RuntimeError):
        trial.check_state(current, initial=True)
    trial.check_state(current)


def test_thirty_seconds_stationary_or_looping_detected():
    assert trial.progress_reason([(t, 0, 0, 0) for t in range(31)])
    assert trial.progress_reason([(t, .001*t, 0, t*.2) for t in range(31)])
    assert trial.progress_reason([(t, 0, 0, 0) for t in range(30)]) is None


def test_normal_progress_and_wrapped_yaw():
    assert trial.progress_reason([(t, .03*t, 0, 0) for t in range(31)]) is None
    assert trial.progress_reason([(t, .03*t, 0, math.atan2(math.sin(t*.1), math.cos(t*.1)))
                                  for t in range(31)]) is None


def test_zero_requires_fresh_odom_selected_and_lease():
    sample = dict(odom=(10, (0, 0)), selected=(10, (0, 0)), lease=(10, False))
    assert trial.quiet(sample, 10.1)
    assert not trial.quiet(sample, 11.1)
    sample['lease'] = (10, True)
    assert not trial.quiet(sample, 10.1)
    sample['lease'] = (10, False); sample['selected'] = (10, (.01, 0))
    assert not trial.quiet(sample, 10.1)

from types import SimpleNamespace
from unittest.mock import Mock
import time

from rk3576_footbath_mobile.gateway import Gateway


def gateway():
    g = Mock()
    g.pending_departure_goal = dict(x=1., y=2., yaw=0.)
    g.departure_deadline = time.monotonic()+50
    g.mode = 'navigation'
    g.home_seen = time.monotonic()
    g.home_status = dict(phase='undocking', message='正在出站')
    g.departure_future.done.return_value = True
    g.departure_future.result.return_value = SimpleNamespace(success=True)
    return g


def test_no_goal_during_exit_or_waiting():
    for phase in ('undocking', 'dock_waiting', 'ready'):
        g = gateway()
        g.home_status['phase'] = phase
        Gateway._tick_departure(g)
        g._goal.assert_not_called()


def test_goal_only_after_fresh_navigation_ready():
    g = gateway()
    target = g.pending_departure_goal.copy()
    g.home_status['phase'] = 'navigation_ready'
    Gateway._tick_departure(g)
    g._goal.assert_called_once_with(target)
    assert g.pending_departure_goal is None


def test_stale_completion_does_not_release_goal():
    g = gateway()
    g.home_status['phase'] = 'navigation_ready'
    g.home_seen = time.monotonic()-3
    Gateway._tick_departure(g)
    g._goal.assert_not_called()


def test_failed_departure_cancels_pending_target():
    g = gateway()
    g.home_status['phase'] = 'failed'
    Gateway._tick_departure(g)
    assert g.pending_departure_goal is None
    g._goal.assert_not_called()
    g._cancel_nav.assert_called_once()


def test_timeout_never_sends_target():
    g = gateway()
    g.departure_deadline = 0
    Gateway._tick_departure(g)
    g._goal.assert_not_called()
    g._cancel_nav.assert_called_once()

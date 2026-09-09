"""No-hardware tests of home capture, cancellation and motion gating."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from rk3576_footbath_exploration.home_return import HomeReturn, home_pose_allowed
from rk3576_footbath_exploration.supervisor import ExplorationSupervisor


@pytest.mark.parametrize('first,current,seconds,allowed', [
    (None, (0, 0), 2, False),
    ((0, 0), (0.01, 0), 2, True),
    ((0, 0), (0.05, 0), 2, False),
    ((0, 0), (0, 0), 0.5, False),
    ((0, 0), (float('nan'), 0), 2, False),
])
def test_capture_only_at_stationary_start(first, current, seconds, allowed):
    assert home_pose_allowed(first, current, seconds) is allowed


def manager():
    h = HomeReturn.__new__(HomeReturn)
    h.localization = True
    h.handoff = None
    h.dock = Mock(enabled=False, active=False, kind=None)
    h.node = Mock()
    h.node.PAUSED_FAULT = 'paused_fault'
    h.node.COMPLETE = 'complete'
    h.node._home_inputs_ready.return_value = True
    h.node._cancel_client.service_is_ready.return_value = True
    h.pose = object()
    h.current_pose = Mock(return_value=object())
    h.nav = Mock()
    h.nav.server_is_ready.return_value = True
    h.phase = 'ready'
    h.generation = 1
    h.handle = None
    h.stage_started = 99
    h.node._format_ages.return_value = 'scan=0.1s, map=1s, odom=0.1s'
    h.waiting_since = None
    h.healthy_since = None
    h.recovery_count = 0
    h.max_recoveries = 5
    h.recovery_timeout = 8.0
    h.stable_time = 1.0
    return h


def test_no_recorded_home_rejected_without_motion():
    h = manager()
    h.pose = None
    assert not h.request()[0]
    h.nav.send_goal_async.assert_not_called()
    h.node._pause.assert_not_called()


def test_unhealthy_return_rejected():
    h = manager()
    h.node._home_inputs_ready.return_value = False
    assert not h.request()[0]
    h.nav.send_goal_async.assert_not_called()


def test_return_first_waits_for_cancellation():
    h = manager()
    assert h.request()[0]
    assert h.phase == 'preparing'
    h.node._cancel_client.call_async.assert_called_once()
    h.nav.send_goal_async.assert_not_called()


def test_mapping_return_requests_save_not_navigation():
    h = manager()
    h.localization = False
    assert h.request()[0]
    assert h.phase == 'saving_map'
    h.nav.send_goal_async.assert_not_called()
    h.node._publish_lease.assert_not_called()


def test_failed_save_never_requests_handoff():
    h = manager()
    h.last_motion = 0
    h.started = 0
    h.active_goals = False
    h.cancel_future = Mock(done=lambda:True, result=lambda:SimpleNamespace(return_code=0))
    h.save_started = True
    h.node._save_in_progress = False
    h.node._save_state = 'save_map_failed_result_1'
    h._save_for_handoff(10)
    assert h.phase == 'failed'
    assert h.handoff is None
    h.nav.send_goal_async.assert_not_called()


def test_delayed_acceptance_after_stop_is_canceled():
    h = manager()
    old = h.generation
    h.phase = 'sending'
    h.interrupt('operator stop')
    handle = Mock(accepted=True)
    h._accepted(Mock(result=Mock(return_value=handle)), old)
    handle.cancel_goal_async.assert_called_once()
    assert h.phase == 'canceled'


def test_success_stops_and_preserves_home():
    h = manager()
    home = h.pose
    h._result(Mock(result=Mock(return_value=SimpleNamespace(status=4))), 1)
    assert h.phase == 'arrived'
    assert h.pose is home
    h.node._pause.assert_called_once()


@pytest.mark.parametrize('state,allowed', [
    ('waiting_for_health', False), ('return_preparing', False),
    ('paused_operator', False), ('complete', False),
    ('returning_home', True), ('running', True),
])
def test_lease_only_for_active_motion_states(state, allowed):
    node = SimpleNamespace(_state=state, RUNNING='running')
    assert ExplorationSupervisor._motion_allowed(node) is allowed


def test_preparing_does_not_send_when_an_old_goal_is_active(monkeypatch):
    h = manager()
    h.phase = 'preparing'
    h.started = 99
    h.timeout = 300
    h.cancel_done_at = 99
    h.cancel_future = Mock()
    h.cancel_future.done.return_value = True
    h.cancel_future.result.return_value = SimpleNamespace(return_code=0, goals_canceling=[])
    h.active_goals = True
    h.last_publish = 101
    h._send = Mock()
    monkeypatch.setattr('rk3576_footbath_exploration.home_return.time.monotonic', lambda: 101)
    h.tick(True)
    h._send.assert_not_called()
    h.active_goals = False
    h.tick(True)
    h._send.assert_called_once()


def test_operator_stop_revokes_lease_and_cancels_return_handle():
    h = manager()
    h.phase = 'returning'
    handle = Mock()
    h.handle = handle
    n = SimpleNamespace(
        home=h, _state='returning_home', RUNNING='running',
        COMPLETE='complete', TIMED_OUT='timed_out',
        _publish_pause=Mock(), _publish_lease=Mock(), _publish_zero=Mock(),
        _cancel_navigation=Mock(), get_logger=Mock())
    ExplorationSupervisor._pause(n, 'paused_operator', 'operator stop')
    handle.cancel_goal_async.assert_called_once()
    n._publish_lease.assert_called_once_with(False)
    n._publish_zero.assert_called_once()
    assert n._state == 'paused_operator'


def test_health_loss_stops_but_preserves_return_intent():
    h = manager()
    h.phase = 'returning'
    handle = Mock()
    h.handle = handle
    h._wait_for_health('TF age=1.1s', 100)
    assert h.phase == 'waiting_health'
    assert h.pose is not None
    h.node._publish_lease.assert_called_with(False)
    h.node._publish_zero.assert_called_once()
    handle.cancel_goal_async.assert_called_once()
    assert h.node._state == 'return_waiting'


def test_stable_recovery_replans_without_resetting_total_timeout(monkeypatch):
    h = manager()
    h.started = 90
    h.timeout = 300
    h.last_publish = 104
    h.phase = 'returning'
    h._wait_for_health('TF', 100)
    monkeypatch.setattr('rk3576_footbath_exploration.home_return.time.monotonic', lambda: 101)
    h.tick(True)
    assert h.phase == 'waiting_health'
    monkeypatch.setattr('rk3576_footbath_exploration.home_return.time.monotonic', lambda: 102.1)
    h.tick(True)
    assert h.phase == 'preparing'
    assert h.started == 90
    h.nav.send_goal_async.assert_not_called()


def test_persistent_health_failure_cancels(monkeypatch):
    h = manager()
    h.started = 90
    h.timeout = 300
    h.last_publish = 110
    h.phase = 'returning'
    h._wait_for_health('TF', 100)
    monkeypatch.setattr('rk3576_footbath_exploration.home_return.time.monotonic', lambda: 108.1)
    h.tick(False)
    assert h.phase == 'failed'
    h.nav.send_goal_async.assert_not_called()

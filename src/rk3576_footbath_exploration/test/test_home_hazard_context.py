"""No hardware/motion tests for preserving a Nav2 return intent through a cliff."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped

from rk3576_footbath_exploration.home_return import HomeReturn


@pytest.fixture
def home(monkeypatch):
    monkeypatch.setattr(
        'rk3576_footbath_exploration.home_return.time.monotonic', lambda: 100.0)
    h = HomeReturn.__new__(HomeReturn)
    h.node = Mock()
    h.node._state = 'returning_home'
    h.node.PAUSED_FAULT = 'paused_fault'
    h.node._home_behavior_tree = ''
    h.node._cancel_client.service_is_ready.return_value = True
    h.node._cancel_client.call_async.return_value = Mock(
        done=lambda: True,
        result=lambda: SimpleNamespace(return_code=0, goals_canceling=[]))
    h.node.get_clock.return_value.now.return_value.to_msg.return_value = PoseStamped().header.stamp
    h.nav = Mock()
    h.dock = Mock(enabled=True, active=False, distance=.5)
    h.pose = PoseStamped()
    h.pose.header.frame_id = 'map'
    h.pose.pose.position.x, h.pose.pose.position.y = 1., 2.
    h.pose.pose.orientation.w = 1.
    h.phase, h.message = 'returning', ''
    h.generation = 7
    h.handle = Mock()
    h.started, h.timeout = 50., 300.
    h.last_publish = 100.
    h.active_goals = True
    h.last_status = 100.
    h.distance = 3.
    h._hazard_token = None
    h._return_target = h._navigation_target()
    h.current_pose = Mock(return_value=h.pose)
    h.publish = Mock()
    return h


def test_suspension_revokes_permission_cancels_and_invalidates_old_result(home):
    old_generation, handle = home.generation, home.handle
    token = home.suspend_for_hazard()
    assert token == old_generation + 1
    assert home.phase == 'hazard_suspended'
    assert home.phase in home.BUSY
    home.node._publish_lease.assert_called_once_with(False)
    home.node._publish_zero.assert_called_once()
    handle.cancel_goal_async.assert_called_once()
    assert home.handle is None
    home._result(Mock(result=lambda: SimpleNamespace(status=4)), old_generation)
    home._feedback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=0.)), old_generation)
    assert home.phase == 'hazard_suspended'
    assert home.distance == 3.


@pytest.mark.parametrize('phase', [
    'preparing', 'sending', 'waiting_health', 'dock_preparing', 'aligning',
    'docking', 'undocking', 'arrived', 'navigation_ready'])
def test_only_navigation_return_leg_can_be_suspended(home, phase):
    home.phase = phase
    assert home.suspend_for_hazard() is None
    home.node._publish_lease.assert_not_called()


def test_dock_activity_or_other_owner_cannot_be_suspended(home):
    home.dock.active = True
    assert home.suspend_for_hazard() is None
    home.dock.active = False
    home.node._state = 'running'
    assert home.suspend_for_hazard() is None


def test_resume_preserves_original_target_and_deadline_and_waits_for_old_goal(home, monkeypatch):
    token = home.suspend_for_hazard()
    home.node._state = 'hazard_recovery'
    home.pose.pose.position.x = 99.  # A changed pose cannot replace the old staging goal.
    assert home.resume_after_hazard(token)
    assert home.phase == 'preparing'
    assert home.started == 50.
    assert home.node._state == 'return_preparing'
    assert not home.hazard_resume_valid(token)
    home.nav.send_goal_async.assert_not_called()
    monkeypatch.setattr(
        'rk3576_footbath_exploration.home_return.time.monotonic', lambda: 101.)
    home.tick(True)
    monkeypatch.setattr(
        'rk3576_footbath_exploration.home_return.time.monotonic', lambda: 102.1)
    home.tick(True)
    home.nav.send_goal_async.assert_not_called()
    home.active_goals = False
    home.tick(True)
    goal = home.nav.send_goal_async.call_args.args[0]
    assert goal.pose.pose.position.x == 1.5
    assert goal.pose.pose.position.y == 2.
    assert goal.pose.pose.orientation.w == 1.
    assert home.started == 50.


def test_operator_interruption_permanently_revokes_resume_token(home):
    token = home.suspend_for_hazard()
    home.interrupt('operator stop')
    assert home.phase == 'canceled'
    assert not home.hazard_resume_valid(token)
    assert not home.resume_after_hazard(token)
    home.nav.send_goal_async.assert_not_called()


def test_resume_token_is_single_use(home):
    token = home.suspend_for_hazard()
    assert home.resume_after_hazard(token)
    assert not home.resume_after_hazard(token)
    home.node._cancel_client.call_async.assert_called_once()


def test_original_timeout_applies_during_recovery(home, monkeypatch):
    token = home.suspend_for_hazard()
    monkeypatch.setattr(
        'rk3576_footbath_exploration.home_return.time.monotonic', lambda: 350.)
    assert not home.hazard_resume_valid(token)
    assert not home.resume_after_hazard(token)
    home.tick(True)
    assert home.phase == 'failed'
    home.nav.send_goal_async.assert_not_called()


def test_expired_return_cannot_open_recovery(home):
    home.started = -201.
    assert home.suspend_for_hazard() is None


def test_resume_waits_for_cancel_service(home):
    token = home.suspend_for_hazard()
    home.node._cancel_client.service_is_ready.return_value = False
    assert not home.resume_after_hazard(token)
    assert home.hazard_resume_valid(token)
    home.nav.send_goal_async.assert_not_called()


def test_suspended_home_tick_does_not_compete_with_recovery_channel(home):
    home.suspend_for_hazard()
    home.node._publish_zero.reset_mock()
    home.node._publish_lease.reset_mock()
    home.tick(False)
    assert home.phase == 'hazard_suspended'
    home.node._publish_zero.assert_not_called()
    home.node._publish_lease.assert_not_called()


def test_cancel_exception_fails_closed(home):
    home.handle.cancel_goal_async.side_effect = RuntimeError('cancel failure')
    assert home.suspend_for_hazard() is None
    assert home.phase == 'failed'
    assert home._hazard_token is None
    home.nav.send_goal_async.assert_not_called()

"""Offline return-target and AMCL acknowledgement tests; no ROS node is started."""
import math
import queue
import threading
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped

from rk3576_footbath_mobile.gateway import Gateway
from rk3576_footbath_mobile.return_destinations import matches_manual_initial


REQUEST = dict(x=2., y=3., yaw=.4)
OLD = dict(x=-1., y=-2., yaw=.1)
DOCK = dict(x=8., y=9., yaw=-.7)


def gateway():
    g = SimpleNamespace(mode='navigation', initialized=True, initial_state='confirmed',
        initial_source='manual', initial_request=dict(OLD), initial_request_at=0.,
        initial_request_ros_ns=0, confirmed_initial_pose=dict(OLD), saved_map_home=dict(DOCK),
        pending_departure_goal=None, return_requested=False, nav_state='succeeded',
        count_subscribers=Mock(return_value=1), initial_pub=Mock(), get_clock=Mock(),
        _goal=Mock(), _service=Mock(), home_status=dict(phase='navigation_ready'))
    g.obstacle_view=SimpleNamespace(pending={})
    g.get_clock().now().nanoseconds = 100_000_000_000
    for name in ('_initial', '_amcl_pose', '_return_initial_pose'):
        setattr(g, name, MethodType(getattr(Gateway, name), g))
    return g


def amcl(pose=None, stamp=101, frame='map'):
    pose = pose or REQUEST
    msg = PoseWithCovarianceStamped()
    msg.header.stamp.sec = stamp; msg.header.frame_id = frame
    msg.pose.pose.position.x = pose['x']; msg.pose.pose.position.y = pose['y']
    msg.pose.pose.orientation.z = math.sin(pose['yaw']/2)
    msg.pose.pose.orientation.w = math.cos(pose['yaw']/2)
    return msg


def test_confirmed_explicit_pose_is_a_separate_copy_not_amcl_drift():
    g = gateway(); g._initial(REQUEST)
    assert g.confirmed_initial_pose == OLD  # pending must not overwrite
    observed = dict(x=2.1, y=3.1, yaw=.5)
    g._amcl_pose(amcl(observed))
    assert g.initialized and g.initial_state == 'confirmed'
    assert g.confirmed_initial_pose == REQUEST and g.saved_map_home == DOCK
    assert g.confirmed_initial_pose is not g.initial_request
    g._amcl_pose(amcl(dict(x=2.4, y=3., yaw=.4), stamp=102))
    assert g.confirmed_initial_pose == REQUEST
    g._return_initial_pose()
    g._goal.assert_called_once_with(REQUEST)
    g._service.assert_not_called()


@pytest.mark.parametrize('source', ['saved_home', 'handoff'])
def test_automatic_initialization_never_creates_manual_return_target(source):
    g = gateway(); g.confirmed_initial_pose = None
    g._initial(REQUEST, source=source, handoff=source=='handoff')
    g._amcl_pose(amcl())
    assert g.initialized and g.confirmed_initial_pose is None
    assert g.saved_map_home == DOCK
    with pytest.raises(ValueError, match='尚无手动'):
        g._return_initial_pose()
    g._goal.assert_not_called()


@pytest.mark.parametrize('msg', [amcl(stamp=99), amcl(frame='odom'),
    amcl(dict(x=5., y=3., yaw=.4)), amcl(dict(x=2., y=3., yaw=1.1))])
def test_queued_or_unrelated_amcl_does_not_confirm_or_replace_target(msg):
    g = gateway(); g._initial(REQUEST); g._amcl_pose(msg)
    assert not g.initialized and g.initial_state == 'waiting'
    assert g.confirmed_initial_pose == OLD and g.saved_map_home == DOCK


def test_invalid_or_failed_publish_preserves_previous_confirmed_target():
    g = gateway()
    with pytest.raises(ValueError): g._initial(dict(x=math.nan, y=0., yaw=0.))
    assert g.confirmed_initial_pose == OLD and g.initial_state == 'confirmed'
    g.initial_pub.publish.side_effect = RuntimeError('publish failure')
    with pytest.raises(RuntimeError): g._initial(REQUEST)
    assert g.confirmed_initial_pose == OLD and g.initial_request == OLD


@pytest.mark.parametrize('state', ['active', 'sending', 'hazard_recovery', 'resuming'])
def test_initialization_while_navigation_owns_motion_is_rejected(state):
    g = gateway(); g.nav_state = state
    with pytest.raises(ValueError, match='先取消'): g._initial(REQUEST)
    assert g.confirmed_initial_pose == OLD
    g.initial_pub.publish.assert_not_called()


@pytest.mark.parametrize('phase', ['undocking', 'docking', 'aligning', 'hazard_suspended'])
def test_initial_return_cannot_bypass_docking_or_hazard_owner(phase):
    g = gateway(); g.home_status['phase'] = phase
    g._goal = MethodType(Gateway._goal, g)
    with pytest.raises(ValueError, match='基站/返航'): g._return_initial_pose()
    g._service.assert_not_called()


def test_failed_localization_cannot_return_using_old_point():
    g = gateway(); g.initialized = False
    g.nav_owner = SimpleNamespace(phase='idle')
    g._goal = MethodType(Gateway._goal, g)
    with pytest.raises(ValueError, match='not localized'): g._return_initial_pose()
    assert g.confirmed_initial_pose == OLD


def test_session_exit_clears_manual_point():
    g = gateway()
    for name in ('_invalidate_navigation', '_release_manual', '_cancel_nav', '_stop_child_group'):
        setattr(g, name, Mock())
    g.return_transition = Mock(); g.child = None
    Gateway._stop_child(g)
    assert g.mode == 'idle' and g.confirmed_initial_pose is None
    assert g.initial_request_ros_ns == 0


def test_api_command_uses_existing_goal_owner_not_dock_service():
    g = gateway(); g.commands = queue.Queue()
    event = threading.Event(); box = {}
    g.commands.put(('return_initial_pose', {}, event, box, time.monotonic()+10))
    Gateway._process(g)
    assert event.is_set() and box['result']['ok']
    g._goal.assert_called_once_with(OLD)
    g._service.assert_not_called()


def test_yaw_wrap_and_nonfinite_acknowledgement():
    request = dict(x=0., y=0., yaw=math.pi-.1)
    assert matches_manual_initial(request,dict(x=0., y=0., yaw=-math.pi+.1),1,2,'map')
    assert not matches_manual_initial(request,dict(x=math.nan, y=0., yaw=0.),1,2,'map')
    assert not matches_manual_initial(request,request,0,2,'map')

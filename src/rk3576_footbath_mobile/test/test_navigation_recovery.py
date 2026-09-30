"""Offline ownership/cancellation tests; no node or motion publisher is started."""
import json
import time
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from std_msgs.msg import String, UInt8

from rk3576_footbath_mobile.gateway import Gateway
from rk3576_footbath_mobile.navigation_owner import NavigationOwner


def command(owner, action='suspend', recovery_id='hazard-1', **changes):
    data=dict(action=action,session=owner.session,token=owner.token,recovery_id=recovery_id)
    data.update(changes)
    return data


def gateway():
    owner=NavigationOwner()
    target=dict(x=1.,y=2.,yaw=.3)
    attempt=owner.begin(target)
    owner.finish('active')
    g=SimpleNamespace(nav_owner=owner,goal_handle=Mock(),nav_context_pub=Mock(),
        mode='navigation',return_requested=False,manual_active=False,initialized=True,child=Mock(),
        pose_map=dict(x=0.,y=0.,yaw=0.),healthy=Mock(return_value=True),
        control_source=0,control_source_at=time.monotonic(),nav_state='active',
        nav_message='',nav_feedback=None,goal_pose=target,get_logger=Mock(),
        _send_owned_goal=Mock(),_service=Mock(),cancel_client=Mock(),
        pending_departure_goal=None,home_status=dict(phase='navigation_ready'),
        nav=Mock(),get_clock=Mock())
    g.child.poll.return_value=None
    g.get_clock().now().to_msg.return_value=Time()
    for name in ('_publish_navigation_context','_cancel_owned_handle','_invalidate_navigation',
                 '_navigation_recovery','_goal_response','_goal_feedback','_goal_result',
                 '_control_source','_cancel_nav'):
        setattr(g,name,MethodType(getattr(Gateway,name),g))
    return g,attempt


def send(g,data):
    msg=String(); msg.data=json.dumps(data)
    g._navigation_recovery(msg)


def test_suspend_preserves_goal_and_invalidates_old_attempt():
    g,attempt=gateway(); target=dict(g.nav_owner.goal); handle=g.goal_handle
    send(g,command(g.nav_owner))
    assert g.nav_owner.phase=='suspended'
    assert g.nav_owner.goal==target
    assert g.nav_owner.token==attempt[0]
    assert not g.nav_owner.current(*attempt)
    handle.cancel_goal_async.assert_called_once()
    context=json.loads(g.nav_context_pub.publish.call_args[0][0].data)
    assert context['phase']=='suspended' and context['recovery_id']=='hazard-1'


@pytest.mark.parametrize('status',[GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED])
def test_old_result_and_feedback_cannot_erase_suspended_context(status):
    g,attempt=gateway(); send(g,command(g.nav_owner))
    future=Mock(); future.result.return_value=SimpleNamespace(status=status)
    g._goal_result(future,*attempt)
    g._goal_feedback(object(),*attempt)
    assert g.nav_owner.phase=='suspended' and g.nav_owner.token==attempt[0]
    assert g.nav_state=='hazard_recovery'
    future.result.assert_not_called()


def test_late_acceptance_is_canceled_not_resumed():
    g,attempt=gateway(); send(g,command(g.nav_owner))
    late_handle=Mock(accepted=True); future=Mock(); future.result.return_value=late_handle
    g._goal_response(future,*attempt)
    late_handle.cancel_goal_async.assert_called_once()
    late_handle.get_result_async.assert_not_called()
    assert g.nav_owner.phase=='suspended'


def test_resume_reuses_logical_target_but_replaces_attempt():
    g,attempt=gateway(); request=command(g.nav_owner)
    send(g,request); request['action']='resume'
    # Supervisor still owns zero-output gating. Gateway must not wait for
    # navigation_ready, which would deadlock waiting for its own ACK.
    g.home_status['phase']='hazard_suspended'
    send(g,request)
    target,new_attempt=g._send_owned_goal.call_args[0]
    assert target==dict(x=1.,y=2.,yaw=.3)
    assert new_attempt[0]==attempt[0] and new_attempt[1]>attempt[1]
    assert g.nav_owner.phase=='resuming'
    handle=Mock(accepted=True); future=Mock(); future.result.return_value=handle
    g._goal_response(future,*new_attempt)
    assert g.nav_owner.phase=='active' and g.nav_owner.recovery_id=='hazard-1'
    assert json.loads(g.nav_context_pub.publish.call_args[0][0].data)['phase']=='active'
    g._service.assert_not_called()


def test_duplicate_resume_and_delayed_suspend_do_not_send_again():
    g,_=gateway(); request=command(g.nav_owner)
    send(g,request); request['action']='resume'; send(g,request); send(g,request)
    g.nav_owner.finish('active')
    request['action']='suspend'; send(g,request)
    assert g.nav_owner.phase=='active'
    g._send_owned_goal.assert_called_once()


@pytest.mark.parametrize('change',[
    dict(session='old-session'),dict(token='old-goal'),dict(recovery_id=''),
    dict(recovery_id=None),dict(recovery_id=123),
])
def test_mismatched_suspend_is_ignored(change):
    g,_=gateway(); send(g,command(g.nav_owner,**change))
    assert g.nav_owner.phase=='active'
    g._send_owned_goal.assert_not_called()


def test_different_recovery_resume_and_abort_are_ignored():
    g,_=gateway(); send(g,command(g.nav_owner))
    send(g,command(g.nav_owner,'resume','old-recovery'))
    send(g,command(g.nav_owner,'abort','old-recovery'))
    assert g.nav_owner.phase=='suspended'
    g._send_owned_goal.assert_not_called()


@pytest.mark.parametrize('boundary',['cancel','stop','mode_change','ps2','debug'])
def test_user_boundary_invalidates_pending_recovery_forever(boundary):
    g,_=gateway(); request=command(g.nav_owner)
    send(g,request)
    if boundary=='cancel': g._cancel_nav()
    elif boundary in ('ps2','debug'):
        msg=UInt8(); msg.data=3 if boundary=='ps2' else 2; g._control_source(msg)
        g._service.assert_called_once_with('stop')
    else: g._invalidate_navigation(boundary)
    request['action']='resume'; send(g,request)
    assert g.nav_owner.token=='' and g.nav_owner.phase=='canceled'
    g._send_owned_goal.assert_not_called()


@pytest.mark.parametrize('bad',[
    'mode','localized','child','tf','health','source','source_stale','return','manual',
])
def test_resume_fails_closed_when_navigation_precondition_lost(bad):
    g,_=gateway(); request=command(g.nav_owner); send(g,request)
    if bad=='mode': g.mode='idle'
    elif bad=='localized': g.initialized=False
    elif bad=='child': g.child.poll.return_value=1
    elif bad=='tf': g.pose_map=None
    elif bad=='health': g.healthy.return_value=False
    elif bad=='source': g.control_source=2
    elif bad=='source_stale': g.control_source_at=0.
    elif bad=='return': g.return_requested=True
    elif bad=='manual': g.manual_active=True
    request['action']='resume'; send(g,request)
    assert g.nav_owner.phase=='failed' and not g.nav_owner.token
    g._send_owned_goal.assert_not_called()


def test_abort_before_suspend_ack_revokes_goal():
    g,_=gateway(); send(g,command(g.nav_owner,'abort'))
    assert g.nav_owner.phase=='failed' and not g.nav_owner.token


def test_takeover_revokes_pending_departure_and_supervisor_lease():
    g,_=gateway(); g.nav_owner.invalidate()
    g.pending_departure_goal=dict(x=1.,y=2.,yaw=0.)
    g._control_source(UInt8(data=3))
    assert g.pending_departure_goal is None
    g._service.assert_called_once_with('stop')


@pytest.mark.parametrize('status',[GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED])
def test_terminal_goal_cannot_be_suspended_or_resumed(status):
    g,attempt=gateway(); request=command(g.nav_owner)
    future=Mock(); future.result.return_value=SimpleNamespace(status=status)
    g._goal_result(future,*attempt)
    phase=g.nav_owner.phase
    send(g,request); request['action']='resume'; send(g,request)
    g._goal_feedback(object(),*attempt)
    assert g.nav_owner.phase==phase
    g._send_owned_goal.assert_not_called()


def test_goal_replacement_changes_token_and_ignores_old_callbacks():
    g,old_attempt=gateway()
    new_attempt=g.nav_owner.begin(dict(x=5.,y=6.,yaw=1.))
    g.nav_state='sending'
    g._goal_result(Mock(),*old_attempt); g._goal_feedback(object(),*old_attempt)
    assert new_attempt[0]!=old_attempt[0]
    assert g.nav_owner.phase=='sending' and g.nav_state=='sending'


def test_new_goal_is_not_allowed_during_recovery():
    g,_=gateway(); send(g,command(g.nav_owner))
    with pytest.raises(ValueError,match='恢复，请先取消任务'):
        Gateway._goal(g,dict(x=3.,y=4.,yaw=0.))


def test_return_recovery_blocks_normal_goal():
    g,_=gateway(); g.home_status['phase']='hazard_suspended'
    with pytest.raises(ValueError,match='返航任务'):
        Gateway._goal(g,dict(x=3.,y=4.,yaw=0.))


def test_resume_send_keeps_token_and_binds_action_callbacks():
    g,_=gateway(); request=command(g.nav_owner); send(g,request)
    request['action']='resume'; attempt=g.nav_owner.resume(request)
    Gateway._send_owned_goal(g,dict(g.nav_owner.goal),attempt)
    g.nav.send_goal_async.assert_called_once()
    assert g.nav_owner.phase=='resuming' and g.nav_owner.current(*attempt)


def test_send_failure_is_terminal_not_unbounded_retry():
    g,_=gateway(); request=command(g.nav_owner); send(g,request)
    request['action']='resume'; attempt=g.nav_owner.resume(request)
    g.nav.send_goal_async.side_effect=RuntimeError('server gone')
    Gateway._send_owned_goal(g,dict(g.nav_owner.goal),attempt)
    assert g.nav_owner.phase=='failed' and not g.nav_owner.token


def test_sessions_unique_across_gateway_restarts():
    assert NavigationOwner().session!=NavigationOwner().session

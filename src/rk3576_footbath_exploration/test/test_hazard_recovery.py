"""No hardware: exercise cancellation, bounded reverse and failure paths."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import math
import pytest
from builtin_interfaces.msg import Time
from rk3576_footbath_exploration.hazard_recovery import (
    HazardRecovery, hazard_kind, ground_clear, reverse_progress)


def test_thresholds_and_hysteresis():
    assert hazard_kind(.28, .16, .54) == 'cliff'
    assert hazard_kind(.155, .28, .54) == 'cliff'
    assert hazard_kind(.155, .16, .20) == 'sonar'
    assert hazard_kind(.279999, .279999, .25) is None
    assert not ground_clear(.16, .260001, .30)
    assert not ground_clear(.260001, .16, .30)
    assert ground_clear(.26, .26, .28)
    with pytest.raises(ValueError): hazard_kind(math.nan,.16,.3)


@pytest.mark.parametrize('pose', [(0,.04,0),(0,0,.11),(.02,0,0),(math.nan,0,0)])
def test_reverse_rejects_drift_and_wrong_direction(pose):
    with pytest.raises(ValueError): reverse_progress((0,0,0),pose)


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setenv("FOOTBATH_SIDE_ULTRASONIC_ENABLED", "0")
    clock=[1000.]
    monkeypatch.setattr('rk3576_footbath_exploration.hazard_recovery.time.monotonic',lambda:clock[0])
    n=Mock()
    n.get_clock.return_value.now.return_value.to_msg.return_value=Time()
    n.create_publisher.side_effect=lambda *args:Mock()
    n.RUNNING='running'; n.PAUSED_FAULT='paused_fault'; n._state='running'
    n.declare_parameter.side_effect=lambda key,default:NS(value=default)
    n.home.current_pose.return_value=NS(pose=NS(position=NS(x=0.,y=0.),
                                               orientation=NS(x=0.,y=0.,z=0.,w=1.)))
    n.home.dock.scan=None; n.home.dock.scan_at=0.
    n.home.dock.odom=(0.,0.,0.); n.home.dock.odom_at=1000.
    n.home.active_goals=False; n.home.last_status=1001.
    n.home.last_motion=999.
    n._cancel_client.service_is_ready.return_value=True
    n._cancel_client.call_async.return_value.done.return_value=True
    n._cancel_client.call_async.return_value.result.return_value=NS(return_code=0)
    def pause(state,reason): n._state=state; n._reason=reason
    n._pause.side_effect=pause
    h=HazardRecovery(n)
    h.start_space=True;h.start_space_at=clock[0]
    h.rear_observed=Mock(return_value=True)  # explicit coverage fixture; failure cases override
    def step(dt=0., values=(.30,.16,.54), progress=0., source_age=0.):
        clock[0]+=dt
        h.ranges={k:(v,clock[0]) for k,v in zip(('left','right','sonar'),values)}
        h.start_space_at=clock[0]
        h.source=1; h.source_at=clock[0]-source_age; h.fault_at=clock[0]
        n.home.dock.odom_at=clock[0]; n.home.dock.odom=(-progress,0.,0.)
        return h.tick(True)
    return h,n,clock,step


def test_cancel_reverse_stable_mark_then_resume(rig):
    h,n,clock,step=rig
    assert step() and h.phase=='cancelling'
    assert step(.7) and h.phase=='reversing'
    assert h.cmdpub.publish.call_args.args[0].linear.x==-.03
    step(.5,(.155,.16,.3),.03)
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    step(.6,(.155,.16,.3),.03)
    assert h.phase=='marking' and len(h.zones)==1
    n._publish_resume.assert_not_called()
    h.applied={k:(h.marked_stamp,clock[0]) for k in ('local_costmap','global_costmap')}
    step(2.1,(.155,.16,.3),.03)
    assert not h.active and n._state=='running'
    n._publish_resume.assert_called_once()


def test_cancel_never_acknowledged_no_reverse(rig):
    h,n,clock,step=rig
    n._cancel_client.call_async.return_value.done.return_value=False
    step(); step(4.1)
    assert h.phase=='failed'
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


@pytest.mark.parametrize('dt,distance', [(6.1,.1),(1.,.15),(2.1,0.)])
def test_reverse_limits_fail_closed(rig,dt,distance):
    h,n,clock,step=rig
    step(); step(.7); step(dt,progress=distance)
    assert h.phase=='failed' and n._state=='paused_fault'
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


def test_lost_tof_not_recorded_as_cliff(rig):
    h,n,clock,step=rig
    step(); h.ranges.pop('left'); h.tick(True)
    assert h.health_wait_at is not None and not h.zones


def test_manual_source_aborts(rig):
    h,n,clock,step=rig
    step(); h._source(NS(data=3))
    assert not h.active and n._state=='paused_fault'


def test_operator_pause_cannot_resume_itself(rig):
    h,n,clock,step=rig
    step(); n._state='paused_operator'; step(.2)
    assert not h.active
    n._publish_resume.assert_not_called()


def test_sonar_record_expires_cliff_does_not(rig):
    h,n,clock,step=rig
    h.zones=[dict(x=1,y=1,radius=.04,expires=0,kind='cliff'),
             dict(x=2,y=2,radius=.06,expires=1001,kind='sonar')]
    h.publish_zones(1002)
    assert [z['kind'] for z in h.zones]==['cliff']


def test_source_gap_stops_then_requires_stable_recovery(rig):
    h,n,clock,step=rig
    step(); step(.7)
    assert h.phase=='reversing'
    step(.1,source_age=.9)
    assert h.active and h.phase=='reversing'
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    step(.1)
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    step(2.01)
    assert h.cmdpub.publish.call_args.args[0].linear.x==-.03
    assert h.source_wait_since is None


def test_source_gap_persistent_failure_never_moves(rig):
    h,n,clock,step=rig
    step(source_age=1.)
    step(1.,source_age=1.); step(2.1,source_age=1.)
    assert h.health_wait_at is not None
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


def test_source_wait_does_not_extend_reverse_limit(rig):
    h,n,clock,step=rig
    step(); step(.7)
    step(5.,progress=.05,source_age=.9)
    step(.5,progress=.05); step(2.1,progress=.05)
    step(1.1,progress=.15)
    assert h.phase=='failed'
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


def test_manual_takeover_aborts_during_source_wait(rig):
    h,n,clock,step=rig
    step(); step(.1,source_age=.9)
    h._source(NS(data=3))
    assert h.phase=='failed' and not h.active


def finish_reverse(h,clock,step):
    step(.5,(.155,.16,.3),.03)
    step(.6,(.155,.16,.3),.03)
    assert h.phase=='marking'
    h.applied={k:(h.marked_stamp,clock[0]) for k in ('local_costmap','global_costmap')}


def test_no_costmap_ack_never_resumes(rig):
    h,n,clock,step=rig
    step(); step(.7)
    finish_reverse(h,clock,step)
    h.applied={}
    step(2.1,(.155,.16,.3),.03)
    assert h.active and h.phase=='marking'
    step(3.,(.155,.16,.3),.03)
    assert h.phase=='marking' and h.active
    assert '重新发送' in n._reason
    n._publish_resume.assert_not_called()


def test_stale_or_single_layer_ack_rejected(rig):
    h,n,clock,step=rig
    step(); step(.7); finish_reverse(h,clock,step)
    h.applied['global_costmap']=(h.marked_stamp-1,clock[0])
    assert not h.zones_applied()
    h.applied['global_costmap']=(h.marked_stamp,clock[0]-1)
    assert not h.zones_applied()


def test_return_owner_preserved_and_no_explore_resume(rig):
    h,n,clock,step=rig
    n._state='returning_home'; n.home.phase='returning'; n.home.started=900.
    n.home.suspend_for_hazard.return_value=12
    n.home.hazard_resume_valid.return_value=True
    n.home.resume_after_hazard.return_value=True
    assert step() and h.owner=='home_return'
    step(.7); finish_reverse(h,clock,step); step(2.1,(.155,.16,.3),.03)
    n.home.resume_after_hazard.assert_called_once_with(12)
    n._publish_resume.assert_not_called()
    assert not h.active


def test_return_timeout_during_recovery_fails_without_resume(rig):
    h,n,clock,step=rig
    n._state='returning_home'; n.home.phase='returning'; n.home.started=900.
    n.home.suspend_for_hazard.return_value=12
    step()
    n.home.hazard_resume_valid.return_value=False
    step(.1)
    assert h.phase=='failed'
    n.home.resume_after_hazard.assert_not_called()


@pytest.mark.parametrize('state,phase', [('dock_motion','aligning'),('dock_motion','docking'),
                                     ('return_preparing','preparing'),('paused_operator','ready')])
def test_dock_and_inactive_states_never_reverse(rig,state,phase):
    h,n,clock,step=rig
    n._state=state; n.home.phase=phase
    assert not step()
    assert not h.active


def normal_navigation(h,n,clock):
    n._state='navigation_ready'; n.home.phase='navigation_ready'
    h.nav_owner.context={'session':'session1','token':'goal1','phase':'active'}
    h.nav_owner.seen=clock[0]


def owner_context(h,clock,phase):
    h.nav_owner.context=dict(h.owner_token,phase=phase)
    h.nav_owner.seen=clock[0]


def test_navigation_suspend_ack_before_cancel_then_resume_same_goal(rig):
    h,n,clock,step=rig
    normal_navigation(h,n,clock)
    step()
    assert h.phase=='acquiring_owner'
    n._cancel_client.call_async.assert_not_called()
    step(.1)
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    owner_context(h,clock,'suspended')
    step(.1)
    assert h.phase=='cancelling'
    n._cancel_client.call_async.assert_called_once()
    owner_context(h,clock,'suspended'); step(.7)
    owner_context(h,clock,'suspended'); step(.5,(.155,.16,.3),.03)
    owner_context(h,clock,'suspended'); step(.6,(.155,.16,.3),.03)
    assert h.phase=='marking'
    h.applied={k:(h.marked_stamp,clock[0]) for k in ('local_costmap','global_costmap')}
    clock[0]+=2.1; owner_context(h,clock,'suspended'); step(0,(.155,.16,.3),.03)
    assert h.phase=='resuming_owner' and n._state=='hazard_recovery'
    owner_context(h,clock,'active'); step(.1,(.155,.16,.3),.03)
    assert n._state=='navigation_ready' and not h.active
    n._publish_resume.assert_not_called()


@pytest.mark.parametrize('change', ['canceled','new_token','stale'])
def test_navigation_owner_lost_never_restarts(rig,change):
    h,n,clock,step=rig
    normal_navigation(h,n,clock); step()
    owner_context(h,clock,'suspended')
    if change=='new_token': h.nav_owner.context['token']='new_goal'
    if change=='canceled': h.nav_owner.context['phase']='canceled'
    if change=='stale': h.nav_owner.seen=clock[0]-2
    step(.1)
    assert h.phase=='failed' and not h.active
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


def test_no_automatic_sonar_recovery_without_fresh_acquired_echo(rig):
    h,n,clock,step=rig
    normal_navigation(h,n,clock)
    assert not step(values=(.155,.16,.19))
    assert not h.active


def test_navigation_resume_ack_timeout_fails_closed(rig):
    h,n,clock,step=rig
    normal_navigation(h,n,clock); step()
    h.phase='resuming_owner'; h.resume_at=clock[0]-4.1
    owner_context(h,clock,'resuming'); step(0,(.155,.16,.3))
    assert h.phase=='failed' and n._state=='paused_fault'


def test_fault_preserves_only_stationary_fresh_trace(rig):
    h,n,c,step=rig;step(values=(.15,.15,.4))
    h.trace.observe(c[0],n.home.dock.odom,True)
    h.fail('route unavailable')
    assert h.trace.points
    n.home.dock.odom=(.2,0.,0.)
    h.fail('moved unexpectedly')
    assert not h.trace.points


def test_operator_cancel_still_discards_trace(rig):
    h,n,c,step=rig;step(values=(.15,.15,.4))
    h.trace.observe(c[0],n.home.dock.odom,True)
    h.cancel('operator stop')
    assert not h.trace.points


def test_stationary_fault_pause_keeps_trace_until_operator_resume(rig):
    h,n,c,step=rig;step(values=(.15,.15,.4))
    h.trace.clear()
    h.trace.observe(c[0]-.2,(-.02,0.,0.),True)
    h.trace.observe(c[0],(0.,0.,0.),True)
    h.fail('no route')
    step(5.,values=(.15,.15,.4))
    assert len(h.trace.points)==2
    n._state='running'
    step(.1,values=(.15,.15,.4))
    assert len(h.trace.points)==2


def test_manual_takeover_while_fault_paused_invalidates_trace(rig):
    h,n,c,step=rig;step(values=(.15,.15,.4))
    h.trace.observe(c[0],(0.,0.,0.),True);h.fail('no route')
    h.source=3;h.tick(True)
    assert not h.trace.points

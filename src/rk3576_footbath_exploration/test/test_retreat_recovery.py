from types import SimpleNamespace as NS
import math
import importlib.util
from pathlib import Path
import pytest
from rk3576_footbath_exploration.retreat_trace import RetreatTrace, tof_retreat_allowed
_spec=importlib.util.spec_from_file_location('retreat_fixture',Path(__file__).with_name('test_hazard_recovery.py'))
f=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(f)
rig=f.rig


def trace(length=.6):
    t=RetreatTrace()
    for i in range(int(length/.02)+1):
        t.observe(990+i*.2,(-length+i*.02,0.,0.),True)
    return t


def test_trace_requires_traversed_aligned_bounded_path():
    t=trace()
    assert .55<t.plan(1000,(0,0,0))<=.61
    with pytest.raises(ValueError): t.plan(1000,(0,0,math.pi))
    with pytest.raises(ValueError): t.plan(1000,(0,.10,0))
    with pytest.raises(ValueError): t.plan(1300,(0,0,0))
    with pytest.raises(ValueError): RetreatTrace().plan(1000,(0,0,0))
    assert .08<=t.plan(1000,(0,0,0),.08,.10)<=.10


def test_trace_does_not_count_back_and_forth_as_long_escape():
    t=RetreatTrace()
    for i in range(40): t.observe(i*.2,(.06 if i%2 else 0.,0.,0.),True)
    with pytest.raises(ValueError): t.plan(8,(.06,0,0))


@pytest.mark.parametrize('state,age,old,sonar,allowed',[
    (14,0,0,.25,True),(12,0,0,.25,True),(15,0,0,.25,False),
    (6,0,0,.25,False),(14,.5,0,.25,False),(14,0,3,.25,False),
    (14,0,0,.5,False)])
def test_tof_frame_loss_is_not_near_range_failure(state,age,old,sonar,allowed):
    good={'left':(.155,10-old),'right':(.16,10-old)}
    assert tof_retreat_allowed(10,state,10-age,good,sonar)==allowed


def setup_retreat(rig):
    h,n,clock,step=rig
    h.trace=trace(); h.rear_observed=lambda:True; h.turning_room=lambda:True
    n.home.current_pose.side_effect=lambda:NS(pose=NS(position=NS(x=n.home.dock.odom[0],y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
    h.escape_requested=True
    return h,n,clock,step


def test_escape_cancels_retreats_then_continues_without_completing(rig):
    h,n,clock,step=setup_retreat(rig)
    assert step(values=(.155,.16,.5));assert h.phase=='cancelling'
    n._publish_resume.assert_not_called()
    step(.7,values=(.155,.16,.5)); assert h.phase=='retreating'
    assert h.cmdpub.publish.call_args.args[0].linear.x==-.03
    step(.5,values=(.155,.16,.5),progress=.60)
    step(.7,values=(.155,.16,.5),progress=.60)
    assert not h.active and n._state=='running'
    n._publish_resume.assert_called_once()
    assert not h.zones  # suppress frontier, never invent a physical obstacle


@pytest.mark.parametrize('failure',['rear','manual','cancel','stale_odom','no_room','no_progress'])
def test_escape_failures_never_resume(rig,failure):
    h,n,clock,step=setup_retreat(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    assert h.active and h.phase=='retreating'
    if failure=='rear':
        h.rear_observed=lambda:False;step(.1,values=(.155,.16,.5))
    elif failure=='manual': h._manual(NS())
    elif failure=='cancel': n._state='paused_operator';step(.1,values=(.155,.16,.5))
    elif failure=='stale_odom':
        n.home.dock.odom_at=0.;h.tick(True)
    elif failure=='no_room':
        h.turning_room=lambda:False
        step(.1,values=(.155,.16,.5),progress=.6);step(1.1,values=(.155,.16,.5),progress=.6)
        # Exploration hands back to collision-aware navigation; no forced turn.
        assert not h.active
        n._publish_resume.assert_called_once()
        assert all(c.args[0].angular.z==0 for c in h.cmdpub.publish.call_args_list)
        return
    else: step(2.1,values=(.155,.16,.5))
    if failure=='stale_odom':
        assert h.active and h.health_wait_at is not None
    else:
        assert not h.active
    n._publish_resume.assert_not_called()
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


def tof_tick(h,n,clock,dt=0.,valid=False,progress=0.):
    clock[0]+=dt;now=clock[0]
    h.tof_state=15 if valid else 14;h.tof_state_at=now
    h.ranges={'right':(.16,now),'sonar':(.30,now)}
    if valid:h.ranges['left']=(.155,now)
    h.source=1;h.source_at=now;h.fault_at=now
    n.home.dock.odom=(-progress,0.,0.);n.home.dock.odom_at=now
    return h.tick(True)


def test_tof_invalid_short_retreat_requires_stable_measurement_before_resume(rig):
    h,n,clock,step=setup_retreat(rig);h.escape_requested=False
    h.last_good_tof={'left':(.155,clock[0]),'right':(.16,clock[0])}
    tof_tick(h,n,clock);assert h.kind=='tof_invalid'
    tof_tick(h,n,clock,.7);assert h.phase=='retreating'
    tof_tick(h,n,clock,.5,True,.03);n._publish_resume.assert_not_called()
    tof_tick(h,n,clock,.7,True,.03)
    n._publish_resume.assert_called_once();assert not h.active and not h.zones


def test_tof_fresh_frames_but_failure_persists_stops_at_limit(rig):
    h,n,clock,step=setup_retreat(rig);h.escape_requested=False
    h.last_good_tof={'left':(.155,clock[0]),'right':(.16,clock[0])}
    tof_tick(h,n,clock);tof_tick(h,n,clock,.7)
    tof_tick(h,n,clock,.5,progress=h.retreat_distance)
    tof_tick(h,n,clock,1.1,progress=h.retreat_distance)
    assert h.phase=='failed';n._publish_resume.assert_not_called()


def test_tof_disconnect_never_auto_reverses(rig):
    h,n,clock,step=setup_retreat(rig);h.escape_requested=False
    h.last_good_tof={'left':(.155,clock[0]),'right':(.16,clock[0])}
    h.ranges={'right':(.16,clock[0]),'sonar':(.25,clock[0])}
    h.tof_state=6;h.tof_state_at=clock[0]
    h.tick(True)
    assert h.health_wait_at is not None
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


def test_navigation_oscillation_retreat_resumes_owned_goal(rig):
    h,n,clock,step=setup_retreat(rig);h.escape_requested=False
    f.normal_navigation(h,n,clock)
    key=('navigation','session1','goal1')
    for i in range(46): h.motion_window.update(key,clock[0]-45+i,(.08 if i%2 else 0.,0,0))
    step(values=(.155,.16,.5));assert h.phase=='acquiring_owner'
    f.owner_context(h,clock,'suspended');step(.1,values=(.155,.16,.5))
    f.owner_context(h,clock,'suspended');step(.7,values=(.155,.16,.5))
    assert h.phase=='retreating'
    f.owner_context(h,clock,'suspended');step(.5,values=(.155,.16,.5),progress=.6)
    f.owner_context(h,clock,'suspended');step(.7,values=(.155,.16,.5),progress=.6)
    assert h.phase=='resuming_owner'
    f.owner_context(h,clock,'active');step(.1,values=(.155,.16,.5),progress=.6)
    assert n._state=='navigation_ready' and not h.active
    n._publish_resume.assert_not_called()


def test_motion_window_resets_for_new_goal_and_allows_slow_progress():
    from rk3576_footbath_exploration.retreat_trace import MotionWindow
    w=MotionWindow()
    for i in range(100): assert not w.update('a',i,(i*.01,0,0))
    assert not w.update('b',101,(0,0,0))
    for i in range(102,146): assert not w.update('b',i,(0,0,0))
    assert w.update('b',146,(0,0,0))


def test_trace_drops_route_when_ground_or_control_is_untrusted():
    t=trace();t.observe(998,(0,0,0),False)
    with pytest.raises(ValueError):t.plan(1000,(0,0,0))


def test_rear_blind_sector_and_old_scan_forbid_retreat(rig):
    from rk3576_footbath_exploration.retreat_recovery import RetreatRecoveryMixin
    from builtin_interfaces.msg import Time
    h,n,clock,step=rig
    h.rear_observed=RetreatRecoveryMixin.rear_observed.__get__(h)
    n.get_clock.return_value.now.return_value.nanoseconds=int(clock[0]*1e9)
    n.home.buffer.lookup_transform.return_value=NS(transform=NS(
        translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    scan=NS(header=NS(stamp=Time(sec=1000),frame_id='laser'),angle_min=0.,angle_increment=math.pi/180,
            range_min=.1,range_max=6.,ranges=[2.]*360)
    n.home.dock.scan=scan;n.home.dock.scan_at=clock[0]
    assert h.rear_observed()
    scan.ranges[120:150]=[math.nan]*30
    assert not h.rear_observed()
    scan.ranges=[2.]*360;n.home.dock.scan_at=clock[0]-.5
    assert not h.rear_observed()


def test_tof_recovery_never_treats_old_distance_as_recovered(rig):
    h,n,clock,step=setup_retreat(rig);h.escape_requested=False
    h.last_good_tof={'left':(.155,clock[0]),'right':(.16,clock[0])}
    tof_tick(h,n,clock);tof_tick(h,n,clock,.7)
    now=clock[0]
    h.tof_state=15;h.tof_state_at=now
    h.ranges={'left':(.155,now-1),'right':(.16,now),'sonar':(.3,now)}
    values=h.retreat_values(now,'exploration')
    assert math.isnan(values[0])


def test_escape_never_moves_before_navigation_cancel_ack(rig):
    h,n,clock,step=setup_retreat(rig)
    n._cancel_client.call_async.return_value.done.return_value=False
    step(values=(.155,.16,.5))
    step(.7,values=(.155,.16,.5))
    assert h.active and h.phase=='cancelling'
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)
    step(4.1,values=(.155,.16,.5))
    assert h.phase=='failed';n._publish_resume.assert_not_called()


def test_attempt_count_starts_at_reverse_command_not_plan(rig):
    h,n,clock,step=setup_retreat(rig)
    step(values=(.155,.16,.5))
    assert h.phase=='cancelling' and h.retreat_episode.attempts==0
    step(.7,values=(.155,.16,.5))
    assert h.phase=='retreating' and h.retreat_episode.attempts==1
    step(.1,values=(.155,.16,.5),progress=.01)
    assert h.retreat_episode.attempts==1


def test_same_episode_exhaustion_reports_budget_not_obstacle(rig):
    h,n,clock,step=setup_retreat(rig)
    h.retreat_episode.begin((0.,0.,0.))
    h.retreat_episode.attempts=2;h.retreat_episode.fallbacks=1
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    assert h.phase=='waiting_start_space'
    h.start_space=False
    step(3.,values=(.155,.16,.5))
    assert h.active and h.phase=='waiting_start_space'
    n._publish_resume.assert_not_called()
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


@pytest.mark.parametrize('owner',['exploration','navigation','home_return'])
@pytest.mark.parametrize('front',[.28,.30,.349,.35])
def test_all_escape_owners_release_below_preferred_margin(rig,owner,front):
    h,n,clock,step=setup_retreat(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    h.owner=owner;h.owner_token={'session':'s','token':'t','recovery_id':'r'}
    n.home.dock.odom=(-.6,0.,0.)
    h.retreat_tick((.155,.16,front),clock[0],.6)
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    clock[0]+=.7
    h.retreat_tick((.155,.16,front),clock[0],.6)
    if owner=='exploration': n._publish_resume.assert_called_once()
    elif owner=='home_return': n.home.resume_after_hazard.assert_called_once_with({'session':'s','token':'t','recovery_id':'r'})
    else: assert h.phase=='resuming_owner'


@pytest.mark.parametrize('front',[.20,.25,.2799])
def test_escape_cannot_bypass_release_by_selecting_new_frontier(rig,front):
    h,n,clock,step=setup_retreat(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    n.home.dock.odom=(-.6,0.,0.)
    h.retreat_tick((.155,.16,front),clock[0],.6)
    assert h.phase=='confirming' and h.kind=='sonar'
    n._publish_resume.assert_not_called()


def test_28cm_does_not_override_side_protection(rig):
    h,n,clock,step=setup_retreat(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    h.sides_clear=lambda now:False
    h.select_other_frontier((.155,.16,.28),clock[0])
    assert h.phase=='waiting_start_space'
    n._publish_resume.assert_not_called()

def test_tof_failure_during_escape_switches_to_bounded_recovery(rig):
    h,n,clock,step=setup_retreat(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    assert h.kind=='escape' and h.phase=='retreating'
    h.corridor_active=True  # Also revoke any previous curved-exit capability.
    h.last_good_tof={'left':(.027,clock[0]),'right':(.16,clock[0])}
    tof_tick(h,n,clock)
    assert h.kind=='tof_invalid' and h.phase=='cancelling'
    assert not h.corridor_active
    assert h.escape_motion_pub.publish.call_args.args[0].data==0
    assert h.health_wait_at is None
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    tof_tick(h,n,clock,.7)
    assert h.phase=='retreating' and h.retreat_distance<=.10
    tof_tick(h,n,clock,.5,True,.03)
    tof_tick(h,n,clock,.7,True,.03)
    assert not h.active and n._state=='running'

def test_valid_27mm_does_not_enter_invalid_wait(rig):
    h,n,clock,step=setup_retreat(rig);h.escape_requested=False
    step(values=(.027,.16,.5))
    assert h.health_wait_at is None and not h.active


def test_blocked_departure_waits_without_blacklisting_or_ending_mapping(rig):
    h,n,c,step=setup_retreat(rig)
    h.retreat_episode.begin((0.,0.,0.));h.retreat_episode.attempts=2
    h.start_space=False
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    for _ in range(20):step(1.,values=(.155,.16,.5))
    assert h.active and h.phase=='waiting_start_space' and n._state=='hazard_recovery'
    assert h.retreat_episode.attempts==2
    n._publish_resume.assert_not_called();n._request_map_save.assert_not_called()
    assert all(call.args[0].linear.x==0 and call.args[0].angular.z==0
               for call in h.cmdpub.publish.call_args_list)
    h.start_space=True
    step(.1,values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    assert not h.active and n._state=='running'
    n._publish_resume.assert_called_once()


def test_waiting_departure_gives_near_confirmation_priority(rig):
    h,n,c,step=setup_retreat(rig)
    h.retreat_episode.begin((0.,0.,0.));h.retreat_episode.attempts=2
    h.start_space=False
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    step(.2,values=(.155,.16,.25))
    assert h.active and h.phase=='confirming' and h.kind=='sonar'
    assert 'front' in h.trigger_sources
    n._publish_resume.assert_not_called()
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


@pytest.mark.parametrize('block',['stale','fault','motion','manual','cancel','health'])
def test_departure_resume_still_obeys_all_gates(rig,block):
    h,n,c,step=setup_retreat(rig)
    h.retreat_episode.begin((0.,0.,0.));h.retreat_episode.attempts=2
    h.start_space=False
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    h.start_space=True
    if block=='stale':
        h.start_space_at=c[0]-2
        h.tick(True)
    elif block=='fault':
        h.fault=1;step(.1,values=(.155,.16,.5))
    elif block=='motion':
        n.home.last_motion=c[0]+10;step(.1,values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    elif block=='manual':h._manual(NS())
    elif block=='cancel':n._state='paused_operator';step(.1,values=(.155,.16,.5))
    else:h.tick(False)
    n._publish_resume.assert_not_called()
    assert h.cmdpub.publish.call_args.args[0].linear.x==0

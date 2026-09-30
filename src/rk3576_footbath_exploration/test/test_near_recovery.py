from types import SimpleNamespace as NS
from unittest.mock import Mock
import math
import pytest
from builtin_interfaces.msg import Time
import importlib.util
from pathlib import Path
_fixture_spec = importlib.util.spec_from_file_location(
    'near_hazard_fixture', Path(__file__).with_name('test_hazard_recovery.py'))
_fixture = importlib.util.module_from_spec(_fixture_spec)
_fixture_spec.loader.exec_module(_fixture)
rig = _fixture.rig
normal_navigation = _fixture.normal_navigation


def echo(h, clock, key='side_left', distance=.12, age=0.):
    stamp = clock[0]-age
    msg = NS(range=distance, header=NS(frame_id=key+'_link',
        stamp=Time(sec=int(stamp), nanosec=int((stamp-int(stamp))*1e9))))
    h.sonar_samples[key] = (msg, stamp, stamp)
    return msg


def prepare(rig):
    h,n,clock,step=rig
    h.side_enabled=True
    h.overlay.gate.since=998.
    h.overlay.tf_stable_at=998.
    h.overlay.stable=lambda now: True
    echo(h,clock,'side_right',math.nan)
    echo(h,clock)
    n.home.buffer.lookup_transform.return_value=NS(transform=NS(
        translation=NS(x=1.,y=2.), rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    return h,n,clock,step


def test_above_12cm_does_not_interrupt_navigation(rig):
    h,n,clock,step=prepare(rig)
    echo(h,clock,distance=.12001)
    assert not step(values=(.155,.16,.5))
    assert not h.active and not h.zones


def test_disabled_sides_never_trigger_or_require_readings(rig):
    h,n,clock,step=prepare(rig)
    h.side_enabled=False
    assert not step(values=(.155,.16,.5))
    h.sonar_samples={}
    assert h.sides_clear(clock[0])


def test_stop_confirm_new_echoes_then_reverse_and_remember(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5)); assert h.phase=='cancelling'
    step(.7,values=(.155,.16,.5)); assert h.phase=='confirming'
    assert not h.pending and not h.zones
    # The echo from before cancellation must never become a map point.
    step(.1,values=(.155,.16,.5)); assert h.phase=='confirming'
    for _ in range(3):
        clock[0]+=.32
        echo(h,clock); echo(h,clock,'side_right',math.nan)
        step(values=(.155,.16,.5))
    assert h.phase=='reversing' and h.cmdpub.publish.call_args.args[0].linear.x == -.03
    assert h.pending[0]['x'] == pytest.approx(1.12)
    used_time = n.home.buffer.lookup_transform.call_args.args[2]
    assert used_time.nanoseconds == echo(h,clock).header.stamp.sec*10**9 + echo(h,clock).header.stamp.nanosec
    # Above threshold permits release, but the fixed point remains in memory.
    clock[0]+=.2; echo(h,clock,distance=.13); echo(h,clock,'side_right',math.nan)
    step(values=(.155,.16,.5),progress=.03)
    clock[0]+=.51; echo(h,clock,distance=.13); echo(h,clock,'side_right',math.nan)
    step(values=(.155,.16,.5),progress=.03)
    assert h.phase=='marking' and h.zones[0]['kind']=='near'
    assert h.zones[0]['expires']==0.


def test_unconfirmed_echo_or_missing_exact_tf_never_reverses(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    n.home.buffer.lookup_transform.side_effect=RuntimeError('no acquisition TF')
    for _ in range(3):
        clock[0]+=.32;echo(h,clock);step(values=(.155,.16,.5))
    assert h.phase=='confirming' and not h.pending
    step(2.1,values=(.155,.16,.5))
    assert h.phase=='confirming' and h.confirm_attempt==2
    step(3.1,values=(.155,.16,.5))
    assert h.phase=='confirming' and h.confirm_attempt==3
    step(3.1,values=(.155,.16,.5))
    assert h.phase=='confirming' and h.confirm_attempt==4 and not h.zones
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


def test_sonar_now_uses_normal_navigation_owner(rig):
    h,n,clock,step=prepare(rig)
    normal_navigation(h,n,clock)
    step(values=(.155,.16,.5))
    assert h.owner=='navigation' and h.phase=='acquiring_owner'
    n._cancel_client.call_async.assert_not_called()


def test_stale_side_reading_cannot_authorize_reverse(rig):
    h,n,clock,step=prepare(rig)
    echo(h,clock,age=.7)
    with pytest.raises(ValueError): h.sides_clear(clock[0])


def test_moving_echo_cannot_be_confirmed_at_later_heading(rig):
    h,n,clock,step=prepare(rig)
    h.trigger_sources=['side_left'];h.confirm_at=clock[0]-.8
    for _ in range(3):
        clock[0]+=.32;echo(h,clock)
        n.home.last_motion=clock[0]-.1
        assert not h.confirm_near(clock[0])
    n.home.buffer.lookup_transform.assert_not_called()


def test_release_above_threshold_still_requires_space_from_remembered_point(rig):
    h,n,clock,step=prepare(rig)
    h.kind='sonar'; h.pending=[dict(x=.24,y=0.,radius=.04)]
    echo(h,clock,distance=.13)
    assert not h.clear_for_resume((.155,.16,.5),clock[0])
    n.home.current_pose.return_value.pose.position.x=-.10
    assert h.clear_for_resume((.155,.16,.5),clock[0])


def test_callback_rejects_old_echo_and_does_not_refresh_duplicate(rig):
    h,n,clock,step=prepare(rig)
    h.sonar_samples={}
    n.get_clock.return_value.now.return_value.nanoseconds=int(clock[0]*1e9)
    msg=echo(h,clock,age=.7)
    h._sonar_range('side_left',msg)
    assert 'side_left' not in h.sonar_samples
    msg=echo(h,clock);h.sonar_samples={}
    h._sonar_range('side_left',msg)
    first=h.sonar_samples['side_left']
    clock[0]+=.2
    n.get_clock.return_value.now.return_value.nanoseconds=int(clock[0]*1e9)
    h._sonar_range('side_left',msg)
    assert h.sonar_samples['side_left']==first


def test_callback_no_echo_is_fresh_but_not_an_obstacle(rig):
    h,n,clock,step=prepare(rig)
    n.get_clock.return_value.now.return_value.nanoseconds=int(clock[0]*1e9)
    msg=echo(h,clock,distance=math.nan);h.sonar_samples={}
    h._sonar_range('side_left',msg)
    assert 'side_left' in h.sonar_samples
    assert not h.near_sources(clock[0])


def test_finite_cleared_echoes_resume_without_retreat_or_new_zone(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    for _ in range(3):
        clock[0]+=.32;echo(h,clock,distance=.6);echo(h,clock,'side_right',math.nan)
        step(values=(.155,.16,.5))
    assert not h.active and n._state=='running' and not h.zones
    n._publish_resume.assert_called_once()
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


def test_no_echo_after_stop_does_not_claim_obstacle_disappeared(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    for _ in range(3):
        clock[0]+=.32;echo(h,clock,distance=math.nan);step(values=(.155,.16,.5))
    assert h.phase=='confirming' and '无回波' in h.confirm_detail
    assert not h.zones
    n._publish_resume.assert_not_called()


def test_one_probe_clear_other_confirmed_still_records_before_retreat(rig):
    h,n,clock,step=prepare(rig);echo(h,clock,'side_right',.10)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    for _ in range(3):
        clock[0]+=.32;echo(h,clock,distance=.18);echo(h,clock,'side_right',.10)
        step(values=(.155,.16,.5))
    assert h.phase=='reversing' and len(h.zones)==2
    assert h.zones[0]['kind']=='near' and h.zones[0]['encounters']==1


def test_different_near_obstacle_not_blocked_by_global_ten_second_cooldown(rig):
    h,n,clock,step=prepare(rig);h.last_success=clock[0]-2.
    step(values=(.155,.16,.5))
    assert h.phase=='cancelling' and h.active


def test_single_far_echo_keeps_near_evidence_and_starts_reverse(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    for distance in (.115,.38,.114,.116):
        clock[0]+=.32
        echo(h,clock,distance=distance);echo(h,clock,'side_right',math.nan)
        step(values=(.155,.16,.5))
    assert h.phase=='reversing' and h.near_recorded
    assert len(h.pending)==1 and h.pending[0]['source']=='side_left'
    assert h.cmdpub.publish.call_args.args[0].linear.x==-.03


def test_retry_can_confirm_and_continue_without_operator(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    step(3.1,values=(.155,.16,.5))
    assert h.confirm_attempt==2 and h.phase=='confirming'
    for distance in (.115,.38,.114,.116):
        clock[0]+=.32
        echo(h,clock,distance=distance);echo(h,clock,'side_right',math.nan)
        step(values=(.155,.16,.5))
    assert h.phase=='reversing' and h.near_recorded

def test_stationary_sides_beyond_stop_range_record_independently(rig):
    h,n,clock,step=prepare(rig)
    h.trigger_sources=['front'];h.confirm_at=clock[0];h.stationary_recorded={}
    for _ in range(3):
        clock[0]+=.32
        echo(h,clock,'front',math.nan)
        echo(h,clock,'side_left',.30)
        echo(h,clock,'side_right',.48)
        assert not h.confirm_near(clock[0])
    assert {z['source'] for z in h.zones}=={'side_left','side_right'}
    before=len(h.zones)
    h.confirm_near(clock[0])
    assert len(h.zones)==before
    assert all(z.get('encounters')==1 for z in h.zones)

def test_confirmation_recovers_after_more_than_three_rounds(rig):
    h,n,clock,step=prepare(rig)
    step(values=(.155,.16,.5));step(.7,values=(.155,.16,.5))
    for _ in range(5):step(3.1,values=(.155,.16,.5))
    assert h.phase=='confirming' and h.confirm_attempt==6
    for _ in range(3):
        clock[0]+=.32;echo(h,clock);echo(h,clock,'side_right',math.nan)
        step(values=(.155,.16,.5))
    assert h.phase=='reversing' and h.zones


@pytest.mark.parametrize('distance',[.20001,.265,.27999])
def test_stable_front_hysteresis_echoes_confirm_then_reverse(rig,distance):
    h,n,clock,step=prepare(rig);h.side_enabled=False
    echo(h,clock,'front',.19)
    step(values=(.155,.16,.19));step(.7,values=(.155,.16,.19))
    assert h.phase=='confirming'
    # Real confirmation implementation with three distinct post-stop messages.
    for i in range(3):
        clock[0]+=.32;echo(h,clock,'front',distance)
        step(values=(.155,.16,distance))
        if i<2:assert h.phase=='confirming'
    assert h.phase=='reversing' and h.near_recorded
    assert h.pending[0]['source']=='front'
    assert h.pending[0]['x']==pytest.approx(1.+distance)
    assert h.cmdpub.publish.call_args.args[0].linear.x==-.03


def test_26cm_during_normal_travel_does_not_trigger_recovery(rig):
    h,n,clock,step=prepare(rig);h.side_enabled=False
    echo(h,clock,'front',.265)
    assert not step(values=(.155,.16,.265))
    assert not h.active and not h.near_sources(clock[0])


@pytest.mark.parametrize('condition',['moving','missing_tf','no_echo','duplicate'])
def test_hysteresis_confirmation_keeps_stationary_fresh_tf_gates(rig,condition):
    h,n,clock,step=prepare(rig);h.side_enabled=False
    echo(h,clock,'front',.19)
    step(values=(.155,.16,.19));step(.7,values=(.155,.16,.19))
    if condition=='missing_tf':n.home.buffer.lookup_transform.side_effect=RuntimeError('no TF')
    for i in range(4):
        clock[0]+=.1
        if condition=='moving':n.home.last_motion=clock[0]
        if condition!='duplicate' or i==0:echo(h,clock,'front',math.nan if condition=='no_echo' else .265)
        step(values=(.155,.16,.265))
    assert h.phase=='confirming'
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)


def test_corridor_handoff_real_26cm_echoes_reach_safe_reverse_check(rig):
    h,n,clock,step=prepare(rig);h.side_enabled=False
    echo(h,clock,'front',.19)
    step(values=(.155,.16,.19));step(.7,values=(.155,.16,.19))
    # Same handoff state used after the corridor stops. Only geometry is mocked;
    # the sensor windows, map recording and state machine are all real.
    h.corridor_near_handoff=True;h.corridor_near_retry_at=None
    h.corridor_near_trace=Mock();h.corridor_near_trace.plan.return_value=.05
    h.corridor_near_ground=[];h.rear_sweep_clear=Mock()
    for d in (.265,.2652,.2651):
        clock[0]+=.32;echo(h,clock,'front',d);step(values=(.155,.16,d))
    assert h.phase=='reversing' and h.sonar_reverse_limit==.05
    h.corridor_near_trace.plan.assert_called_once()
    h.rear_sweep_clear.assert_called_once_with(.05)


def test_hysteresis_confirmed_reverse_releases_at_28cm(rig):
    h,n,clock,step=prepare(rig);h.side_enabled=False
    echo(h,clock,'front',.19)
    step(values=(.155,.16,.19));step(.7,values=(.155,.16,.19))
    for d in (.265,.2652,.2651):
        clock[0]+=.32;echo(h,clock,'front',d);step(values=(.155,.16,d))
    clock[0]+=.2;echo(h,clock,'front',.28);step(values=(.155,.16,.28),progress=.03)
    clock[0]+=.51;echo(h,clock,'front',.28);step(values=(.155,.16,.28),progress=.03)
    assert h.phase=='marking'
    h.applied={k:(h.marked_stamp,clock[0]) for k in ('local_costmap','global_costmap')}
    clock[0]+=2.1;echo(h,clock,'front',.28);step(values=(.155,.16,.28),progress=.03)
    assert not h.active and n._state=='running'

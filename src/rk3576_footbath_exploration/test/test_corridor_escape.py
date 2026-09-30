import math
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from rk3576_footbath_exploration.corridor_route import FrozenRoute, swept_centers
from test_hazard_recovery import rig


def records():
    return [(900+i*.1,(math.sin(i*.02),1-math.cos(i*.02),i*.02),False) for i in range(41)]


def test_curved_reverse_tracks_without_cutting_across_corner():
    route=FrozenRoute(records(),910.,records()[-1][1]);pose=list(records()[-1][1])
    previous=0.
    for _ in range(2000):
        v,w,done=route.command(pose,-1)
        assert -.03<=v<=0 and abs(w)<=.12
        assert route.progress>=previous;previous=route.progress
        if done:break
        pose[0]+=v*math.cos(pose[2])*.05;pose[1]+=v*math.sin(pose[2])*.05;pose[2]+=w*.05
    else:pytest.fail('reverse never finished')
    assert math.hypot(pose[0],pose[1])<.025


def test_turn_then_forward_tracks_same_frozen_route():
    route=FrozenRoute(records(),910.,records()[-1][1]);pose=list(records()[-1][1]);pose[2]+=math.pi
    for _ in range(2000):
        v,w,done=route.command(pose,1)
        assert 0<=v<=.03 and abs(w)<=.12
        if done:break
        pose[0]+=v*math.cos(pose[2])*.05;pose[1]+=v*math.sin(pose[2])*.05;pose[2]+=w*.05
    else:pytest.fail('forward never finished')
    assert math.hypot(pose[0],pose[1])<.025


@pytest.mark.parametrize('points,now', [([],910.),(records(),1200.),(records()[:2],910.)])
def test_no_fresh_continuous_ground_never_invents_route(points,now):
    with pytest.raises(ValueError):FrozenRoute(points,now,records()[-1][1])


def test_heading_and_cross_track_limits():
    route=FrozenRoute(records(),910.,records()[-1][1]);x,y,a=records()[-1][1]
    with pytest.raises(ValueError):route.command((x,y,a+math.pi/2),-1)
    with pytest.raises(ValueError):route.command((x+.2,y,a),-1)


def test_sweep_includes_curved_motion_not_just_centerline():
    centers=swept_centers(-.03,.12)
    assert centers[0]==(0.,0.) and centers[-1][0]<0 and centers[-1][1]<0


def prepare(rig):
    h,n,c,step=rig
    h.kind='escape';h.owner='exploration';h.retreat_counted=False
    h.trace.points.extend(records());c[0]=910.
    h.start_space=True;h.start_space_at=c[0]
    n.home.dock.odom=records()[-1][1];n.home.dock.radius=.22
    h.clear_for_resume=Mock(return_value=True);h.corridor_clear=Mock();h.fault=0
    return h,n,c


def test_turn_preferred_when_full_sweep_is_clear(rig):
    h,n,c=prepare(rig)
    assert h.begin_corridor((.15,.15,.4),c[0])
    assert h.phase=='escape_turn'
    h.corridor_clear.assert_called_once_with(c[0],[(0.,0.)],turn=True)
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


def test_curve_selected_only_when_its_own_sweep_is_clear(rig):
    h,n,c=prepare(rig)
    def check(now,centers,turn=False):
        if turn:raise ValueError('no turning margin')
    h.corridor_clear.side_effect=check
    assert h.begin_corridor((.15,.15,.4),c[0]) and h.phase=='escape_curve'
    h.corridor_clear.side_effect=ValueError('occupied')
    assert not h.begin_corridor((.15,.15,.4),c[0])


def test_protection_never_authorizes_steering(rig):
    h,n,c=prepare(rig);h.clear_for_resume.return_value=False
    assert not h.begin_corridor((.15,.15,.2),c[0])
    h.corridor_clear.assert_not_called()


def test_operator_cancel_revokes_escape_capability(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0]);h.cancel('operator')
    assert not h.corridor_active
    assert h.escape_motion_pub.publish.call_args.args[0].data==0


def spatial_rig(rig):
    from builtin_interfaces.msg import Time
    h,n,c,step=rig
    n.home.dock.radius=.22
    n.get_clock.return_value.now.return_value.nanoseconds=int(c[0]*1e9)
    n.home.buffer.lookup_transform.return_value=NS(transform=NS(
        translation=NS(x=0.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    scan=NS(header=NS(stamp=Time(sec=1000),frame_id='laser'),angle_min=0.,angle_increment=math.pi/180,
        range_min=.02,range_max=8.,ranges=[2.]*360)
    n.home.dock.scan=scan;n.home.dock.scan_at=c[0]
    import copy
    n.home.dock.low_scan=copy.deepcopy(scan);n.home.dock.low_at=c[0]
    h.overlay.grid=NS(info=NS(resolution=.05,width=200,height=200,
        origin=NS(position=NS(x=-5.,y=-5.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*40000)
    for delta in (-.30,-.15,0.):
        stamp=c[0]+delta
        n.home.dock.scan.header.stamp=Time(sec=int(stamp),nanosec=round((stamp%1)*1e9))
        n.get_clock.return_value.now.return_value.nanoseconds=round(stamp*1e9)
        n.home.dock.scan_at=stamp
        h.collect_corridor_scan(stamp)
    return h,n,c


def test_full_turn_checks_rear_and_low_obstacles(rig):
    h,n,c=spatial_rig(rig)
    h.corridor_clear(c[0],[(0.,0.)],turn=True)
    n.home.dock.low_scan.ranges[180]=.29
    with pytest.raises(ValueError,match='雷达障碍'):h.corridor_clear(c[0],[(0.,0.)],turn=True)
    # Small curve margin may fit where a full turn margin does not.
    h.corridor_clear(c[0],[(0.,0.)])


@pytest.mark.parametrize('change',['unknown','glass','blind','stale'])
def test_unknown_glass_and_sensor_loss_never_authorize_motion(rig,change):
    h,n,c=spatial_rig(rig)
    if change=='unknown':h.overlay.grid.data[100*200+100]=-1
    elif change=='glass':h.zones=[dict(x=.2,y=0.,radius=.04)]
    elif change=='blind':
        n.home.dock.scan.ranges[120:135]=[math.inf]*15
        h.corridor_scans=[]
    else:n.home.dock.low_at-=.5
    with pytest.raises(ValueError):h.corridor_clear(c[0],[(0.,0.)],turn=True)


def test_curve_sweep_checks_future_body_position(rig):
    h,n,c=spatial_rig(rig)
    n.home.dock.low_scan.ranges[180]=.28
    h.corridor_clear(c[0],[(0.,0.)])
    with pytest.raises(ValueError):h.corridor_clear(c[0],swept_centers(-.03,0.))


def test_corridor_tick_cancels_on_new_protection(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.clear_for_resume.return_value=False;h.resume_detail='side near'
    h.corridor_tick((.15,.15,.263),c[0])
    assert h.phase=='confirming' and h.kind=='sonar' and not h.corridor_active
    assert 'front' in h.trigger_sources and h.corridor_near_handoff
    assert all(call.args[0].angular.z==0 for call in h.cmdpub.publish.call_args_list)


@pytest.mark.parametrize('can_turn',[True,False])
def test_complete_escape_state_machine_releases_original_mapping(rig,can_turn):
    h,n,c=prepare(rig)
    n.home.current_pose.side_effect=lambda:NS(pose=NS(position=NS(x=n.home.dock.odom[0],y=n.home.dock.odom[1]),
        orientation=NS(x=0.,y=0.,z=math.sin(n.home.dock.odom[2]/2),w=math.cos(n.home.dock.odom[2]/2))))
    def clearance(now,centers,turn=False):
        if turn and not can_turn:raise ValueError('narrow')
    h.corridor_clear.side_effect=clearance
    assert h.begin_corridor((.15,.15,.4),c[0])
    phases=set()
    for _ in range(4000):
        phases.add(h.phase)
        c[0]+=.05
        h.start_space_at=c[0]
        h.corridor_tick((.15,.15,.4),c[0])
        if not h.corridor_active:break
        cmd=h.cmdpub.publish.call_args.args[0]
        x,y,a=n.home.dock.odom
        n.home.dock.odom=(x+cmd.linear.x*math.cos(a)*.05,y+cmd.linear.x*math.sin(a)*.05,a+cmd.angular.z*.05)
    else:pytest.fail('state machine did not complete')
    assert n._state=='running' and h.retreat_episode.attempts==1
    n._publish_resume.assert_called_once()
    assert ('escape_turn' in phases)==can_turn
    assert ('escape_forward' in phases)==can_turn
    assert ('escape_curve' in phases)==(not can_turn)


def test_scattered_missing_returns_pass_multiframe_but_not_single_frame(rig):
    from builtin_interfaces.msg import Time
    h,n,c=spatial_rig(rig);h.corridor_scans=[]
    for i in range(360):
        if i%3==0:n.home.dock.scan.ranges[i]=math.inf
    for dt in (.1,.25,.4):
        stamp=c[0]+dt
        n.home.dock.scan.header.stamp=Time(sec=int(stamp),nanosec=round((stamp%1)*1e9))
        n.home.dock.scan_at=stamp;n.home.dock.low_at=stamp
        n.home.dock.low_scan.header.stamp=n.home.dock.scan.header.stamp
        n.get_clock.return_value.now.return_value.nanoseconds=round(stamp*1e9)
        if dt<.4:
            with pytest.raises(ValueError,match='至少3帧'):h.corridor_clear(stamp,[(0.,0.)],turn=True)
        else:h.corridor_clear(stamp,[(0.,0.)],turn=True)


def test_small_alignment_requires_clear_body_and_feasible_curve(rig):
    h,n,c=prepare(rig)
    x,y,a=n.home.dock.odom;n.home.dock.odom=(x,y,a+.30)
    def check(now,centers,turn=False):
        if turn:raise ValueError('full turn margin unavailable')
    h.corridor_clear.side_effect=check
    assert h.begin_corridor((.15,.15,.4),c[0])
    assert h.phase=='escape_align' and abs(h.corridor_alignment)<=math.pi/6


def test_alignment_rejected_if_body_sweep_blocked(rig):
    h,n,c=prepare(rig)
    x,y,a=n.home.dock.odom;n.home.dock.odom=(x,y,a+.30)
    h.corridor_clear.side_effect=ValueError('occupied')
    assert not h.begin_corridor((.15,.15,.4),c[0])
    assert not h.corridor_active


def test_alignment_then_reverse_completes(rig):
    h,n,c=prepare(rig)
    x,y,a=n.home.dock.odom;n.home.dock.odom=(x,y,a+.30)
    n.home.current_pose.side_effect=lambda:NS(pose=NS(position=NS(x=n.home.dock.odom[0],y=n.home.dock.odom[1]),
        orientation=NS(x=0.,y=0.,z=math.sin(n.home.dock.odom[2]/2),w=math.cos(n.home.dock.odom[2]/2))))
    def check(now,centers,turn=False):
        if turn:raise ValueError('turn margin blocked')
    h.corridor_clear.side_effect=check
    assert h.begin_corridor((.15,.15,.4),c[0]) and h.phase=='escape_align'
    for _ in range(4000):
        c[0]+=.05;h.start_space_at=c[0];h.corridor_tick((.15,.15,.4),c[0])
        if not h.corridor_active:break
        cmd=h.cmdpub.publish.call_args.args[0];x,y,a=n.home.dock.odom
        n.home.dock.odom=(x+cmd.linear.x*math.cos(a)*.05,y+cmd.linear.x*math.sin(a)*.05,a+cmd.angular.z*.05)
    else:pytest.fail('alignment/reverse not completed')
    n._publish_resume.assert_called_once()

from rk3576_footbath_exploration.corridor_escape import required_sectors, RadarEvidencePending


def missing_sector(h, n, c, sector, value=math.inf):
    from builtin_interfaces.msg import Time
    h.corridor_scans=[]
    n.home.dock.scan.ranges[sector*15:(sector+1)*15]=[value]*15
    for dt in (.1,.25,.4):
        stamp=c[0]+dt
        msgstamp=Time(sec=int(stamp),nanosec=round((stamp%1)*1e9))
        n.home.dock.scan.header.stamp=msgstamp
        n.home.dock.low_scan.header.stamp=msgstamp
        n.home.dock.scan_at=stamp;n.home.dock.low_at=stamp
        n.get_clock.return_value.now.return_value.nanoseconds=round(stamp*1e9)
        h.collect_corridor_scan(stamp)
    c[0]+=.4


def test_reverse_ignores_front_missing_echo_but_turn_does_not(rig):
    h,n,c=spatial_rig(rig);missing_sector(h,n,c,0)
    h.corridor_clear(c[0],swept_centers(-.03,.06))
    with pytest.raises(RadarEvidencePending):h.corridor_clear(c[0],[(0.,0.)],turn=True)
    with pytest.raises(RadarEvidencePending):h.corridor_clear(c[0],swept_centers(.03,0.))


def test_reverse_still_requires_rear_and_side_coverage(rig):
    h,n,c=spatial_rig(rig);missing_sector(h,n,c,12,math.nan)
    with pytest.raises(RadarEvidencePending):h.corridor_clear(c[0],swept_centers(-.03,0.))
    h.corridor_clear(c[0],swept_centers(.03,0.))
    sectors=required_sectors(swept_centers(-.03,.12))
    assert 12 in sectors and 0 not in sectors and 6 in sectors and 18 in sectors
    assert required_sectors([(0.,0.)])==set(range(24))


def test_unrequired_sector_real_obstacle_still_blocks_body(rig):
    h,n,c=spatial_rig(rig);missing_sector(h,n,c,0)
    n.home.dock.low_scan.ranges[0]=.20
    with pytest.raises(ValueError,match='雷达障碍'):
        h.corridor_clear(c[0],swept_centers(-.03,0.))


def test_transient_radar_loss_preserves_phase_and_resumes(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    phase=h.phase;started=h.corridor_started
    h.corridor_clear.side_effect=RadarEvidencePending('missing')
    h.corridor_tick((.15,.15,.4),c[0])
    c[0]+=8.;h.corridor_tick((.15,.15,.4),c[0])
    assert h.phase==phase and h.corridor_active
    assert h.cmdpub.publish.call_args.args[0].angular.z==0
    assert h.escape_motion_pub.publish.call_args.args[0].data==0
    assert h.corridor_started>=started+8.
    h.corridor_clear.side_effect=None
    c[0]+=.1;h.corridor_tick((.15,.15,.4),c[0])
    assert h.corridor_wait_at is None
    assert h.cmdpub.publish.call_args.args[0].angular.z>0


def test_operator_cancel_while_radar_wait_revokes_ownership(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.corridor_clear.side_effect=RadarEvidencePending('missing')
    h.corridor_tick((.15,.15,.4),c[0]);h.cancel('operator')
    assert not h.corridor_active and not h.active
    assert h.escape_motion_pub.publish.call_args.args[0].data==0


def test_persistent_turn_blindness_switches_to_verified_reverse(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    original=h.corridor_route
    def clear(now,centers,turn=False):
        if turn:raise RadarEvidencePending('front blind')
    h.corridor_clear.side_effect=clear
    h.corridor_tick((.15,.15,.4),c[0])
    c[0]+=2.9;h.corridor_tick((.15,.15,.4),c[0])
    assert h.phase=='escape_turn'
    c[0]+=.2;h.corridor_tick((.15,.15,.4),c[0])
    assert h.phase=='escape_curve' and h.corridor_route is not original
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    assert h.escape_motion_pub.publish.call_args.args[0].data==0
    c[0]+=.2;h.corridor_tick((.15,.15,.4),c[0])
    assert h.cmdpub.publish.call_args.args[0].linear.x<0
    assert h.retreat_episode.attempts==1


def test_alternative_unknown_or_collision_cannot_authorize_reverse(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    original=h.corridor_route;progress=original.progress
    def clear(now,centers,turn=False):
        if turn:raise RadarEvidencePending('front blind')
        raise ValueError('rear occupied')
    h.corridor_clear.side_effect=clear
    h.corridor_tick((.15,.15,.4),c[0])
    c[0]+=3.1;h.corridor_tick((.15,.15,.4),c[0])
    assert h.phase=='escape_turn' and h.corridor_route is original
    assert original.progress==progress and h.retreat_episode.attempts==0
    assert 'rear occupied' in n._reason
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


def test_missing_route_does_not_invent_reverse(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.corridor_route=None;h.trace.points.clear()
    h.corridor_clear.side_effect=RadarEvidencePending('blind')
    h.corridor_tick((.15,.15,.4),c[0])
    c[0]+=3.1;h.corridor_tick((.15,.15,.4),c[0])
    assert h.phase=='escape_turn' and h.corridor_route is None
    assert '不足8cm' in n._reason


def test_half_turn_does_not_force_wrong_heading_reverse(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    x,y,a=n.home.dock.odom;n.home.dock.odom=(x,y,a+math.pi/2)
    # Candidate evaluation at actual pose; both directions are misaligned.
    h.corridor_clear.reset_mock()
    ok,reason=h.reconsider_corridor((.15,.15,.4),c[0])
    assert not ok and '偏差过大' in reason
    h.corridor_clear.assert_not_called()
    assert h.phase=='escape_turn'


def test_reassessment_is_rate_limited_and_preserves_task(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.corridor_clear.side_effect=RadarEvidencePending('blind')
    h.reconsider_corridor=Mock(return_value=(False,'no safe alternative'))
    for i in range(40):
        h.corridor_tick((.15,.15,.4),c[0]);c[0]+=.2
    assert h.reconsider_corridor.call_count==2
    assert h.corridor_active and h.phase=='escape_turn'
    assert all(v.args[0].linear.x==0 and v.args[0].angular.z==0 for v in h.cmdpub.publish.call_args_list)


@pytest.mark.parametrize('sector', [6, 12, 17])
def test_rear_positive_infinity_authorizes_coverage_not_phantom_hit(rig, sector):
    h,n,c=spatial_rig(rig);missing_sector(h,n,c,sector)
    h.corridor_clear(c[0],swept_centers(-.03,0.))
    h.corridor_clear(c[0],[(0.,0.)],turn=True)
    assert all(math.isfinite(x) and math.isfinite(y) for x,y in h.corridor_scan_points(c[0]))
    # A real hit from either laser still wins over no-return clearance.
    n.home.dock.scan.ranges[180]=.20
    with pytest.raises(ValueError,match='雷达障碍'):
        h.corridor_clear(c[0],swept_centers(-.03,0.))


@pytest.mark.parametrize('value', [math.nan, -math.inf, 0., 99.])
def test_rear_invalid_is_not_normal_no_return(rig,value):
    h,n,c=spatial_rig(rig);missing_sector(h,n,c,12,value)
    with pytest.raises(RadarEvidencePending):
        h.corridor_clear(c[0],swept_centers(-.03,0.))


def test_intermittent_good_scan_does_not_reset_accumulated_wait(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.reconsider_corridor=Mock(return_value=(False,'no safe alternative'))
    # A successful tick is not evidence of actual odometry progress.
    for i in range(20):
        h.corridor_tick_ready=Mock(side_effect=RadarEvidencePending('blind') if i%2==0 else None,
                                   return_value=True)
        h.corridor_tick((.15,.15,.4),c[0]);c[0]+=.4
    assert h.corridor_wait_total>=3.
    assert h.reconsider_corridor.call_count>=1


def test_real_motion_resets_accumulated_wait(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.corridor_wait_total=2.;h.corridor_progress=.09
    h.corridor_tick_ready=Mock(return_value=True)
    h.corridor_tick((.15,.15,.4),c[0])
    assert h.corridor_wait_total==0.


def handoff(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.active=True;n._state='hazard_recovery'
    h.clear_for_resume.return_value=False;h.resume_detail='front below 28cm'
    h.corridor_tick((.15,.15,.263),c[0])
    assert h.owner=='exploration' and h.active and h.phase=='confirming'
    h.wait_for_health=Mock(return_value=False);h.gate=Mock(return_value=True)
    h.retreat_values=Mock(return_value=(.15,.15,.263))
    h.confirm_near=Mock(return_value=True)
    h.pending=[dict(x=.4,y=0.,radius=.04,source='front')]
    h.corridor_near_trace=Mock();h.corridor_near_trace.plan.side_effect=ValueError('wrong heading')
    h.short_rear_plan=Mock(side_effect=ValueError('rear floor unknown'))
    return h,n,c


def test_new_near_without_safe_reverse_waits_without_latching_failure(rig):
    h,n,c=handoff(rig)
    h.tick(True)
    assert h.active and h.phase=='confirming' and n._state=='hazard_recovery'
    assert '保留任务' in n._reason
    assert h.cmdpub.publish.call_args.args[0].linear.x==0.
    h.short_rear_plan.assert_called_once()
    c[0]+=.2;h.tick(True);h.short_rear_plan.assert_called_once()


def test_new_near_uses_only_prechecked_reverse_length(rig):
    h,n,c=handoff(rig);h.short_rear_plan.side_effect=None;h.short_rear_plan.return_value=.05
    h.tick(True)
    assert h.phase=='reversing' and h.sonar_reverse_limit==.05, n._reason
    assert h.cmdpub.publish.call_args.args[0].linear.x<0.


def test_ground_protection_still_stops_turn(rig):
    h,n,c=prepare(rig);h.begin_corridor((.15,.15,.4),c[0])
    h.clear_for_resume.return_value=False;h.resume_detail='ground unsafe'
    with pytest.raises(ValueError,match='地面保护'):
        h.corridor_tick((.29,.15,.263),c[0])


def test_near_clears_after_confirmation_and_resumes_same_owner(rig):
    h,n,c=handoff(rig);h.pending=[];h.clear_for_resume.return_value=True
    h.tick(True)
    assert n._state=='running' and not h.active and h.phase=='idle'
    n._publish_resume.assert_called_once()
    h.short_rear_plan.assert_not_called()


def test_confirmed_obstacle_already_clear_needs_no_reverse_permission(rig):
    h,n,c=handoff(rig);h.clear_for_resume.return_value=True
    h.tick(True)
    assert h.phase=='reversing' and h.sonar_reverse_limit==0.
    assert h.cmdpub.publish.call_args.args[0].linear.x==0.
    h.short_rear_plan.assert_not_called()


def prepare_handoff(rig):
    h,n,c=prepare(rig)
    h.begin_corridor((.15,.15,.4),c[0])
    h.active=True;n._state='hazard_recovery';n.home.last_motion=c[0]-1
    h.near_sources=Mock(return_value=[])
    h.corridor_streams_ready=Mock(return_value=True)
    return h,n,c


def test_persistent_coverage_failure_waits_for_start_space_then_retains_goal(rig):
    from rk3576_footbath_exploration.corridor_escape import RadarEvidencePending
    h,n,c=prepare_handoff(rig)
    h.corridor_tick_ready=Mock(side_effect=RadarEvidencePending('missing sector'))
    h.reconsider_corridor=Mock(return_value=(False,'no alternative'))
    h.retreat_episode.attempts=2;h.start_space=False
    for i in range(65):
        now=c[0]+i*.2;h.start_space_at=now
        h.corridor_tick((.15,.15,.4),now)
    assert h.active and not h.corridor_active and h.phase=='waiting_start_space'
    n._publish_resume.assert_not_called()
    h.start_space=True;h.start_space_at=c[0]+14
    h.wait_for_start_space((.15,.15,.4),c[0]+14)
    h.wait_for_start_space((.15,.15,.4),c[0]+14.7)
    assert not h.active and n._state==n.RUNNING
    n._publish_resume.assert_called_once()
    assert h.escape_motion_pub.publish.call_args.args[0].data==0
    assert all(call.args[0].linear.x==0 and call.args[0].angular.z==0
               for call in h.cmdpub.publish.call_args_list)


@pytest.mark.parametrize('block',['fault','near','clear','stream','motion','owner','cancel'])
def test_handoff_cannot_bypass_protection_or_owner(rig,block):
    h,n,c=prepare_handoff(rig)
    if block=='fault':h.fault=1
    elif block=='near':h.near_sources.return_value=['front']
    elif block=='clear':h.clear_for_resume.return_value=False
    elif block=='stream':h.corridor_streams_ready.return_value=False
    elif block=='motion':n.home.last_motion=c[0]+10
    elif block=='owner':h.owner='navigation'
    elif block=='cancel':n._state='paused'
    assert not h.yield_corridor_to_exploration((.15,.15,.4),c[0])
    assert not h.yield_corridor_to_exploration((.15,.15,.4),c[0]+1)
    n._publish_resume.assert_not_called()


@pytest.mark.parametrize('block',['none','stale_high','stale_low','stamp','invalid'])
def test_handoff_needs_live_radar_but_not_sector_coverage(rig,block):
    h,n,c=spatial_rig(rig)
    n.home.dock.scan.ranges[60:90]=[math.inf]*30
    if block=='stale_high':n.home.dock.scan_at-=1
    elif block=='stale_low':n.home.dock.low_at-=1
    elif block=='stamp':n.home.dock.low_scan.header.stamp.sec-=1
    elif block=='invalid':n.home.dock.scan.ranges=[math.nan]*360
    assert h.corridor_streams_ready(c[0])==(block=='none')

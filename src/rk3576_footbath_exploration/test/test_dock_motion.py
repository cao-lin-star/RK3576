"""Geometric and state-level checks without connecting to any hardware."""
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PoseStamped
from rk3576_footbath_exploration.dock_motion import (
    DockMotion, alignment_translation_unsafe, entry_segment_distance,
    forward_point, straight_command, swept_obstacle)
from rk3576_footbath_exploration.home_return import HomeReturn


@pytest.mark.parametrize('yaw', [0, math.pi/2, -math.pi/2, math.pi])
def test_staging_is_in_front_of_recorded_heading(yaw):
    dock=(3.,-2.,yaw)
    staging=forward_point(dock,.5)
    assert math.hypot(staging[0]-dock[0],staging[1]-dock[1])==pytest.approx(.5)
    assert staging[2]==dock[2]
    v,w,progress,done=straight_command(staging,dock,-1,.5,.05,.01,.04,.10)
    assert done and v==w==0 and progress==pytest.approx(.5)


@pytest.mark.parametrize('direction', [1,-1])
def test_segment_distance_is_measured_not_timed(direction):
    start=(2.,3.,.7)
    current=start
    for i in range(2000):
        v,w,progress,done=straight_command(start,current,direction,.5,.05,.01,.04,.10)
        if done: break
        assert direction*v>0 and abs(v)<=.05 and abs(w)<=.08
        current=(current[0]+v*math.cos(current[2])*.05,
                 current[1]+v*math.sin(current[2])*.05,current[2]+w*.05)
    assert done and .49<=progress<=.51


@pytest.mark.parametrize('pose', [(0,.05,0),(0,0,.15),(-.04,0,0),(.54,0,0),(float('nan'),0,0)])
def test_deviation_wrong_direction_and_overshoot_reject(pose):
    with pytest.raises(ValueError):
        straight_command((0,0,0),pose,1,.5,.05,.01,.04,.1)


def test_rear_obstacle_is_checked_in_reverse():
    assert swept_obstacle([(-.25,0)],-1,.22,.01,.05)
    assert not swept_obstacle([(.25,0)],-1,.22,.01,.05)
    assert not swept_obstacle([(-.1,.30)],-1,.22,.01,.05)


def test_rear_dock_is_not_a_forward_obstacle_but_front_stays_protected():
    assert not swept_obstacle([(-.1,0),(-.2,.1)],1,.22,.01,.05)
    assert swept_obstacle([(.25,0)],1,.22,.01,.05)


def test_navigation_depart_does_not_send_nav_goal(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.navigation_session=True; h.phase='ready'; h.BUSY=HomeReturn.BUSY
    h.active_goals=False
    response=SimpleNamespace(success=False,message='')
    HomeReturn._depart(h,None,response)
    assert response.success and d.active and d.kind=='exit'
    h.nav.send_goal_async.assert_not_called()


def test_navigation_depart_rejects_interrupted_segment(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.navigation_session=True; h.phase='canceled'; h.BUSY=HomeReturn.BUSY
    d.kind='exit'
    response=SimpleNamespace(success=False,message='')
    HomeReturn._depart(h,None,response)
    assert not response.success and not d.active


def pose(x=0,y=0,yaw=0):
    p=PoseStamped(); p.pose.position.x=float(x); p.pose.position.y=float(y)
    p.pose.orientation.z=math.sin(yaw/2); p.pose.orientation.w=math.cos(yaw/2)
    return p


def test_saved_home_uses_mapping_pose_not_zero(tmp_path):
    import json
    from rk3576_footbath_exploration.map_home import save_home
    (tmp_path/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
    save_home(str(tmp_path/'map'),pose(1.,2.,.7))
    data=json.loads((tmp_path/'map.home.json').read_text())
    assert data['pose']['x']==1. and data['pose']['y']==2.
    assert data['pose']['yaw']==pytest.approx(.7)


def manager(monkeypatch):
    now=[100.]
    monkeypatch.setattr('rk3576_footbath_exploration.dock_motion.time.monotonic',lambda:now[0])
    h=Mock(); h.node.declare_parameter.side_effect=lambda name,default:SimpleNamespace(value=default)
    h.pose=pose(); h.current_pose.return_value=pose()
    h.recovery_timeout=8.; h.stable_time=1.
    d=DockMotion(h); h.dock=d; d.odom=(0.,0.,0.); d.odom_at=now[0]
    d._obstacle=Mock(return_value=False)
    return h,d,now


def test_exit_then_explore_only_after_stopped_at_distance(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.begin('exit')
    d.tick(True)
    h.node._begin_exploration.assert_not_called()
    d.odom=(.495,0,0); h.current_pose.return_value=pose(.495)
    d.tick(True)
    assert h.node._state=='dock_waiting'
    h.node._begin_exploration.assert_not_called()
    now[0]+=.7; d.odom_at=now[0]; d.tick(True)
    assert d.exit_complete and not d.active
    h.node._begin_exploration.assert_called_once()


def test_cancelled_segment_does_not_restart_or_reset_distance(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.begin('exit'); d.pause(); h.node._stop_publisher.reset_mock()
    d.tick(True)
    h.node._stop_publisher.publish.assert_not_called()
    with pytest.raises(ValueError): d.begin('exit')


def test_alignment_reports_observed_false_arrival_position_without_claiming_yaw_failure(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.current_pose.return_value=pose(.5+.128738,0,.8)
    with pytest.raises(ValueError,match='位置偏差12.87cm.*朝向偏差0.800rad待到点后对齐'):
        d.begin('align')
    assert not d.active and d.kind is None
    h.nav.send_goal_async.assert_not_called()


def test_precise_staging_does_not_require_heading_before_independent_alignment(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.current_pose.return_value=pose(.52,0,.8)
    d.begin('align')
    assert d.active and d.kind=='align'
    h.node._publish_lease.assert_called_with(False)


def test_exit_heading_rejection_reports_actual_error_and_stays_stopped(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.current_pose.return_value=pose(0,0,.1)
    with pytest.raises(ValueError,match='位置偏差0.00cm.*朝向偏差0.100rad'):
        d.begin('exit')
    assert not d.active and d.kind is None


def test_entry_requires_staging_position_and_original_heading(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.current_pose.return_value=pose(.5,0,math.pi)
    with pytest.raises(ValueError): d.begin('entry')
    assert not d.active
    h.current_pose.return_value=pose(.5,0,0)
    d.begin('entry'); d.tick(True)
    assert h.node._stop_publisher.publish.call_args.args[0].linear.x<0


def test_health_loss_revokes_lease_and_preserves_distance(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.begin('exit'); d.tick(False)
    h.node._publish_lease.assert_called_with(False)
    assert h.phase=='dock_waiting' and d.start==(0.,0.,0.)
    now[0]+=8.1; d.tick(False)
    h._fail.assert_called_once()


def test_nav_success_at_staging_is_not_docked(monkeypatch):
    h,d,now=manager(monkeypatch)
    h.generation=2; h.handle=None
    HomeReturn._result(h,Mock(result=lambda:SimpleNamespace(status=4)),2)
    assert h.phase=='dock_preparing'
    h.node._pause.assert_called_with('return_preparing','已到基站前方，等待旧导航退出后倒车')


@pytest.mark.parametrize('yaw', [2.35,-2.35])
def test_alignment_rotates_without_translation_then_hands_off(monkeypatch,yaw):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,yaw); h.current_pose.return_value=pose(.5,0,yaw)
    d.begin('align'); d.tick(True)
    cmd=h.node._stop_publisher.publish.call_args.args[0]
    assert cmd.linear.x==0 and cmd.angular.z*yaw<0
    assert abs(cmd.angular.z)<=.15 and d.kind=='align'
    now[0]+=2; d.odom=(.5,0.,0.); d.odom_at=now[0]
    h.current_pose.return_value=pose(.5,0,0)
    d.tick(True)
    assert d.kind=='align'
    now[0]+=1.1; d.odom_at=now[0]; d.tick(True)
    assert d.kind=='entry' and d.active


def test_alignment_oscillation_does_not_extend_deadline(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,1.); h.current_pose.return_value=pose(.5,0,1.)
    d.begin('align')
    for yaw in (1.,1.1,1.,1.1):
        now[0]+=3; d.odom=(.5,0.,yaw); d.odom_at=now[0]
        h.current_pose.return_value=pose(.5,0,yaw); d.tick(True)
    h._fail.assert_called_once()
    assert '角度未持续改善' in h._fail.call_args.args[0]
    assert d.kind=='align'


def test_alignment_checks_all_directions():
    assert swept_obstacle([(-.2,0)],0,.22,.01,.05)
    assert swept_obstacle([(0,.2)],0,.22,.01,.05)
    assert not swept_obstacle([(-.3,0)],0,.22,.01,.05)


def test_alignment_hard_position_drift_prevents_reverse(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,1.); h.current_pose.return_value=pose(.5,0,1.)
    d.begin('align')
    d.odom=(.555,0.,1.); h.current_pose.return_value=pose(.585,0,1.)
    d.tick(True)
    assert h.node._publish_lease.call_args.args[0] is False
    assert h._fail.call_count==0
    for _ in range(3):
        now[0]+=.21; d.odom_at=now[0]; d.tick(True)
    h._fail.assert_called_once()
    assert d.kind=='align'


def test_alignment_uses_its_actual_start_not_ideal_staging_boundary(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.539,0.,1.); h.current_pose.return_value=pose(.539,0.,1.)
    d.begin('align')
    # A small AMCL correction crosses the old absolute 4 cm staging boundary,
    # while odometry confirms that the chassis did not translate.
    h.current_pose.return_value=pose(.543,0.,.9)
    d.odom=(.539,0.,.9); d.tick(True)
    h._fail.assert_not_called()
    assert h.node._stop_publisher.publish.call_args.args[0].angular.z != 0


def test_alignment_small_map_only_correction_is_not_physical_translation():
    unsafe,suspect,map_shift,odom_shift=alignment_translation_unsafe(
        (.50,0.,1.),(.545,0.,.8),(2.,3.,1.),(2.005,3.,.8),.04,.02,.08,.05)
    assert not unsafe and not suspect
    assert map_shift==pytest.approx(.045) and odom_shift==pytest.approx(.005)


def test_alignment_small_corroborated_shift_waits_for_final_corridor_gate(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,1.); h.current_pose.return_value=pose(.5,0.,1.)
    d.begin('align')
    d.odom=(.525,0.,.9); h.current_pose.return_value=pose(.545,0.,.9)
    d.tick(True)
    h._fail.assert_not_called()
    h.node.get_logger.return_value.warning.assert_called_once()
    assert d.kind=='align'


def test_alignment_one_source_hard_jump_still_stops(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,1.); h.current_pose.return_value=pose(.5,0.,1.)
    d.begin('align')
    d.odom=(.5,0.,.9); h.current_pose.return_value=pose(.581,0.,.9)
    for _ in range(4):
        d.odom_at=now[0]; d.tick(True); now[0]+=.21
    h._fail.assert_called_once()
    assert 'map=0.081m, odom=0.000m' in h._fail.call_args.args[0]


def test_alignment_transient_heading_disagreement_stops_then_recovers(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,1.); h.current_pose.return_value=pose(.5,0.,1.)
    d.begin('align')
    d.odom=(.5,0.,.9); h.current_pose.return_value=pose(.5,0.,.7)
    d.tick(True)
    assert h._fail.call_count==0
    assert h.node._publish_lease.call_args.args[0] is False
    now[0]+=.2; d.odom_at=now[0]
    d.odom=(.5,0.,.8); h.current_pose.return_value=pose(.5,0.,.8)
    d.tick(True)
    assert h._fail.call_count==0
    assert h.node._stop_publisher.publish.call_args.args[0].angular.z != 0


def test_alignment_persistent_heading_disagreement_fails_closed(monkeypatch):
    h,d,now=manager(monkeypatch)
    d.odom=(.5,0.,1.); h.current_pose.return_value=pose(.5,0.,1.)
    d.begin('align')
    d.odom=(.5,0.,.9); h.current_pose.return_value=pose(.5,0.,.7)
    for _ in range(4):
        d.odom_at=now[0]; d.tick(True); now[0]+=.21
    h._fail.assert_called_once()
    assert '转角持续不一致' in h._fail.call_args.args[0]


def test_entry_distance_compensates_longitudinal_staging_error(monkeypatch):
    measured=entry_segment_distance((0.,0.,0.),(.543,-.004,0.),.5,.06,.04,.06)
    assert measured==pytest.approx(.543)
    h,d,now=manager(monkeypatch)
    d.odom=(.4,0.,0.); h.current_pose.return_value=pose(.543,-.004,0.)
    d.begin('entry')
    assert d.segment_distance==pytest.approx(.543)
    d.tick(True)
    cmd=h.node._stop_publisher.publish.call_args.args[0]
    assert cmd.linear.x<0


def test_entry_still_rejects_cross_track_or_large_longitudinal_error():
    with pytest.raises(ValueError):
        entry_segment_distance((0.,0.,0.),(.50,.041,0.),.5,.06,.04,.06)
    with pytest.raises(ValueError):
        entry_segment_distance((0.,0.,0.),(.57,0.,0.),.5,.06,.04,.06)

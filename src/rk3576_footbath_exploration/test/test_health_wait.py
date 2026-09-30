from types import SimpleNamespace as NS
import json
import pytest
from test_hazard_recovery import rig, normal_navigation, owner_context

CLEAR = (.155, .16, .40)


def recover(h, n, step):
    h.fault = 0
    step(.1, CLEAR)
    step(2.01, CLEAR)
    n.home.last_status = 100000.
    step(.7, CLEAR)


def test_transient_fault_resumes_mapping_without_reverse(rig):
    h,n,clock,step=rig
    h.fault=2
    step(values=CLEAR)
    assert h.active and h.kind=='health_wait'
    step(20.,CLEAR)
    assert h.health_wait_at is not None
    recover(h,n,step)
    assert n._state=='running' and not h.active
    assert all(c.args[0].linear.x==0 for c in h.cmdpub.publish.call_args_list)
    n._publish_resume.assert_called_once()


def test_stability_timer_restarts_on_new_fault(rig):
    h,n,clock,step=rig
    h.fault=4;step(values=CLEAR)
    h.fault=0;step(.1,CLEAR);step(1.5,CLEAR)
    h.fault=4;step(.1,CLEAR)
    h.fault=0;step(.1,CLEAR);step(1.5,CLEAR)
    assert h.health_wait_at is not None
    n._publish_resume.assert_not_called()


@pytest.mark.parametrize('bit',[256,512])
def test_control_fault_or_estop_never_auto_resumes(rig,bit):
    h,n,clock,step=rig
    h.fault=bit;step(values=CLEAR)
    recover(h,n,step)
    assert n._state=='paused_fault'
    n._publish_resume.assert_not_called()


def test_operator_stop_during_wait_cancels_resume(rig):
    h,n,clock,step=rig
    h.fault=2;step(values=CLEAR)
    n._state='paused_operator'
    recover(h,n,step)
    assert not h.active and n._state=='paused_operator'
    n._publish_resume.assert_not_called()


def test_return_keeps_original_token(rig):
    h,n,clock,step=rig
    n._state='returning_home';n.home.phase='returning';n.home.started=900.
    n.home.suspend_for_hazard.return_value=12
    n.home.hazard_resume_valid.return_value=True
    n.home.resume_after_hazard.return_value=True
    h.fault=2;step(values=CLEAR)
    recover(h,n,step)
    n.home.resume_after_hazard.assert_called_once_with(12)
    n._publish_resume.assert_not_called()


def test_navigation_keeps_same_goal(rig):
    h,n,clock,step=rig
    normal_navigation(h,n,clock)
    h.fault=2;step(values=CLEAR)
    identity=dict(h.owner_token)
    h.fault=0
    for dt in (.1,2.01,.7):
        clock[0]+=dt;owner_context(h,clock,'suspended')
        n.home.last_status=clock[0]
        step(0,CLEAR)
    assert h.phase=='resuming_owner'
    sent=json.loads(h.nav_owner.publisher.publish.call_args.args[0].data)
    assert sent==dict(identity,action='resume')
    owner_context(h,clock,'active');step(.1,CLEAR)
    assert n._state=='navigation_ready' and not h.active


def test_recovered_link_with_cliff_enters_existing_recovery(rig):
    h,n,clock,step=rig
    h.fault=2;step(values=CLEAR)
    h.fault=0;step(.1);step(2.01)
    n.home.last_status=100000.;step(.7)
    assert h.kind=='cliff' and h.phase=='reversing'


def test_recovery_preserves_distance_and_pending_obstacles(rig):
    h,n,clock,step=rig
    step();step(.7)
    pending=list(h.pending);start=h.start_odom
    h.fault=2;step(.1,progress=.02)
    h.fault=0;step(.1,progress=.02);step(2.01,progress=.02)
    assert h.phase=='reversing' and h.pending==pending and h.start_odom==start


@pytest.mark.parametrize('connected,age',[(False,0.),(True,5.),(True,float('nan'))])
def test_fresh_diagnostics_do_not_hide_lost_heartbeat(rig,connected,age):
    h,n,clock,step=rig
    h.bridge_connected=connected;h.heartbeat_age=age
    step(values=CLEAR)
    assert h.health_wait_at is not None
    n._publish_resume.assert_not_called()

@pytest.mark.parametrize('bit',[1,2,4,8,16,32,64,128,2048])
def test_recoverable_fault_preserves_mapping_and_budget(rig, bit):
    h,n,clock,step=rig
    n._exploration_started_at=clock[0]-100.
    began=n._exploration_started_at
    h.fault=bit;step(values=CLEAR)
    step(30.,CLEAR)
    assert h.health_wait_at is not None
    n._publish_resume.assert_not_called()
    recover(h,n,step)
    assert n._state=='running' and not h.active
    assert n._exploration_started_at >= began+32.
    n._publish_resume.assert_called_once()


def test_health_reason_identifies_map_delay(rig):
    h,n,clock,step=rig
    step(values=CLEAR)
    n._ages={'scan': .1, 'map': 6., 'odom': .1}
    n._maximum_age={'scan': 1., 'map': 5., 'odom': 1.}
    assert '地图更新延迟 6.00秒' in h.health_problem(False,clock[0],'exploration')


def test_health_reason_identifies_tf_delay(rig):
    h,n,clock,step=rig
    step(values=CLEAR)
    n.home.current_pose.return_value=None
    n.home.pose_error='map->base_footprint TF age=1.250s (limit 1.0s)'
    assert 'TF age=1.250s' in h.health_problem(True,clock[0],'exploration')


def test_normal_exploration_does_not_build_exit_scan_history(rig):
    from unittest.mock import Mock
    h,n,clock,step=rig
    h.collect_corridor_scan=Mock()
    step(values=CLEAR)
    h.collect_corridor_scan.assert_not_called()
    h.escape_requested=True
    step(.2,values=CLEAR)
    h.collect_corridor_scan.assert_called_once()


def test_resume_does_not_reset_mapping_budget():
    from unittest.mock import Mock
    from rk3576_footbath_exploration.supervisor import ExplorationSupervisor
    n=Mock()
    n.hazard.overlay.motion_blocked.return_value=False
    n.home.navigation_session=False
    n.home.dock.enabled=False
    n._exploration_started_at=123.
    assert ExplorationSupervisor._begin_exploration(n)
    assert n._exploration_started_at==123.
    n._exploration_started_at=None
    assert ExplorationSupervisor._begin_exploration(n)
    assert isinstance(n._exploration_started_at,float)

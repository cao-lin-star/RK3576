"""No hardware: exercise cancellation, bounded reverse and failure paths."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import math
import pytest
from builtin_interfaces.msg import Time
from rk3576_footbath_exploration.hazard_recovery import (
    HazardRecovery, hazard_kind, ground_clear, reverse_progress)


def test_thresholds_and_hysteresis():
    assert hazard_kind(.215, .16, .54) == 'cliff'
    assert hazard_kind(.155, .220, .54) == 'cliff'
    assert hazard_kind(.155, .16, .20) == 'sonar'
    assert hazard_kind(.20, .21, .25) is None
    assert not ground_clear(.20, .16, .30)
    assert ground_clear(.195, .20, .28)
    with pytest.raises(ValueError): hazard_kind(math.nan,.16,.3)


@pytest.mark.parametrize('pose', [(0,.04,0),(0,0,.11),(.02,0,0),(math.nan,0,0)])
def test_reverse_rejects_drift_and_wrong_direction(pose):
    with pytest.raises(ValueError): reverse_progress((0,0,0),pose)


@pytest.fixture
def rig(monkeypatch):
    clock=[1000.]
    monkeypatch.setattr('rk3576_footbath_exploration.hazard_recovery.time.monotonic',lambda:clock[0])
    n=Mock()
    n.get_clock.return_value.now.return_value.to_msg.return_value=Time()
    n.create_publisher.side_effect=lambda *args:Mock()
    n.RUNNING='running'; n.PAUSED_FAULT='paused_fault'; n._state='running'
    n.declare_parameter.side_effect=lambda key,default:NS(value=default)
    n.home.current_pose.return_value=NS(pose=NS(position=NS(x=0.,y=0.),
                                               orientation=NS(x=0.,y=0.,z=0.,w=1.)))
    n.home.dock.odom=(0.,0.,0.); n.home.dock.odom_at=1000.
    n.home.active_goals=False; n.home.last_status=1001.
    n.home.last_motion=999.
    n._cancel_client.service_is_ready.return_value=True
    n._cancel_client.call_async.return_value.done.return_value=True
    n._cancel_client.call_async.return_value.result.return_value=NS(return_code=0)
    def pause(state,reason): n._state=state; n._reason=reason
    n._pause.side_effect=pause
    h=HazardRecovery(n)
    def step(dt=0., values=(.26,.16,.54), progress=0.):
        clock[0]+=dt
        h.ranges={k:(v,clock[0]) for k,v in zip(('left','right','sonar'),values)}
        h.source=1; h.source_at=clock[0]; h.fault_at=clock[0]
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
    assert h.phase=='failed' and not h.zones


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

import math
from unittest.mock import Mock
import pytest
from test_hazard_recovery import rig
from test_corridor_escape import prepare, spatial_rig
from rk3576_footbath_exploration.corridor_route import FrozenRoute


def test_mark_ack_wait_preserves_trace_and_ground(rig):
    h,n,c,step=rig
    h.active=True;h.owner='exploration';h.kind='sonar';h.phase='marking'
    n._state='hazard_recovery';h.start_odom=(0.,0.,0.)
    h.marking_tick=Mock(return_value=True)
    for i in range(11):
        h.trace.observe(c[0]-1.+i*.1,(-.20+i*.02,0.,0.),True)
    for _ in range(40):step(.1,(.15,.15,.8))
    assert h.trace.plan(c[0],(0.,0.,0.),.08,.3) >= .18
    assert h.ground_history
    h.active=False;n._state='running'
    step(.1,(.15,.15,.8))
    assert h.trace.plan(c[0],(0.,0.,0.),.08,.3) >= .18


def test_valid_reverse_updates_ground_even_before_release(rig):
    h,n,c,step=rig
    h.active=True;h.owner='exploration';h.kind='sonar';h.phase='marking_recheck'
    n._state='hazard_recovery';h.start_odom=(0.,0.,0.)
    h.marking_tick=Mock(return_value=True)
    for i in range(6):step(.1,(.15,.15,.27),i*.02)
    assert len(h.ground_history)>=3
    assert h.ground_history[-1][1]<-.07


def test_invalid_ground_does_not_add_evidence(rig):
    h,n,c,step=rig
    h.active=True;h.owner='exploration';h.kind='sonar';h.phase='marking_recheck'
    n._state='hazard_recovery';h.start_odom=(0.,0.,0.)
    h.marking_tick=Mock(return_value=True)
    step(.1,(.29,.15,.8))
    assert not h.ground_history


def test_already_facing_traversed_exit_needs_no_turn(rig):
    h,n,c=prepare(rig)
    h.trace.points.clear()
    h.trace.points.extend((909+i*.02,(.20-i*.02,0.,0.),False) for i in range(11))
    n.home.dock.odom=(0.,0.,0.)
    def check(now,centers,turn=False):
        if turn:raise ValueError('rotation blocked')
    h.corridor_clear.side_effect=check
    assert h.begin_corridor((.15,.15,.8),c[0])
    assert h.phase=='escape_forward'
    assert h.cmdpub.publish.call_args.args[0].linear.x==0.
    h.corridor_clear.side_effect=ValueError('occupied')
    assert not h.begin_corridor((.15,.15,.8),c[0])


@pytest.mark.parametrize('observed',[False,True])
def test_obstacle_reason_identifies_source_and_position(rig,observed):
    h,n,c=spatial_rig(rig)
    z=dict(x=.2,y=0.,radius=.04)
    if observed:h.overlay.observations=[z]
    else:h.zones=[z]
    with pytest.raises(ValueError,match=('待确认观察点' if observed else '已确认障碍')):
        h.corridor_clear(c[0],[(0.,0.)],turn=True)

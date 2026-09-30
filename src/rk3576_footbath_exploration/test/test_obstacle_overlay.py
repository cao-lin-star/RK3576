"""Safety boundaries for static recording, manual edits and persistence."""
import json
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from std_msgs.msg import String
from rk3576_footbath_exploration.obstacle_overlay import StopGate, ObstacleOverlay, write_sidecar, load_sidecar


def feed(g,now,pose=(0.,0.,0.),speed=(0.,0.,0.),age=0.):
    g.update(now,now,age,pose,speed)


def test_continuous_stop_and_inflight_cutoff():
    g=StopGate()
    for i in range(11): feed(g,10+i*.1)
    assert g.ready(11)
    assert g.after()==pytest.approx(10.9)
    assert not g.ready(11.3)  # stale odometry cannot claim stopped


@pytest.mark.parametrize('pose,speed', [((.009,0,0),(0,0,0)),((0,0,.016),(0,0,0)),
                                       ((0,0,0),(.006,0,0)),((0,0,0),(0,0,.011))])
def test_drift_or_actual_twist_restarts_stop_window(pose,speed):
    g=StopGate()
    for i in range(11):feed(g,10+i*.1)
    feed(g,11.1,pose,speed)
    assert not g.ready(11.1)


@pytest.mark.parametrize('age',[-.01,.251,math.nan])
def test_invalid_time_never_authorizes_stop(age):
    g=StopGate()
    for i in range(11):feed(g,10+i*.1)
    feed(g,11.1,age=age)
    assert not g.ready(11.1)


def test_replayed_odometry_resets_confirmation():
    g=StopGate()
    for i in range(11):feed(g,10+i*.1)
    feed(g,11.)
    assert not g.ready(11.)


def make_map(tmp_path):
    prefix=tmp_path/'sample'
    (tmp_path/'sample.yaml').write_text('image: sample.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    (tmp_path/'sample.pgm').write_bytes(b'P5\n1 1\n255\n'+bytes([254]))
    return str(prefix)


def mark(kind='manual'):
    return dict(id='a',kind=kind,x=1.,y=1.,radius=.04,expires=0.)


def test_saved_overlay_is_bound_to_image_and_separates_observation(tmp_path):
    prefix=make_map(tmp_path)
    write_sidecar(prefix,[mark()],[mark('observe')])
    zones,obs=load_sidecar(prefix)
    assert zones[0]['kind']=='manual' and obs[0]['kind']=='observe'
    (tmp_path/'sample.pgm').write_bytes(b'changed map')
    with pytest.raises(ValueError,match='不匹配'):load_sidecar(prefix)


def test_legacy_cliff_loads_but_unknown_overlay_rejected(tmp_path):
    prefix=make_map(tmp_path)
    file=tmp_path/'sample.hazards.json'
    file.write_text(json.dumps(dict(frame_id='map',zones=[mark('cliff')])))
    assert load_sidecar(prefix)[0][0]['kind']=='cliff'
    file.write_text(json.dumps(dict(frame_id='map',zones=[mark('manual')])))
    with pytest.raises(ValueError):load_sidecar(prefix)


def editor():
    o=ObstacleOverlay.__new__(ObstacleOverlay)
    o.session='current';o.revision=3;o.pending=None;o.unapplied=None;o.prefix='';o.deleted=[]
    o.observations=[mark('observe')];o.observations[0]['id']='yellow'
    o.h=NS(zones=[mark()],publish_zones=Mock(return_value=99))
    o.n=Mock();o.n.home.current_pose.return_value=NS(pose=NS(position=NS(x=0.,y=0.)))
    o.editable=lambda now:True
    o.ack=Mock()
    o.grid=NS(header=NS(frame_id='map'),info=NS(resolution=.05,width=200,height=200,
        origin=NS(position=NS(x=0.,y=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
    return o


def request(o,**kw):
    import time
    d=dict(id='edit',session='current',revision=3,sent_at=time.time(),op='add',x=2.,y=2.,radius=.04)
    d.update(kw);o.edit(String(data=json.dumps(d)))


@pytest.mark.parametrize('changes',[dict(session='previous'),dict(revision=2),dict(sent_at=0),
    dict(radius=.151),dict(x=math.nan),dict(x=100.),dict(x=.01,y=.01),dict(op='erase_all')])
def test_bad_edit_has_no_planning_effect(changes):
    o=editor();request(o,**changes)
    assert o.h.zones==[mark()]
    assert o.pending is None
    assert not json.loads(o.ack.publish.call_args.args[0].data)['ok']


def test_moving_robot_rejects_manual_add():
    o=editor();o.editable=lambda now:False;request(o)
    assert o.h.zones==[mark()] and o.pending is None


def test_manual_edit_waits_for_costmaps_and_does_not_move():
    o=editor();request(o)
    assert len(o.h.zones)==2 and o.h.zones[-1]['kind']=='manual'
    assert o.pending['stamp']==99 and o.motion_blocked()
    o.ack.publish.assert_not_called()  # no premature success
    o.n._publish_resume.assert_not_called()
    o.n._publish_lease.assert_not_called()
    o.n._publish_zero.assert_not_called()


def test_delete_id_cannot_erase_neighbors_or_slam():
    o=editor();request(o,op='delete',obstacle_id='a')
    assert o.h.zones==[] and len(o.observations)==1
    assert o.deleted[0]['id']=='a'
    o.h.publish_zones.assert_called_once()


def test_unknown_or_missing_lidar_is_not_clear():
    o=editor();o.scans={}
    assert not o.lidar_clear((1,1),5.)


def test_observation_only_and_three_fresh_independent_echoes():
    from rk3576_footbath_exploration.near_obstacle import StationaryConfirmation
    from builtin_interfaces.msg import Time
    o=editor();o.observations=[];o.confirm=StationaryConfirmation();o.stable=lambda now:True
    o.review_required=False;o.gate=NS(after=lambda:10.9);o.tf_stable_at=10.
    o.h.side_enabled=True;o.h.sonar_samples={};o.scans={}
    o.n.home.buffer.lookup_transform.return_value=NS(transform=NS(translation=NS(x=1.,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    for stamp in (10.5,11.,11.,11.3):
        m=NS(range=.12,header=NS(frame_id='side_left',stamp=Time(sec=int(stamp),nanosec=int(stamp%1*1e9))))
        o.h.sonar_samples={'side_left':(m,stamp,stamp)};o.observe(stamp)
    assert not o.observations
    stamp=11.6;m.header.stamp=Time(sec=11,nanosec=600000000)
    o.h.sonar_samples={'side_left':(m,stamp,stamp)};o.observe(stamp)
    assert len(o.observations)==1 and o.observations[0]['kind']=='observe'
    assert o.h.zones==[mark()]  # observation never enters planning publisher
    assert o.observations[0]['lidar_clear'] is False


def test_cancel_ack_and_terminal_status_required():
    o=editor();o.n.home.active_goals=False;o.n._overlay_cancel_at=10.
    o.n._overlay_cancel_future=None
    assert not o.cancellation_done()
    f=Mock();f.done.return_value=True;o.n._overlay_cancel_future=f
    f.result.return_value=NS(return_code=0,goals_canceling=[object()])
    o.n.home.last_status=9.
    assert not o.cancellation_done()
    o.n.home.last_status=11.
    assert o.cancellation_done()
    o.n.home.active_goals=True
    assert not o.cancellation_done()
    o.n.home.active_goals=False;f.result.return_value=NS(return_code=0,goals_canceling=[])
    assert o.cancellation_done()  # already idle: no new status heartbeat required


def test_tf_jump_restarts_settling_without_pausing_mapping_or_navigation():
    from builtin_interfaces.msg import Time
    for localization in (True,False):
        o=editor();o.tf_previous=(0.,0.,0.);o.tf_anchor=(0.,0.,0.)
        o.tf_stable_at=8.;o.tf_fresh_at=8.;o.review_required=False
        o.n.home.localization=localization;o.confirm=Mock()
        o.n.get_clock.return_value.now.return_value.nanoseconds=10000000000
        o.n.home.buffer.lookup_transform.return_value=NS(header=NS(stamp=Time(sec=9,nanosec=850000000)),
            transform=NS(translation=NS(x=.2,y=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        o.update_tf(10.)
        assert o.tf_stable_at==10.
        assert not o.review_required
        assert not o.h.zones[0].get('needs_review')
        assert not o.n._pause.called
        assert o.h.zones == [mark()]
        o.confirm.reset.assert_called_once()
        o.gate=NS(ready=lambda now:True)
        assert not o.stable(10.)
        # Fresh unchanged TF allows collection again without operator review.
        o.n.get_clock.return_value.now.return_value.nanoseconds=11000000000
        o.n.home.buffer.lookup_transform.return_value.header.stamp=Time(sec=10,nanosec=850000000)
        o.update_tf(11.)
        assert o.stable(11.)
        assert not o.motion_blocked()


def test_missing_one_costmap_keeps_motion_locked_after_http_timeout():
    o=editor();o.pending=None;o.unapplied=dict(stamp=42,at=10.)
    assert o.motion_blocked()


def test_failed_persistence_leaves_live_obstacles_unchanged(monkeypatch):
    o=editor();o.prefix='/map/current'
    monkeypatch.setattr('rk3576_footbath_exploration.obstacle_overlay.write_sidecar',Mock(side_effect=OSError('disk full')))
    request(o)
    assert o.h.zones==[mark()] and o.pending is None
    o.h.publish_zones.assert_not_called()


def test_legacy_review_flag_does_not_block_loaded_obstacles(tmp_path):
    prefix=make_map(tmp_path)
    zones=[dict(mark(k),needs_review=True) for k in ('near','manual','cliff')]
    write_sidecar(prefix,zones,[dict(mark('observe'),needs_review=True)])
    loaded,observations=load_sidecar(prefix)
    assert len(loaded)==3 and len(observations)==1
    assert all('needs_review' not in z for z in loaded+observations)
    assert [(z['x'],z['y'],z['radius']) for z in loaded]==[(1.,1.,.04)]*3


def test_line_add_is_continuous_and_acknowledged_as_one_edit():
    o=editor();request(o,op='add_line',points=[dict(x=2.,y=2.),dict(x=3.,y=2.)])
    added=o.h.zones[1:]
    assert len(added)>2 and added[0]['x']==2. and added[-1]['x']==3.
    assert all(b['x']-a['x']<=a['radius']+1e-9 for a,b in zip(added,added[1:]))
    assert o.revision==4 and o.pending
    o.h.publish_zones.assert_called_once()


@pytest.mark.parametrize('points',[[dict(x=2.,y=2.),dict(x=20.,y=2.)],
    [dict(x=2.,y=2.),dict(x=.01,y=.01)], [dict(x=2.,y=2.),dict(x=float('nan'),y=2.)]])
def test_bad_line_rejects_whole_transaction(points):
    o=editor();request(o,op='add_line',points=points)
    assert o.h.zones==[mark()] and not o.pending and o.revision==3


def test_line_erase_removes_crossed_records_not_neighbors():
    o=editor();neighbor=dict(mark(),id='neighbor',y=2.)
    o.h.zones.append(neighbor)
    request(o,op='delete_line',points=[dict(x=.5,y=1.),dict(x=1.5,y=1.)])
    assert o.h.zones==[neighbor] and not o.observations
    assert len(o.deleted)==2 and o.pending


def test_line_exceeds_old_capacity_but_still_requires_pause():
    o=editor();request(o,op='add_line',points=[dict(x=1.,y=2.),dict(x=9.,y=2.)],radius=.02)
    assert len(o.h.zones)>128 and o.pending
    o=editor();o.editable=lambda now:False
    request(o,op='delete_line',points=[dict(x=.5,y=1.),dict(x=1.5,y=1.)])
    assert o.h.zones==[mark()] and not o.pending


def test_navigation_line_edit_survives_reload(tmp_path):
    o=editor();o.prefix=make_map(tmp_path)
    request(o,op='add_line',points=[dict(x=2.,y=2.),dict(x=2.5,y=2.)])
    zones,observations=load_sidecar(o.prefix)
    assert zones==o.h.zones and observations==o.observations

def test_long_line_and_large_sidecar(tmp_path):
    o=editor();o.validate_add=lambda x,y,r:None
    request(o,op='add_line',points=[dict(x=2.,y=2.),dict(x=32.,y=2.)],radius=.02)
    assert len(o.h.zones)>1000 and o.pending
    prefix=make_map(tmp_path)
    write_sidecar(prefix,o.h.zones*2,[])
    loaded,_=load_sidecar(prefix)
    assert len(loaded)==2*len(o.h.zones)


def test_fast_stop_observation_survives_slow_overlay_maintenance(monkeypatch):
    import rk3576_footbath_exploration.obstacle_overlay as mod
    o=editor()
    o.last_publish=100.
    o.update_tf=Mock();o.observe=Mock()
    o.pub=Mock()
    o.saved_signature=None;o.started=1.;o.review_required=False
    o.error='';o.h.applied={}
    o.observations=[]
    monkeypatch.setattr(mod.time,'monotonic',lambda:100.1)
    o.tick()
    o.update_tf.assert_called_once_with(100.1)
    o.observe.assert_called_once_with(100.1)
    o.pub.publish.assert_not_called()
    monkeypatch.setattr(mod.time,'monotonic',lambda:100.6)
    o.tick()
    o.pub.publish.assert_called_once()

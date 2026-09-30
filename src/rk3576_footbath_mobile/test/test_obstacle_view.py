import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rk3576_footbath_mobile.obstacle_view import ObstacleView


def view():
    g=Mock();g.mode='auto_mapping';g.mode_started_at=1.;g.manual_active=False
    g.home_status={'supervisor_state':'running'}
    g.get_clock.return_value.now.return_value.nanoseconds=100000000000
    o=ObstacleView(g);o.seen=time.monotonic()
    o.overlay=dict(session='s',started_at=2.,revision=1,editable=True)
    o.explore=dict(stamp=100.,goal=dict(id='g',x=1.,y=1.,sent_at=99.),unreachable=[])
    o.explore_seen=time.monotonic()
    return o


def plan(stamp=100,frame='map',x=1.):
    p=Path();p.header.frame_id=frame;p.header.stamp.sec=stamp
    pose=PoseStamped();pose.pose.position.x=x;pose.pose.position.y=1.
    p.poses=[pose];return p


def test_current_path_shown_then_hidden_on_pause():
    o=view();o.on_plan(plan());assert o.view()['route']
    o.g.home_status={'supervisor_state':'paused_operator'}
    assert not o.view()['route'] and o.view()['goal'] is None


def test_old_wrong_goal_or_wrong_frame_path_is_hidden():
    for p in (plan(98),plan(frame='odom'),plan(x=4.)):
        o=view();o.on_plan(p);assert not o.view()['route']


def test_switch_map_or_session_hides_old_overlay():
    o=view();o.g.mode_started_at=101.
    result=o.view();assert not result['fresh'] and not result['editable'] and result['goal'] is None


def test_stale_overlay_is_not_editable():
    o=view();o.seen-=3
    assert not o.view()['editable']


def test_committed_route_persists_while_same_goal_heartbeat_is_live():
    o=view();o.on_plan(plan());o.route['at']-=30.
    assert o.view()['route']
    o.explore_seen-=3.
    assert not o.view()['route']  # dead explorer must not leave a green stale path


def test_line_payload_forwarded_in_navigation_and_mapping():
    import json
    for mode in ('auto_mapping','navigation'):
        o=view();o.g.mode=mode
        points=[dict(x=1.,y=1.),dict(x=2.,y=1.)]
        o.submit(dict(session='s',revision=1,op='add_line',points=points,radius=.04),Mock(),{})
        sent=json.loads(o.pub.publish.call_args.args[0].data)
        assert sent['points']==points and sent['op']=='add_line'

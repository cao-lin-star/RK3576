"""Offline map metadata -> gateway launch -> supervisor -> return target tests.

ROS constructors, subprocesses, publishers and action clients are mocked. No
robot or ROS node is started; real MapStore, Gateway, HomeReturn and DockMotion
logic verifies the destination and conservative departure gate end to end.
"""
import hashlib
import json
import math
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from rk3576_footbath_mobile.gateway import Gateway
from rk3576_footbath_mobile.map_store import MapStore
from rk3576_footbath_exploration.home_return import HomeReturn, restored_home_pose
from rk3576_footbath_exploration.dock_motion import pose_tuple


DOCK = dict(x=8., y=9., yaw=-.7)
MANUAL = dict(x=2., y=3., yaw=.4)


def pose(data):
    result = PoseStamped()
    result.header.frame_id = 'map'
    result.pose.position.x, result.pose.position.y = data['x'], data['y']
    result.pose.orientation.z = math.sin(data['yaw']/2)
    result.pose.orientation.w = math.cos(data['yaw']/2)
    return result


def map_fixture(tmp_path, saved=DOCK):
    path = tmp_path/'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    image = tmp_path/'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\xff')
    if saved is not None:
        data = dict(version=1, frame_id='map', pose=saved,
                    geometry=dict(resolution=.05, origin=[0, 0, 0]),
                    image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())
        (tmp_path/'map.home.json').write_text(json.dumps(data))
    return path


def launched_gateway(tmp_path, monkeypatch, saved=DOCK, return_home=None, departure=False):
    path = map_fixture(tmp_path, saved)
    popen = Mock(return_value=Mock(pid=1234))
    monkeypatch.setattr('rk3576_footbath_mobile.gateway.subprocess.Popen', popen)
    monkeypatch.setattr('rk3576_footbath_mobile.gateway.os.getpgid', lambda pid: 1234)
    g = SimpleNamespace(mode='idle', child=None, child_pgid=None, ws=tmp_path,
        side_ultrasonic_enabled=True,
        map_name='', mapping_scan_source='', depart_from_dock=True,
        map_store=MapStore([tmp_path]), _stop_child=Mock(),
        _group_exists=Mock(return_value=False), tf_buffer=Mock(), get_logger=Mock(),
        pending_departure_goal=None, return_requested=False, initial_pub=Mock(),
        count_subscribers=Mock(return_value=1), get_clock=Mock(), confirmed_initial_pose=None)
    g.get_clock().now().nanoseconds = 100_000_000_000
    for name in ('_initial', '_amcl_pose', '_return_initial_pose'):
        setattr(g, name, MethodType(getattr(Gateway, name), g))
    Gateway._launch(g, 'navigation', str(path), return_home=return_home, depart_from_dock=departure)
    arguments = dict(item.split(':=', 1) for item in popen.call_args.args[0] if ':=' in item)
    return g, arguments


def home(monkeypatch, args, *, localization=True, current=MANUAL):
    for name in ('Buffer', 'TransformListener', 'ActionClient'):
        monkeypatch.setattr('rk3576_footbath_exploration.home_return.'+name, Mock())
    monkeypatch.setattr('rk3576_footbath_exploration.home_return.time.monotonic', lambda: 100.)
    params = {'return_home.localization_mode': localization,
              'dock.navigation_session': args.get('return_session', 'false') != 'true',
              'dock.departure_required': args.get('depart_from_dock', 'true') == 'true',
              'return_home.pose_json': args['home_pose_json']}
    n = Mock()
    n.declare_parameter.side_effect = lambda name, default: SimpleNamespace(value=params.get(name, default))
    n._home_inputs_ready.return_value = True
    n._cancel_client.service_is_ready.return_value = True
    n._home_behavior_tree = 'home_return_precise.xml'
    n._created_at = 99.
    n._state, n._reason = 'waiting_for_health', ''
    n._last_explore_status = 'not_received'
    n.get_clock().now().to_msg.return_value = Time(sec=100)
    h = HomeReturn(n)
    h.current_pose = Mock(return_value=pose(current))
    h.publish = Mock()
    h.dock.odom = (current['x'], current['y'], current['yaw'])
    h.dock.odom_at = 100.
    h.nav.server_is_ready.return_value = True
    def navigation_ready():
        h.phase = 'navigation_ready'
        n._state = 'navigation_ready'
    n._begin_exploration.side_effect = navigation_ready
    return h


def confirm_manual(g):
    g._initial(MANUAL)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'; msg.header.stamp.sec = 101
    msg.pose.pose = pose(MANUAL).pose
    g._amcl_pose(msg)
    assert g.confirmed_initial_pose == MANUAL


def test_saved_dock_survives_manual_seed_and_off_dock_navigation(tmp_path, monkeypatch):
    g, args = launched_gateway(tmp_path, monkeypatch)
    assert json.loads(args['home_pose_json']) == DOCK
    assert args.get('return_session', 'false') == 'false'
    h = home(monkeypatch, args)
    confirm_manual(g)
    h.observe_odom(Odometry()); h.tick(True)
    response = SimpleNamespace(success=False, message='')
    h._depart(None, response)
    assert response.success and h.phase == 'navigation_ready' and not h.dock.active
    assert pose_tuple(h.pose) == pytest.approx(tuple(DOCK.values()))
    assert g.saved_map_home == DOCK
    # The actual NavigateToPose return goal is dock+50cm, not manual+50cm.
    assert h.request()[0]
    h._send()
    sent = h.nav.send_goal_async.call_args.args[0].pose
    assert sent.pose.position.x == pytest.approx(DOCK['x'] + .5*math.cos(DOCK['yaw']))
    assert sent.pose.position.y == pytest.approx(DOCK['y'] + .5*math.sin(DOCK['yaw']))
    g._goal = Mock()
    g._return_initial_pose()
    g._goal.assert_called_once_with(MANUAL)


def test_true_dock_departure_still_runs_original_50cm_segment(tmp_path, monkeypatch):
    _, args = launched_gateway(tmp_path, monkeypatch, departure=True)
    h = home(monkeypatch, args, current=DOCK)
    response = SimpleNamespace(success=False, message='')
    h._depart(None, response)
    assert response.success and h.dock.active and h.dock.kind == 'exit'
    assert h.dock.segment_distance == .5 and h.dock.direction == 1
    assert pose_tuple(h.pose) == pytest.approx(tuple(DOCK.values()))
    h.nav.send_goal_async.assert_not_called()


@pytest.mark.parametrize('current', [MANUAL, dict(x=8., y=9., yaw=-.6)])
def test_departure_selected_but_not_at_saved_dock_rejects_without_rebinding(tmp_path, monkeypatch, current):
    _, args = launched_gateway(tmp_path, monkeypatch, departure=True)
    h = home(monkeypatch, args, current=current)
    response = SimpleNamespace(success=False, message='')
    h._depart(None, response)
    assert not response.success and not h.dock.active and h.dock.kind is None
    assert '未对齐' in response.message
    assert '取消“当前位于基站内”勾选' in response.message
    assert pose_tuple(h.pose) == pytest.approx(tuple(DOCK.values()))
    h.nav.send_goal_async.assert_not_called()


def test_no_metadata_never_invents_dock_from_manual_seed_or_stationary_odom(tmp_path, monkeypatch):
    g, args = launched_gateway(tmp_path, monkeypatch, saved=None)
    assert json.loads(args['home_pose_json']) == {} and g.saved_map_home is None
    h = home(monkeypatch, args)
    confirm_manual(g)
    for _ in range(5):
        h.observe_odom(Odometry()); h.tick(True)
    assert h.pose is None
    HomeReturn.publish(h)
    # The existing phone button is disabled when home.available is false.
    assert json.loads(h.publisher.publish.call_args.args[0].data)['available'] is False
    assert h.request()[0] is False
    # Explicitly selecting off-dock navigation remains available without a dock.
    response = SimpleNamespace(success=False, message='')
    h._depart(None, response)
    assert response.success and h.phase == 'navigation_ready' and not h.dock.active
    h.tick(True)
    assert h.phase == 'navigation_ready' and h.pose is None
    g._goal = Mock(); g._return_initial_pose()
    g._goal.assert_called_once_with(MANUAL)


def test_no_metadata_blocks_claimed_automatic_dock_departure(tmp_path, monkeypatch):
    _, args = launched_gateway(tmp_path, monkeypatch, saved=None, departure=True)
    h = home(monkeypatch, args)
    response = SimpleNamespace(success=False, message='')
    h._depart(None, response)
    assert not response.success and not h.dock.active and h.pose is None
    assert '没有有效基站' in response.message


def test_map_handoff_preserves_its_original_home_not_the_saved_map_fallback(tmp_path, monkeypatch):
    original = dict(x=1., y=2., yaw=.3)
    g, args = launched_gateway(tmp_path, monkeypatch, return_home=original)
    assert args['return_session'] == 'true'
    h = home(monkeypatch, args)
    assert pose_tuple(h.pose) == pytest.approx(tuple(original.values()))
    assert not h.navigation_session
    assert g.saved_map_home is None  # localization seed comes from handoff current_pose


def test_mapping_still_records_actual_stationary_start(monkeypatch):
    h = home(monkeypatch, dict(home_pose_json='{}'), localization=False)
    h.observe_odom(Odometry())
    h.stationary_since = 98.
    h.tick(True)
    assert pose_tuple(h.pose) == pytest.approx(tuple(MANUAL.values()))
    assert h.phase == 'ready'


@pytest.mark.parametrize('payload', ['{}', 'null', '{"x":1}', '[]', '{"x":NaN,"y":0,"yaw":0}'])
def test_required_handoff_rejects_missing_or_invalid_home(payload):
    with pytest.raises(ValueError):
        restored_home_pose(payload, required=True)

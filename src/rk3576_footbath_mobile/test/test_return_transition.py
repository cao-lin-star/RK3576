from unittest.mock import Mock
import pytest
from rk3576_footbath_mobile.return_transition import ReturnTransition, validate_handoff


def data():
    return dict(pose=dict(x=1,y=2,yaw=.1),current_pose=dict(x=3,y=4,yaw=.2),map_path='/maps/a.yaml')


def test_preserves_original_home_and_localization_seed():
    g=Mock(); g.map_store.checked_yaml.return_value='/maps/a.yaml'
    t=ReturnTransition(g); payload=data(); t.begin(payload)
    g._launch.assert_called_once_with('navigation','/maps/a.yaml',return_home=payload['pose'])
    assert t.pending['current_pose']==payload['current_pose']
    g.nav.send_goal_async.assert_not_called()


def test_invalid_pose_never_launches():
    g=Mock(); t=ReturnTransition(g); payload=data(); payload['current_pose']['x']=float('nan')
    with pytest.raises(ValueError): t.begin(payload)
    g._launch.assert_not_called()


def test_failed_old_launch_shutdown_never_arms_return():
    g=Mock(); g._launch.side_effect=RuntimeError('old processes remain')
    t=ReturnTransition(g)
    with pytest.raises(RuntimeError): t.begin(data())
    assert t.pending is None


def test_transition_timeout_stops_without_goal():
    g=Mock(); g.mode='navigation'; t=ReturnTransition(g)
    t.pending=data(); t.deadline=0; t.tick()
    assert t.pending is None
    g._service.assert_called_once_with('stop')
    g.nav.send_goal_async.assert_not_called()

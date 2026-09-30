"""Token/freshness checks for the ordinary-navigation recovery client."""
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from std_msgs.msg import String
from rk3576_footbath_exploration.navigation_recovery import NavigationRecoveryClient
from rk3576_footbath_exploration.supervisor import ExplorationSupervisor


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr('rk3576_footbath_exploration.navigation_recovery.time.monotonic',lambda:100.)
    return NavigationRecoveryClient(Mock())


def context(client, **changes):
    value=dict(session='s',token='g',phase='active')
    value.update(changes)
    client.receive(String(data=json.dumps(value)))


def test_active_context_capture_does_not_reuse_recovery_id(client):
    context(client)
    a,b=client.capture(100.),client.capture(100.)
    assert a['token']==b['token']=='g'
    assert a['recovery_id']!=b['recovery_id']
    assert not client.available(101.1)


@pytest.mark.parametrize('body', ['{}','[]','null','no json','{"session":1,"token":"x","phase":"active"}'])
def test_invalid_context_cannot_authorize(client,body):
    client.receive(String(data=body))
    assert not client.available(100.)


def test_old_recovery_ack_cannot_complete_new_handshake(client):
    context(client)
    identity=client.capture(100.)
    context(client,phase='suspended',recovery_id='previous')
    assert client.phase(identity,100.)=='awaiting_owner'
    context(client,phase='suspended',recovery_id=identity['recovery_id'])
    assert client.phase(identity,100.)=='suspended'


@pytest.mark.parametrize('phase',['canceled','failed','succeeded','idle'])
def test_terminal_phase_cannot_resume(client,phase):
    context(client); identity=client.capture(100.)
    context(client,phase=phase,recovery_id=identity['recovery_id'])
    with pytest.raises(ValueError): client.phase(identity,100.)


@pytest.mark.parametrize('key',['token','session'])
def test_replaced_goal_or_process_cannot_resume(client,key):
    context(client); identity=client.capture(100.)
    context(client,**{key:'new'})
    with pytest.raises(ValueError): client.phase(identity,100.)


def supervisor(state, phase):
    n=Mock()
    n._state=state; n.home.phase=phase
    n.RUNNING='running'; n.COMPLETE='complete'; n.TIMED_OUT='timed_out'
    n.PAUSED_OPERATOR='paused_operator'; n.PAUSED_FAULT='paused_fault'
    n.hazard.active=False
    return n


def test_pause_preserves_only_explicit_home_hazard_context():
    n=supervisor('returning_home','hazard_suspended')
    ExplorationSupervisor._pause(n,'hazard_recovery','cliff')
    n.home.interrupt.assert_not_called()
    n._cancel_navigation.assert_not_called()  # HazardRecovery owns cancel/ack ordering.
    n._publish_lease.assert_called_with(False)


@pytest.mark.parametrize('state',['returning_home','navigation_ready','hazard_recovery'])
def test_operator_stop_cancels_navigation_and_recovery_synchronously(state):
    n=supervisor(state,'hazard_suspended')
    n.hazard.active=True
    ExplorationSupervisor._pause(n,'paused_operator','operator')
    n.hazard.cancel.assert_called_once_with('operator')
    n.home.interrupt.assert_called_once_with('operator')
    n._cancel_navigation.assert_called_once()
    n._publish_lease.assert_called_once_with(False)

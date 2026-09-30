from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from rk3576_footbath_mobile.gateway import Gateway

def gateway(**kwargs):
    fields=dict(mode='idle',transitioning=False,child_pgid=None,
                _group_exists=Mock(return_value=False),side_ultrasonic_enabled=True,
                side_ultrasonic_actual=True,side_ultrasonic_seen=123.)
    fields.update(kwargs)
    return SimpleNamespace(**fields)

@pytest.mark.parametrize('mode',['mapping','auto_mapping','navigation'])
def test_running_mode_rejected(mode):
    g=gateway(mode=mode)
    with pytest.raises(ValueError):
        Gateway._set_side_ultrasonic(g,dict(enabled=False,confirm_disabled=True))
    assert g.side_ultrasonic_enabled

@pytest.mark.parametrize('value',[None,0,1,'false',[],{}])
def test_strict_boolean(value):
    with pytest.raises(ValueError):
        Gateway._set_side_ultrasonic(gateway(),dict(enabled=value))

def test_disable_requires_confirmation_and_clears_ack():
    g=gateway()
    with pytest.raises(ValueError): Gateway._set_side_ultrasonic(g,dict(enabled=False))
    Gateway._set_side_ultrasonic(g,dict(enabled=False,confirm_disabled=True))
    assert g.side_ultrasonic_enabled is False
    assert g.side_ultrasonic_actual is None
    assert g.side_ultrasonic_seen == 0
    Gateway._set_side_ultrasonic(g,dict(enabled=True))
    assert g.side_ultrasonic_enabled

def test_old_child_or_transition_rejected():
    for g in (gateway(transitioning=True),gateway(_group_exists=Mock(return_value=True))):
        with pytest.raises(ValueError): Gateway._set_side_ultrasonic(g,dict(enabled=True))

def test_only_valid_feedback_is_accepted():
    g=gateway(side_ultrasonic_actual=None)
    Gateway._side_ultrasonic_state(g,SimpleNamespace(data=2))
    assert g.side_ultrasonic_actual is None
    Gateway._side_ultrasonic_state(g,SimpleNamespace(data=0))
    assert g.side_ultrasonic_actual is False

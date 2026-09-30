import pytest
from rk3576_footbath_exploration.retreat_trace import RetreatEpisode


def budget():
    e=RetreatEpisode();e.begin((0.,0.,0.));e.attempts=2;e.fallbacks=1
    return e


def test_leave_with_progress_then_return_gets_new_episode():
    e=budget();reset=False
    for i in range(21):reset=e.observe(i*.2,(i*.05,0.,0.),True) or reset
    assert reset and e.anchor is None and e.attempts==0 and e.fallbacks==0
    e.begin((0.,0.,0.))
    assert e.attempts==0  # returning within five minutes is a fresh episode


@pytest.mark.parametrize('case',['stationary','turning','oscillation','recovery','unhealthy','jump','short_move'])
def test_no_budget_reset_without_real_departure(case):
    e=budget()
    for i in range(31):
        x=0.;yaw=0.;safe=True
        if case=='turning':yaw=i*.1
        elif case=='oscillation':x=.10 if i%2 else 0.
        elif case=='recovery':x=i*.04;safe=False
        elif case=='unhealthy':x=i*.04;safe=False
        elif case=='jump':x=2. if i>10 else 0.
        elif case=='short_move':x=min(i*.03,.5)
        assert not e.observe(i*.2,(x,0.,yaw),safe)
    assert e.attempts==2 and e.fallbacks==1


def test_new_attempt_does_not_itself_clear_budget():
    e=budget();e.begin((1.,0.,0.))
    assert e.attempts==2 and e.fallbacks==1


def test_telemetry_gap_restarts_departure_observation():
    e=budget()
    for i in range(10):e.observe(i*.2,(i*.05,0.,0.),True)
    assert not e.observe(20.,(1.,0.,0.),True)
    assert e.attempts==2

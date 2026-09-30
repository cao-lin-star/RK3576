import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('trial_progress',Path(__file__).with_name('trial_progress.py'))
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_turning_without_goal_is_not_looping():
    p=module.TrialProgress()
    for t in range(30): p.observe(t,0,0,t*.2)
    assert not p.summary()[2]


def test_single_goal_loop_stops_but_new_goal_resets_window():
    p=module.TrialProgress();p.set_goal('first')
    for t in range(30): p.observe(t,0,0,t*.2)
    assert p.summary()[2]
    p.set_goal('second')
    assert not p.poses
    for t in range(30,40):p.observe(t,0,0,t*.2)
    assert not p.summary()[2]


def test_goal_end_and_translation_are_not_loops():
    p=module.TrialProgress();p.set_goal('first')
    for t in range(30):p.observe(t,t*.03,0,t*.2)
    assert not p.summary()[2]
    p.set_goal(None)
    assert not p.poses

"""Regression tests for the Nav2 startup gate."""

import time
from types import SimpleNamespace

from rk3576_footbath_exploration.supervisor import ExplorationSupervisor


def gate(states):
    node = SimpleNamespace(
        _nav_clients=dict.fromkeys(
            ('planner_server', 'controller_server', 'bt_navigator')),
        _nav_states=states)
    return ExplorationSupervisor._navigation_ready(node)


def test_missing_nodes_block_start():
    assert not gate({})


def test_all_active_allow_start():
    now = time.monotonic()
    assert gate({name: (3, now) for name in (
        'planner_server', 'controller_server', 'bt_navigator')})


def test_inactive_or_expired_node_blocks_start():
    now = time.monotonic()
    states = {name: (3, now) for name in (
        'planner_server', 'controller_server', 'bt_navigator')}
    states['planner_server'] = (2, now)
    assert not gate(states)
    states['planner_server'] = (3, now - 5.0)
    assert not gate(states)

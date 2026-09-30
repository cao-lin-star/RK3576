from unittest.mock import Mock
import pytest
from test_hazard_recovery import rig, finish_reverse

CLEAR=(.155,.16,.40)


def marked(rig):
    h,n,c,step=rig
    step();step(.7);finish_reverse(h,c,step)
    return h,n,c,step


def test_transient_release_failure_keeps_task_and_requires_stability(rig):
    h,n,c,step=marked(rig)
    token=h.owner_token;zones=list(h.zones)
    step(.1,(.27,.16,.4),.03)
    assert h.phase=='marking_recheck' and h.active and n._state=='hazard_recovery'
    assert '左TOF' in n._reason and h.owner_token==token and h.zones==zones
    step(.2,CLEAR,.03);step(.7,CLEAR,.03)
    n._publish_resume.assert_not_called()
    step(.11,CLEAR,.03)
    assert h.phase=='marking'
    h.applied={k:(h.marked_stamp,c[0]) for k in ('local_costmap','global_costmap')}
    step(2.1,CLEAR,.03)
    assert n._state=='running' and not h.active
    n._publish_resume.assert_called_once()


def test_repeated_bad_reading_restarts_stable_window(rig):
    h,n,c,step=marked(rig)
    step(.1,(.27,.16,.4),.03)
    step(.2,CLEAR,.03);step(.7,CLEAR,.03)
    step(.01,(.27,.16,.4),.03);step(.1,CLEAR,.03);step(.7,CLEAR,.03)
    assert h.phase=='marking_recheck'
    n._publish_resume.assert_not_called()
    assert all(call.args[0].linear.x==0 for call in h.cmdpub.publish.call_args_list[-6:])


def sonar_retry(rig):
    h,n,c,step=marked(rig)
    h.kind='sonar';h.sonar_total_limit=.30;h.sonar_reverse_limit=.15
    h.near_original_trace=Mock();h.near_original_trace.plan.return_value=.05
    h.near_ground_history=[];h.near_sources=Mock(return_value=[])
    h.rear_sweep_clear=Mock();h.short_rear_plan=Mock(return_value=.05)
    return h,n,c,step


def test_persistent_near_rechecks_bounded_retreat_without_resetting_origin(rig):
    h,n,c,step=sonar_retry(rig);origin=h.start_odom
    step(.1,(.155,.16,.27),.03)
    step(1.1,(.155,.16,.27),.03)
    assert h.phase=='reversing' and h.sonar_reverse_limit==pytest.approx(.08)
    assert h.start_odom==origin and h.sonar_total_limit==.30
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    h.rear_sweep_clear.assert_called()


def test_missing_retreat_evidence_waits_and_rechecks_without_latching(rig):
    h,n,c,step=sonar_retry(rig)
    h.near_original_trace.plan.side_effect=ValueError('old ground expired')
    h.short_rear_plan.side_effect=ValueError('rear blocked')
    step(.1,(.155,.16,.27),.03);step(1.1,(.155,.16,.27),.03)
    assert h.phase=='marking_recheck' and 'rear blocked' in n._reason
    calls=h.short_rear_plan.call_count
    step(1.,(.155,.16,.27),.03)
    assert h.short_rear_plan.call_count==calls
    h.short_rear_plan.side_effect=None
    step(2.1,(.155,.16,.27),.03)
    assert h.phase=='reversing'


def test_exhausted_distance_waits_but_can_resume_when_clear(rig):
    h,n,c,step=sonar_retry(rig)
    step(.1,(.155,.16,.27),.30);step(1.1,(.155,.16,.27),.30)
    assert h.phase=='marking_recheck' and '额度已用完' in n._reason
    h.near_original_trace.plan.assert_not_called()
    n.home.current_pose.return_value.pose.position.x=-.30
    step(.1,CLEAR,.30);step(.9,CLEAR,.30)
    assert h.phase=='marking'


def test_late_costmap_ack_recovers_without_operator(rig):
    h,n,c,step=marked(rig);h.applied={}
    step(6.,CLEAR,.03)
    assert h.phase=='marking' and h.active
    h.applied={k:(h.marked_stamp,c[0]) for k in ('local_costmap','global_costmap')}
    step(2.1,CLEAR,.03)
    assert n._state=='running'


def test_data_loss_during_recheck_preserves_stage_and_owner(rig):
    h,n,c,step=marked(rig)
    step(.1,(.27,.16,.4),.03)
    h.bridge_connected=False
    step(1.,CLEAR,.03)
    assert h.health_wait_at is not None and h.phase=='marking_recheck'
    h.bridge_connected=True
    step(.1,CLEAR,.03);step(2.1,CLEAR,.03)
    assert h.health_wait_at is None and h.phase=='marking_recheck'
    n._publish_resume.assert_not_called()


def test_estop_during_recheck_never_auto_resumes(rig):
    h,n,c,step=marked(rig)
    step(.1,(.27,.16,.4),.03)
    h.fault=256
    step(.1,CLEAR,.03)
    assert h.phase=='failed' and not h.active
    n._publish_resume.assert_not_called()


def test_mapping_wait_does_not_consume_exploration_budget(rig):
    h,n,c,step=marked(rig);n._exploration_started_at=900.
    step(.1,(.27,.16,.4),.03)
    old=n._exploration_started_at
    step(5.,(.27,.16,.4),.03)
    assert n._exploration_started_at==pytest.approx(old+5.)
    assert h.phase=='marking_recheck'


def test_rear_protection_changes_during_short_retry_return_to_wait(rig):
    h,n,c,step=sonar_retry(rig)
    step(.1,(.155,.16,.27),.03);step(1.1,(.155,.16,.27),.03)
    assert h.phase=='reversing'
    h.rear_observed.return_value=False
    step(.1,(.155,.16,.27),.03)
    assert h.phase=='marking_recheck' and h.active
    assert '后方雷达' in n._reason
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    assert h.sonar_total_limit==.30


def test_short_retry_does_not_refill_motion_time_budget(rig):
    h,n,c,step=sonar_retry(rig)
    h.reverse_at=c[0]-20.
    step(.1,(.155,.16,.27),.03);step(1.1,(.155,.16,.27),.03)
    assert h.phase=='marking_recheck' and '额度已用完' in n._reason
    h.near_original_trace.plan.assert_not_called()

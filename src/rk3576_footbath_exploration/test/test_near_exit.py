"""Near recovery boundary, source-aware clearance, and extension permission."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
spec=importlib.util.spec_from_file_location('exit_fixture',Path(__file__).with_name('test_hazard_recovery.py'))
f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)
rig=f.rig


def recovery(rig, source='side_left', distance=.374):
    h,n,clock,step=rig
    h.kind='sonar';h.owner='exploration';h.active=True;n._state='hazard_recovery'
    h.trigger_sources=[source]
    h.pending=[dict(x=distance,y=0.,radius=.04,kind='near',expires=0.,source=source)]
    h.zones=list(h.pending);h.near_recorded=True
    h.start_near_reverse(clock[0]);h.near_sources=lambda now:[]
    return h,n,clock,step


@pytest.mark.parametrize('source,front,expected',[
    ('side_left',.30,True),('side_right',.30,True),('front',.30,True),
    ('front',.35,True),('side_left',.279,False)])
def test_source_aware_resume(rig,source,front,expected):
    h,n,c,step=recovery(rig,source)
    assert h.clear_for_resume((.16,.16,front),c[0]) is expected


def test_combined_sources_require_release_threshold_and_all_points_clear(rig):
    h,n,c,step=recovery(rig)
    h.trigger_sources=['front','side_left','side_right']
    assert h.clear_for_resume((.16,.16,.30),c[0])
    h.pending.append(dict(x=.25,y=0.,radius=.04,source='side_right'))
    assert not h.clear_for_resume((.16,.16,.5),c[0])
    assert '右侧' in h.resume_detail


@pytest.mark.parametrize('progress',[.145,.15,.153])
def test_clear_at_distance_boundary_waits_and_applies_costmaps(rig,progress):
    h,n,c,step=recovery(rig)
    step(.1,(.16,.16,.8),progress)
    assert h.phase=='reversing' and h.clear_since is not None, n._reason
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    step(.6,(.16,.16,.8),progress)
    assert h.phase=='marking'
    n._publish_resume.assert_not_called()
    h.applied={k:(h.marked_stamp,c[0]) for k in ('local_costmap','global_costmap')}
    step(2.1,(.16,.16,.8),progress)
    n._publish_resume.assert_called_once()


def test_extension_waits_stopped_then_checks_ground_and_rear(rig):
    h,n,c,step=recovery(rig,distance=.25)
    h.near_original_trace.plan=Mock(side_effect=ValueError('not straight'))
    h.short_rear_plan=Mock(return_value=.05)
    step(.1,(.16,.16,.8),.15)
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    h.short_rear_plan.assert_not_called()
    n.home.last_motion=c[0]
    step(.2,(.16,.16,.8),.15)
    h.short_rear_plan.assert_not_called()
    step(.3,(.16,.16,.8),.15)
    assert h.sonar_reverse_limit==pytest.approx(.20)
    assert h.short_rear_plan.call_args.args[2] is h.near_ground_history
    assert h.cmdpub.publish.call_args.args[0].linear.x==0


@pytest.mark.parametrize('failure',['rear','ground','manual','tof','new_side','total','timeout','rear_stale'])
def test_no_motion_when_extension_unsafe(rig,failure):
    h,n,c,step=recovery(rig,distance=.25)
    h.near_original_trace.plan=Mock(return_value=.06)
    h.rear_sweep_clear=Mock()
    h.short_rear_plan=Mock(return_value=.05)
    step(.1,(.16,.16,.8),.15)
    if failure=='rear':
        h.rear_sweep_clear.side_effect=ValueError('rear blocked')
        h.short_rear_plan.side_effect=ValueError('rear blocked')
    elif failure=='ground':
        h.near_original_trace.plan.side_effect=ValueError('no old trace')
        h.short_rear_plan.side_effect=ValueError('no known ground')
    elif failure=='rear_stale':h.rear_observed=Mock(return_value=False)
    elif failure=='manual':h._manual(NS())
    elif failure=='tof':
        step(.1,(.29,.16,.8),.15)
    elif failure=='new_side':h.near_sources=lambda now:['side_right']
    elif failure=='total':h.sonar_total_limit=.15
    elif failure=='timeout':h.reverse_at=c[0]-21.
    if h.active:step(.5,(.16,.16,.8),.15)
    assert not h.active and h.phase=='failed'
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    n._publish_resume.assert_not_called()


def test_ground_snapshot_cannot_be_extended_by_its_own_reverse(rig):
    h,n,c,step=recovery(rig,distance=.25)
    h.ground_history.append((c[0],-.20,0.))
    assert h.near_ground_history==[]


def test_manual_and_navigation_owners_keep_their_resume_path(rig):
    h,n,c,step=recovery(rig)
    h.owner='navigation'
    assert h.clear_for_resume((.16,.16,.30),c[0])
    h.owner='home_return'
    assert h.clear_for_resume((.16,.16,.30),c[0])


@pytest.mark.parametrize('front,expected',[(.2799,False),(.28,True),(.288,True),(.35,True)])
def test_front_hard_release_boundary(rig,front,expected):
    h,n,c,step=recovery(rig,source='front')
    assert h.clear_for_resume((.16,.16,front),c[0]) is expected


def test_front_288_at_segment_limit_resumes_without_extra_ground(rig):
    h,n,c,step=recovery(rig,source='front')
    h.near_original_trace.plan=Mock(side_effect=ValueError('no history'))
    h.short_rear_plan=Mock(side_effect=ValueError('no ground'))
    h.rear_observed=Mock(return_value=False)
    # Rear permission is not needed to remain stopped and hand back to Nav2.
    step(.1,(.16,.16,.288),.15)
    assert h.phase=='reversing' and h.clear_since is not None
    assert h.cmdpub.publish.call_args.args[0].linear.x==0
    step(.6,(.16,.16,.288),.15)
    assert h.phase=='marking'
    h.applied={k:(h.marked_stamp,c[0]) for k in ('local_costmap','global_costmap')}
    step(2.1,(.16,.16,.288),.15)
    n._publish_resume.assert_called_once()
    h.short_rear_plan.assert_not_called()


@pytest.mark.parametrize('available',[False,True])
def test_margin_is_optional_and_requires_preexisting_ground_and_rear(rig,available):
    h,n,c,step=recovery(rig,source='front')
    h.near_original_trace.plan=Mock(return_value=.06)
    h.rear_sweep_clear=Mock(side_effect=None if available else ValueError('rear blocked'))
    step(.1,(.16,.16,.30),.05)
    assert h.phase=='reversing'
    assert h.cmdpub.publish.call_args.args[0].linear.x==(-.03 if available else 0.)


def test_front_28_does_not_bypass_collision_clearance(rig):
    h,n,c,step=recovery(rig,source='front',distance=.25)
    assert not h.clear_for_resume((.16,.16,.288),c[0])
    assert '障碍距车体中心' in h.resume_detail

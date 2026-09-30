"""Voice frames and navigation safety tests; no physical ports or robot motion."""
import json
import os
from pathlib import Path
import queue
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rk3576_footbath_mobile.voice_serial import Decoder, Protocol, VoiceSerial
from rk3576_footbath_mobile.voice_control import VoiceControl

CONFIG = Path(__file__).parents[1]/'config/voice_protocol.json'


def test_workbook_protocol():
    p = Protocol(CONFIG)
    assert len(p.speech)==57
    assert p.commands[bytes.fromhex('BB 01 00 66')]=='妮妮'
    assert p.speech[bytes.fromhex('AA 03 09 55')]=='已返回起点并停车'


def test_split_noise_glued_frames_and_single_wake():
    d=Decoder()
    assert d.feed(b'noise\xbb\x01',10)==[]
    assert d.feed(b'\x00\x66\xbb\x01\x02\x66\xbb\x01\x01\x66',10.1)==['start']
    assert len(d.buffer)<4


@pytest.mark.parametrize('data',[b'\xbb\x01\x01\x66',b'BB 01 01 66',b'\xaa\x03\x05\x55',b'\xbb\x01\x03\x66'])
def test_unwoken_unknown_and_ascii_do_not_move(data):
    assert Decoder().feed(data,10)==[]


def test_wake_expiry_and_partial_timeout():
    d=Decoder();d.feed(bytes.fromhex('BB 01 00 66'),1)
    assert d.feed(bytes.fromhex('BB 01 01 66'),17)==[]
    d.feed(bytes.fromhex('BB 01 00 66'),20)
    d.feed(bytes.fromhex('BB 01'),21)
    assert d.feed(bytes.fromhex('01 66'),22)==[]


def voice():
    now=time.monotonic()
    v=VoiceControl.__new__(VoiceControl)
    g=SimpleNamespace(mode='navigation',mode_started_at=now-20,transitioning=False,child=Mock(),manual_active=False,
        control_source=0,control_source_at=now,pose_map={'x':1.,'y':2.,'yaw':.3},healthy=Mock(return_value=True),
        home_seen=now,home_status={'phase':'navigation_ready','available':True,'dock_enabled':True},
        pending_departure_goal=None,return_requested=False,exp_clients={'return_home':Mock(),'return_start':Mock()},
        cancel_client=Mock(),_invalidate_navigation=Mock(),_service=Mock(),_goal=Mock(),
        nav_owner=SimpleNamespace(token='token'),get_logger=Mock())
    g.child.poll.return_value=None
    v.g=g;v.io=Mock();v.enabled=True;v.pending=None;v.intent=None;v.intent_token=None
    v.diag_at=v.base_at=v.odom_at=v.status_at=now
    v.last_motion=now-2
    v.diag={'state':'running','fault_latched':'False'}
    v.base={'connected':'true','heartbeat_age_s':'.1','fault_flags':'0','sensor_fault_flags':'0'}
    v.start_pose={'x':0.,'y':0.,'yaw':0.};v.pending_capture=None;v.capture_pending=False
    v.active_goals=False
    return v,now


@pytest.mark.parametrize('field,value',[('manual_active',True),('control_source',2),('control_source_at',0),('pose_map',None),('transitioning',True),('mode','idle')])
def test_unsafe_request_rejected(field,value):
    v,now=voice();setattr(v.g,field,value)
    with pytest.raises(ValueError):v.request('dock',now)
    v.g._invalidate_navigation.assert_not_called()


@pytest.mark.parametrize('fault',['512','1024','2048'])
def test_voice_does_not_clear_mcu_fault(fault):
    v,now=voice();v.base['fault_flags']=fault
    with pytest.raises(ValueError):v.request('dock',now)


def test_start_waits_for_cancel_status_and_standstill():
    v,now=voice();v.request('start',now)
    p=v.pending;p['future'].done.return_value=True;p['future'].result.return_value=SimpleNamespace(return_code=0)
    v.status_at=now-1
    v.tick_pending(now+.1);v.g._goal.assert_not_called()
    v.status_at=now+.2;v.active_goals=True
    v.tick_pending(now+.2);v.g._goal.assert_not_called()
    v.active_goals=False;v.odom_at=now+.3
    v.tick_pending(now+.3)
    v.g._goal.assert_called_once_with({'x':0.,'y':0.,'yaw':0.})
    assert v.intent=='start'


def test_missing_start_does_not_fall_back_to_dock():
    v,now=voice();v.start_pose=None;v.request('start',now)
    v.io.say.assert_called_once_with(3,11,False)
    v.g._invalidate_navigation.assert_not_called()


def test_mapping_start_uses_distinct_service():
    v,now=voice();v.g.mode='auto_mapping';v.request('start',now)
    v.g.exp_clients['return_start'].call_async.assert_called_once()
    v.g.exp_clients['return_home'].call_async.assert_not_called()


def test_dock_uses_existing_return_service_and_no_goal():
    v,now=voice();v.request('dock',now)
    v.g.exp_clients['return_home'].call_async.assert_called_once()
    v.g._goal.assert_not_called()
    v.request('dock',now+.1)
    assert v.g.exp_clients['return_home'].call_async.call_count==1


def test_capture_before_departure_not_after():
    v,now=voice();v.before_goal()
    v.g.pose_map={'x':9.,'y':9.,'yaw':1.};v.before_goal();v.goal_accepted()
    assert v.start_pose=={'x':1.,'y':2.,'yaw':.3}
    v.intent='start';v.before_goal();v.goal_accepted()
    assert v.start_pose=={'x':1.,'y':2.,'yaw':.3}


def test_moving_new_departure_invalidates_old_start():
    v,now=voice();v.last_motion=now;v.before_goal();v.goal_accepted()
    assert v.start_pose is None


def test_real_pty_transport_round_trip():
    import pty
    import select
    master,slave=pty.openpty()
    transport=VoiceSerial(os.ttyname(slave),Protocol(CONFIG))
    try:
        deadline=time.monotonic()+3
        while not transport.connected and time.monotonic()<deadline:time.sleep(.02)
        assert transport.connected
        os.write(master,bytes.fromhex('BB 01 00 66 BB 01 02 66'))
        assert transport.incoming.get(timeout=2)[0]=='start'
        transport.say(3,9)
        assert select.select([master],[],[],3)[0]
        assert os.read(master,4)==bytes.fromhex('AA 03 09 55')
    finally:
        transport.close();os.close(master);os.close(slave)

def test_start_handoff_disables_dock_without_changing_target():
    from rk3576_footbath_mobile.return_transition import ReturnTransition
    g=Mock();g.map_store.checked_yaml.return_value='/maps/a.yaml'
    data=dict(pose=dict(x=1,y=2,yaw=.1),current_pose=dict(x=3,y=4,yaw=.2),map_path='/maps/a.yaml',return_kind='start')
    t=ReturnTransition(g);t.begin(data)
    g._launch.assert_called_once_with('navigation','/maps/a.yaml',return_home=data['pose'],return_dock=False)
    assert t.pending['pose']==data['pose']


def test_mapping_start_request_uses_original_pose_and_save_flow():
    from rk3576_footbath_exploration.home_return import HomeReturn
    h=HomeReturn.__new__(HomeReturn)
    h.localization=False;h.phase='ready';h.pose=object();original=h.pose
    h.request=Mock(return_value=(True,'accepted'))
    response=SimpleNamespace(success=False,message='')
    h._return_start(None,response)
    assert response.success and h.return_kind=='start' and h.pose is original


def test_mapping_start_refuses_busy_or_localization():
    from rk3576_footbath_exploration.home_return import HomeReturn
    h=HomeReturn.__new__(HomeReturn);h.localization=True;h.phase='ready';h.request=Mock()
    response=SimpleNamespace(success=True,message='')
    h._return_start(None,response)
    assert not response.success
    h.request.assert_not_called()


def test_expired_supervisor_blocks_voice():
    v,now=voice();v.diag_at=now-3
    with pytest.raises(ValueError):v.request('dock',now)


def test_cancel_does_not_rewrite_saved_start():
    v,now=voice();v.before_goal();v.cancel()
    assert v.start_pose=={'x':0.,'y':0.,'yaw':0.}
    assert v.pending_capture is None and not v.capture_pending

def observable():
    v,now=voice();g=v.g
    g.mode_started_at=now-20;g.grid=object();g.initialized=True;g.launch_error='';g.nav_state='idle'
    g.side_ultrasonic_enabled=True
    v.previous={};v.planning_since=None;v.us_at=now
    v.us=[{'status':2,'age_ms':10,'distance_m':4.} for _ in range(3)]
    v.diag.update(hazard_phase='idle',save_state='idle',reason='health gate passed')
    return v,now


def test_dock_arrival_never_announces_exploration_complete():
    v,now=observable();v.g.home_status.update(phase='arrived');v.diag.update(state='complete',reason='已返回基站')
    v.observe(now)
    frames=[c.args[:2] for c in v.io.say.call_args_list]
    assert (3,7) in frames and (2,4) not in frames


def test_stale_canceled_home_does_not_erase_start_goal_intent():
    v,now=observable();v.intent='start';v.intent_token='token'
    v.g.nav_state='active';v.g.home_status['phase']='canceled'
    v.observe(now)
    assert v.intent=='start'
    assert (3,8) in [c.args[:2] for c in v.io.say.call_args_list]


def test_no_echo_is_not_sonar_fault_and_valid_side_hit_announced():
    v,now=observable();v.observe(now)
    assert (5,2) not in [c.args[:2] for c in v.io.say.call_args_list]
    v.us[1]={'status':1,'age_ms':10,'distance_m':.12};v.observe(now+.01)
    assert (4,2) in [c.args[:2] for c in v.io.say.call_args_list]


def test_no_arrival_announcement_without_fresh_stopped_odom():
    v,now=observable();v.g.nav_state='succeeded';v.odom_at=now-1
    v.observe(now)
    assert (3,2) not in [c.args[:2] for c in v.io.say.call_args_list]


def test_no_zero_velocity_guess_for_frontier_selection():
    v,now=observable();v.g.mode='auto_mapping';v.diag['explore_status']='not_received'
    v.planning_since=now-10;v.observe(now)
    assert (2,3) not in [c.args[:2] for c in v.io.say.call_args_list]


@pytest.mark.parametrize('kind',['escape','health_wait','tof_invalid'])
def test_non_obstacle_recovery_never_announces_front_near(kind):
    v,now=observable();v.diag.update(hazard_phase='cancelling',hazard_kind=kind,hazard_near_sources='front')
    v.observe(now)
    assert not any(c.args[:2] in [(4,1),(4,2),(4,3)] for c in v.io.say.call_args_list)


def test_both_sides_do_not_claim_front_obstacle():
    v,now=observable()
    for i in (1,2):v.us[i]={'status':1,'age_ms':10,'distance_m':.10}
    v.observe(now)
    frames=[c.args[:2] for c in v.io.say.call_args_list]
    assert (4,1) not in frames and (4,2) in frames


def test_coverage_shortage_is_not_reported_as_front_obstacle_or_broken_lidar():
    v,now=observable();v.diag.update(state='paused_fault',hazard_phase='failed',hazard_kind='escape',reason='多帧雷达覆盖不足，曲线退路不可用')
    v.observe(now)
    frames=[c.args[:2] for c in v.io.say.call_args_list]
    assert (4,1) not in frames and (5,1) not in frames and (4,8) in frames

def test_mapping_command_requires_wake_and_is_single_use():
    d=Decoder()
    assert d.feed(bytes.fromhex('BB 01 03 66'),1)==[]
    assert d.feed(bytes.fromhex('BB 01 00 66 BB 01 03 66 BB 01 03 66'),2)==['mapping']


def test_mapping_starts_only_from_idle_through_normal_launch():
    v,now=voice();v.g._launch=Mock();v.g.mode='idle';v.g.child=None
    v.request('mapping',now)
    v.g._launch.assert_called_once_with('auto_mapping')
    v.g._service.assert_not_called()


@pytest.mark.parametrize('mode',['navigation','mapping','auto_mapping'])
def test_mapping_command_does_not_interrupt_existing_task(mode):
    v,now=voice();v.g.mode=mode;v.g._launch=Mock()
    v.request('mapping',now)
    v.g._launch.assert_not_called()
    v.io.say.assert_called_with(5,16,False)


def test_recovery_does_not_announce_new_mapping_or_fake_obstacle():
    v,now=observable();v.g.mode='auto_mapping';v.observe(now)
    v.diag.update(state='hazard_recovery',hazard_phase='resuming_owner',hazard_kind='health_wait')
    v.observe(now+.01);v.io.say.reset_mock()
    v.diag.update(state='running',hazard_phase='idle');v.observe(now+.02)
    frames=[c.args[:2] for c in v.io.say.call_args_list]
    assert (2,14) in frames and (2,1) not in frames and (4,7) not in frames


def test_unreachable_exploration_completion_is_announced():
    v,now=observable();v.g.mode='auto_mapping'
    v.diag.update(state='complete',explore_status='exploration_complete_with_unreachable')
    v.observe(now)
    assert (2,15) in [c.args[:2] for c in v.io.say.call_args_list]


def test_committed_obstacle_count_announces_once():
    v,now=observable();v.diag.update(obstacle_count='0',hazard_kind='sonar',hazard_phase='confirming');v.observe(now)
    v.io.say.reset_mock();v.diag['obstacle_count']='1';v.observe(now+.01);v.observe(now+.02)
    assert [c.args[:2] for c in v.io.say.call_args_list].count((4,7))==1


def test_repeated_info_has_cooldown():
    v,now=observable();v.say(2,3);v.say(2,3)
    assert v.io.say.call_count==1


@pytest.mark.parametrize('field,value',[('transitioning',True),('manual_active',True),('return_requested',True)])
def test_busy_idle_does_not_start_mapping(field,value):
    v,now=voice();v.g.mode='idle';v.g.child=None;v.g._launch=Mock()
    setattr(v.g,field,value);v.request('mapping',now)
    v.g._launch.assert_not_called()


def test_recovered_speech_waits_for_stable_clear():
    v,now=observable();v.previous['had_fault']=True
    v.observe(now);v.base_at=v.diag_at=v.us_at=now+1;v.observe(now+1)
    assert (5,7) not in [c.args[:2] for c in v.io.say.call_args_list]
    v.base_at=now+2.1;v.diag_at=now+2.1;v.us_at=now+2.1
    v.observe(now+2.1)
    assert (5,7) in [c.args[:2] for c in v.io.say.call_args_list]

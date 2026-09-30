"""Workbook-driven voice events and guarded return requests in the ROS thread."""
import json
import math
import queue
import time
from pathlib import Path

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from .voice_serial import Protocol, VoiceSerial


class VoiceControl:
    def __init__(self, gateway, cfg, share):
        self.g = gateway
        self.io = VoiceSerial(cfg.get('FOOTBATH_VOICE_PORT', '/dev/footbath_voice'),
                              Protocol(Path(share)/'config/voice_protocol.json'))
        self.enabled = cfg.get('FOOTBATH_VOICE_CONTROL', '1') == '1'
        self.previous = {}
        self.speech_times = {}
        self.mapping_announced = False
        self.recovered_since = None
        self.start_pose = None
        self.pending_capture = None
        self.capture_pending = False
        self.capture_token = None
        self.pending = None
        self.intent = None
        self.intent_token = None
        self.status_at = self.odom_at = self.diag_at = self.base_at = 0.
        self.active_goals = False
        self.last_motion = time.monotonic()
        self.diag = {}
        self.base = {}
        self.us = None
        self.us_at = 0.
        self.planning_since = None
        self.connected_before = False
        self.mode_key = None
        self.last_error = ''
        gateway.create_subscription(Odometry, '/odom', self.odom, 20)
        gateway.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
        from std_msgs.msg import String
        gateway.create_subscription(String, '/ultrasonic/status', self.ultrasonic, 10)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        gateway.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, qos)
        gateway.create_timer(.1, self.tick)

    def ultrasonic(self, msg):
        try:
            readings = json.loads(msg.data)['readings']
            if len(readings) == 3:
                self.us, self.us_at = readings, time.monotonic()
        except (KeyError, ValueError, TypeError):
            pass

    def status_view(self):
        return dict(enabled=True, connected=self.io.connected, control_enabled=self.enabled,
                    port=self.io.device, error=self.last_error or self.io.error,
                    start_available=self.start_pose is not None,
                    pending=self.pending is not None)

    def odom(self, msg):
        now = time.monotonic()
        age = (self.g.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if not all(math.isfinite(x) for x in (age,v,w)) or not 0 <= age <= .6:
            return
        self.odom_at = now
        if abs(v) > .01 or abs(w) > .02:
            self.last_motion = now

    def stopped(self, now):
        return now-self.odom_at <= .6 and now-self.last_motion >= .5

    def status(self, msg):
        self.status_at = time.monotonic()
        self.active_goals = any(s.status in (1, 2, 3) for s in msg.status_list)

    def diagnostics(self, msg):
        now = time.monotonic()
        age = (self.g.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if not 0 <= age <= 2.:
            return
        for item in msg.status:
            values = {v.key:v.value for v in item.values}
            if item.name == 'rk3576_footbath/exploration_supervisor':
                self.diag, self.diag_at = values, now
            elif item.name == 'rk3576_footbath/stm32_serial_bridge':
                self.base, self.base_at = values, now

    def say(self, group, item, urgent=False):
        # Limit repeated informational/recovery speech without delaying a first alarm.
        now = time.monotonic()
        times = getattr(self, 'speech_times', {})
        code = (group, item)
        limited = code in ((2,3),(2,14),(2,16),(4,4),(4,6),(4,11),(4,12),(4,13),(5,2),(5,3),(5,8))
        if limited and now-times.get(code, -100.) < 25.:
            return
        times[code] = now
        self.speech_times = times
        self.io.say(group, item, urgent)

    def edge(self, key, value, code, condition=True):
        old = self.previous.get(key)
        if old != value and condition:
            self.previous[key] = value
            if code:
                if key in ('nav','home','supervisor'):
                    self.io.clear()
                self.say(*code, urgent=code[0] in (4,5))

    def mode_reset(self):
        self.io.clear()
        self.pending = None
        self.previous.clear()
        self.speech_times = {}
        self.mapping_announced = False
        self.recovered_since = None
        self.diag, self.base = {}, {}
        self.diag_at = self.base_at = 0.
        self.start_pose = self.pending_capture = None
        self.capture_pending = False
        self.capture_token = None
        self.intent = self.intent_token = None
        self.mode_key = (self.g.mode, self.g.mode_started_at)
        if self.g.mode == 'navigation': self.say(2, 10)

    def before_goal(self):
        if self.intent == 'start' or self.capture_pending or self.g.pending_departure_goal is not None:
            return
        self.capture_pending = True
        now = time.monotonic()
        if self.stopped(now) and self.g.pose_map is not None:
            self.pending_capture = dict(self.g.pose_map)
        else:
            self.pending_capture = None

    def goal_accepted(self):
        if self.capture_pending:
            self.start_pose = dict(self.pending_capture) if self.pending_capture is not None else None
            self.pending_capture = None
            self.capture_pending = False
        if self.intent == 'start':
            self.intent_token = self.g.nav_owner.token

    def cancel(self):
        self.pending = None
        self.pending_capture = None
        self.capture_pending = False
        self.intent = self.intent_token = None
        self.io.clear()

    def guard(self, now):
        g = self.g
        if g.mode not in ('mapping','auto_mapping','navigation') or g.transitioning or g.child is None or g.child.poll() is not None:
            raise ValueError('尚未启动有效建图或导航会话')
        if g.manual_active or g.control_source is None or g.control_source >= 2 or now-g.control_source_at > .8:
            raise ValueError('手动接管或控制来源未就绪')
        if g.pose_map is None or not all(g.healthy(k) for k in ('scan','odom','map')):
            raise ValueError('地图、雷达或定位未就绪')
        if now-self.diag_at > 2. or now-self.base_at > 2.:
            raise ValueError('监督器或底盘状态过期')
        if self.diag.get('state') in ('paused_fault','startup_timeout','hazard_recovery') or self.diag.get('fault_latched','False').lower() == 'true':
            raise ValueError('故障或危险恢复中，语音不能解除保护')
        if int(self.base.get('sensor_fault_flags','0')) & 0x3e:
            raise ValueError('底盘测距传感器异常')
        if int(self.base.get('fault_flags','0')) & ~1:
            raise ValueError('底盘存在故障，语音不能解除保护')
        if self.base.get('connected') != 'true' or float(self.base.get('heartbeat_age_s','99')) > .8:
            raise ValueError('底盘通信未就绪')
        if now-g.home_seen > 2.:
            raise ValueError('返航监督器未就绪')

    def start_mapping(self):
        g = self.g
        # Idle has no live sensor session. The normal launch supervisor owns
        # startup health gates; never bypass them or replace an existing task.
        if (g.mode != 'idle' or g.transitioning or g.manual_active or
                g.return_requested or self.pending or self.intent or
                (g.child is not None and g.child.poll() is None)):
            self.say(5, 16)
            return
        g._launch('auto_mapping')

    def request(self, kind, now):
        if not self.enabled:
            return
        if kind == 'mapping':
            self.start_mapping()
            return
        if kind not in ('dock', 'start'):
            raise ValueError('未知语音指令')
        if self.pending or self.intent or self.g.return_requested:
            return
        self.guard(now)
        g = self.g
        if g.home_status.get('phase') in ('undocking','docking','dock_preparing','dock_waiting','aligning','hazard_suspended'):
            raise ValueError('出入站或危险恢复中不能切换任务')
        if kind == 'dock':
            if not g.home_status.get('available') or not g.home_status.get('dock_enabled'):
                self.say(3,10); return
            service = 'return_home'
        elif g.mode in ('mapping','auto_mapping'):
            if not g.home_status.get('available'):
                self.say(3,11); return
            service = 'return_start'
        else:
            if self.start_pose is None:
                self.say(3,11); return
            service = None
        if service:
            client = g.exp_clients[service]
            if not client.service_is_ready(): raise ValueError('返航服务未就绪')
            # Existing supervisor owns cancellation, standstill and map handoff.
            g._invalidate_navigation('语音请求返航')
            self.intent = kind
            self.pending = dict(kind=kind, stage='service', future=client.call_async(Trigger.Request()), deadline=now+4)
            return
        if not g.cancel_client.service_is_ready(): raise ValueError('导航取消服务未就绪')
        # Start return uses the normal navigation owner, never dock reverse.
        target = dict(self.start_pose)
        g._invalidate_navigation('语音请求返回最近出发位置')
        g._service('stop')
        self.intent = 'start'
        self.pending = dict(kind='start',stage='cancel',target=target,
                            future=g.cancel_client.call_async(CancelGoal.Request()),at=now,deadline=now+6)

    def tick_pending(self, now):
        p = self.pending
        if not p: return
        self.guard(now)
        if now > p['deadline']: raise ValueError('语音返航等待超时')
        if not p['future'].done(): return
        result = p['future'].result()
        if p['stage'] == 'service':
            if result is None or not result.success: raise ValueError(result.message if result else '返航服务无响应')
            self.g.return_requested = True
            self.g.return_requested_at = now
            self.pending = None
            self.say(3,5 if p['kind']=='dock' else 8)
        else:
            if result is None or result.return_code not in (0,2): raise ValueError('旧导航取消失败')
            if self.active_goals or self.status_at < p['at'] or not self.stopped(now): return
            self.pending = None
            self.g._goal(p['target'])
            self.intent_token = self.g.nav_owner.token

    def observe(self, now):
        g = self.g
        if g.mode in ('mapping','auto_mapping'):
            self.edge('launch_error',g.launch_error,(5,17) if g.launch_error else None)
        if g.mode == 'idle': return
        stopped = self.stopped(now)
        if g.mode == 'navigation':
            self.edge('map', bool(g.grid is not None), (2,11) if g.grid is not None else None)
            self.edge('localized', g.initialized, (2,12) if g.initialized else None)
            self.edge('launch_error',g.launch_error,(2,13) if g.launch_error else None)
        ready = not g.transitioning and all(g.healthy(k) for k in ('scan','map','odom')) and (g.initialized or g.mode!='navigation')
        self.edge('ready',ready,(1,2) if ready else None)
        self.edge('manual',g.manual_active or (g.control_source is not None and g.control_source>=2),(1,5) if g.manual_active or (g.control_source is not None and g.control_source>=2) else None)
        nav = g.nav_state
        start_return = self.intent == 'start' and self.intent_token == g.nav_owner.token
        codes = {'no_path':(3,3),'active':(3,8) if start_return else (3,1),'succeeded':(3,9) if start_return else (3,2),
                 'aborted':(3,4),'failed':(3,4),'error':(3,4),'rejected':(3,4),'canceled':(1,4)}
        self.edge('nav',nav,codes.get(nav),nav=='active' or stopped)
        if nav in ('succeeded','canceled','aborted','failed','error','rejected','no_path') and stopped and not self.pending:
            if self.intent_token is not None: self.intent = self.intent_token = None
        home = g.home_status
        if now-g.home_seen < 2.:
            phase = home.get('phase')
            hc = {'dock_preparing':(3,6),'arrived':(3,7) if home.get('dock_enabled') else (3,9),'failed':(3,12)}
            self.edge('home',phase,hc.get(phase),phase=='dock_preparing' or stopped)
            if phase in ('arrived','failed') and stopped and not self.pending and self.intent_token is None:
                self.intent = self.intent_token = None
        if now-self.base_at > 2. and now-g.mode_started_at>15:
            self.edge('base_connected',False,(5,5))
        if now-self.diag_at > 2.: return
        d = self.diag
        state, reason = d.get('state'), d.get('reason','')
        codes = {'waiting_for_health':(2,2),'running':(2,1),'startup_timeout':(5,17),
                 'exploration_timeout':(2,6),'paused_operator':(1,3)}
        code = codes.get(state)
        if state == 'running' and getattr(self, 'mapping_announced', False):
            code = (2,14)
        if state == 'running' and self.previous.get('supervisor') in ('paused_operator','paused_fault'):
            code = (1,6)
        if state == 'running' and g.mode in ('mapping','auto_mapping'):
            self.mapping_announced = True
        if g.mode == 'navigation' and state == 'running': code = None
        if state == 'paused_fault':
            if d.get('hazard_kind')=='escape' and any(word in reason for word in ('覆盖不足','曲线','退路')): code=(4,8)
            elif '无实际进展' in reason: code=(2,5)
            elif 'TF' in reason or '定位' in reason: code=(5,4)
            elif 'scan' in reason or '雷达' in reason or not g.healthy('scan'): code=(5,1)
            elif '控制来源' in reason: code=(5,6)
        wait_ok = state!='waiting_for_health' or now-g.mode_started_at>3
        self.edge('supervisor',state,code,wait_ok and (state=='running' or stopped))
        save = d.get('save_state','idle')
        sc = (2,8) if save=='complete' else ((2,7) if save in ('saving_map','saving_occupancy_map','serializing_pose_graph','saving') else ((2,9) if 'fail' in save or 'unavailable' in save else None))
        self.edge('save',save,sc)
        if g.mode in ('mapping','auto_mapping'):
            self.edge('explore_complete', d.get('explore_status'), {'exploration_complete':(2,4), 'exploration_complete_with_unreachable':(2,15)}.get(d.get('explore_status')), stopped)
            self.edge('reevaluating', d.get('explore_status')=='reevaluating_frontier', (2,16) if d.get('explore_status')=='reevaluating_frontier' else None)
            selecting = (state=='running' and d.get('explore_status')=='selecting_frontier' and not self.active_goals and now-self.status_at<2 and stopped)
            if selecting:
                if self.planning_since is None: self.planning_since=now
                if now-self.planning_since>=3: self.edge('selecting',True,(2,3))
            else:
                self.planning_since=None
                self.previous['selecting']=False
        phase = d.get('hazard_phase','idle')
        hc={'confirming':(4,4),'reversing':(4,6),'retreating':(4,6),'escape_curve':(4,6),
            'escape_turn':(4,12),'escape_align':(4,12),'waiting_start_space':(4,11)}
        if d.get('hazard_kind') == 'health_wait' and phase not in ('idle','failed'):
            hc[phase]=(5,8)
        if phase == 'confirming' and any(w in reason for w in ('未稳定','未确认','回波不足')):
            hc[phase]=(4,13)
        if phase=='failed':
            if any(word in reason for word in ('反复','同一','连续受困')): hc[phase]=(4,9)
            elif d.get('hazard_kind')=='escape': hc[phase]=(4,8)
            elif '确认' in reason or '回波' in reason: hc[phase]=(4,10)
            elif '后退' in reason or '后方' in reason: hc[phase]=(4,8)
        # Count only committed zone additions, never infer a write from resuming.
        count = int(d.get('obstacle_count','0'))
        old_count = self.previous.get('obstacle_count')
        self.previous['obstacle_count'] = count
        if old_count is not None and count > old_count and d.get('hazard_kind') == 'sonar' and phase not in ('idle','failed'):
            self.say(4,7)
        self.edge('hazard',(phase,hc.get(phase)),hc.get(phase),phase in ('reversing','retreating','escape_curve','escape_turn','escape_align') or stopped)
        if phase=='cancelling' and stopped and d.get('hazard_kind') in ('sonar','cliff'):
            kind=d.get('hazard_kind')
            sources=d.get('hazard_near_sources','')
            hits=set(sources.split(','))
            code=(4,5) if kind=='cliff' else ((4,1) if 'front' in hits else ((4,2) if 'side_left' in hits else ((4,3) if 'side_right' in hits else None)))
            self.edge('near', (phase,kind,sources),code)
        else:
            self.previous.pop('near',None)
        # Specific sensor faults take precedence over generic task messages.
        if now-self.base_at < 2.:
            connected=self.base.get('connected')=='true' and float(self.base.get('heartbeat_age_s','99'))<.8
            self.edge('base_connected',connected,(5,5) if not connected else None)
            sensor_fault = int(self.base.get('sensor_fault_flags','0'))
            tof_bad = bool(sensor_fault & 0x1a)
            us_bad = bool(sensor_fault & 0x24)
            if g.side_ultrasonic_enabled:
                us_bad = us_bad or now-self.us_at>.6 or any(r.get('status') not in (1,2) or r.get('age_ms',9999)>.6*1000 for r in (self.us or [])[1:])
            self.edge('tof_bad',tof_bad,(5,3) if tof_bad else None,stopped)
            self.edge('us_bad',us_bad,(5,2) if us_bad else None,stopped)
            fault = tof_bad or us_bad or not connected or state=='paused_fault'
            if fault:
                self.previous['had_fault'] = True
                self.recovered_since = None
            elif self.previous.get('had_fault'):
                if getattr(self, 'recovered_since', None) is None: self.recovered_since = now
                if now-self.recovered_since >= 2.:
                    self.say(5,7)
                    self.previous['had_fault'] = False
            obstacles=int(self.base.get('obstacle_flags','0'))
            near = (4,5) if obstacles&3 else ((4,1) if obstacles&4 else None)
            if not near and self.us and now-self.us_at<=.6 and g.side_ultrasonic_enabled:
                hit=[i for i in (1,2) if self.us[i].get('status')==1 and self.us[i].get('age_ms',9999)<=600 and .02<=self.us[i].get('distance_m',99)<=.12]
                if hit: near=(4,2 if 1 in hit else 3)
            self.edge('sensor_obstacle',near,near,stopped)
        elif now-g.mode_started_at>15:
            self.edge('base_connected',False,(5,5))

    def tick(self):
        try:
            now = time.monotonic()
            key=(self.g.mode,self.g.mode_started_at)
            if self.mode_key != key: self.mode_reset()
            if self.io.connected and not getattr(self, 'startup_announced', False):
                self.say(1,1)
                self.startup_announced = True
            if self.connected_before != self.io.connected:
                self.g.get_logger().info('CI1302串口'+('已连接: '+self.io.device if self.io.connected else '已断开: '+self.io.error))
            self.connected_before = self.io.connected
            if not self.io.connected:
                # An accepted autonomous task keeps its existing supervision;
                # pending, not-yet-executed serial requests cannot survive loss.
                if self.pending: self.cancel()
                return
            for _ in range(4):
                try: kind, at = self.io.incoming.get_nowait()
                except queue.Empty: break
                if now-at <= .5: self.request(kind,now)
            self.tick_pending(now)
            self.observe(now)
        except Exception as error:
            self.last_error = str(error)
            if self.pending:
                self.g._service('stop')
                self.g._invalidate_navigation('语音返航失败，保持停车')
            self.cancel()
            self.g.get_logger().warning('语音指令未执行: '+str(error))
            # Neutral control-state warning, never claim motion has started.
            self.say(5,16 if locals().get('kind')=='mapping' else 6,urgent=True)

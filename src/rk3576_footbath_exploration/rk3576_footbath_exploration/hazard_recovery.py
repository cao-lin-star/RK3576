from .near_obstacle import record_echo
"""Bounded reverse recovery with explicit task ownership; never rotates over a cliff."""
import json
import copy
import math
import os
import time
from pathlib import Path

from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Header, UInt8
from .dock_motion import pose_tuple, segment_error
from .navigation_recovery import NavigationRecoveryClient
from .retreat_recovery import RetreatRecoveryMixin
from .health_wait import HealthWaitMixin
from .marking_recheck import MarkingRecheckMixin
from .corridor_escape import CorridorEscapeMixin
from .obstacle_overlay import ObstacleOverlay, load_sidecar, write_sidecar
from .near_obstacle import (FRONT_CLEAR_M, FRONT_PREFERRED_M, MAX_AGE_S, SIDE_STOP_M, WindowedConfirmation, ClearConfirmation,
                            near_echo, recovery_near_echo, merge_session_obstacles)


# Absolute probe distances; keep aligned with F407 board_config.h.
CLIFF_STOP_M = 0.28
CLIFF_CLEAR_M = 0.26


def hazard_kind(left, right, sonar):
    if not all(math.isfinite(v) and v > 0 for v in (left, right, sonar)):
        raise ValueError('测距无效，禁止自动恢复，请人工处理')
    if left >= CLIFF_STOP_M or right >= CLIFF_STOP_M:
        return 'cliff'
    return 'sonar' if sonar <= .20 else None


def ground_clear(left, right, sonar):
    return left <= CLIFF_CLEAR_M and right <= CLIFF_CLEAR_M and sonar >= FRONT_CLEAR_M


def reverse_progress(start, current):
    progress, cross, heading = segment_error(start, current, -1)
    if not all(math.isfinite(v) for v in (progress, cross, heading)):
        raise ValueError('里程计无效')
    if progress < -.015 or abs(cross) > .035 or abs(heading) > .10:
        raise ValueError('后退方向或轨迹异常，请人工处理')
    return progress


class HazardRecovery(MarkingRecheckMixin, CorridorEscapeMixin, HealthWaitMixin, RetreatRecoveryMixin):
    def __init__(self, node):
        self.n = node
        self.assume_rear_safe = bool(node.declare_parameter(
            'hazard_recovery.assume_rear_ground_safe', True).value)
        self.ranges = {}
        self.init_retreat()
        self.init_corridor()
        self.side_enabled = os.environ.get('FOOTBATH_SIDE_ULTRASONIC_ENABLED', '1') != '0'
        self.sonar_samples = {}
        self.confirmation = WindowedConfirmation()
        self.clear_confirmation = ClearConfirmation()
        self.confirm_detail = "等待停车后的新回波"
        self.confirmed_points = {}
        self.near_resume_clearance = float(node.declare_parameter(
            'near_obstacle.resume_center_clearance_m', .33).value)
        if not math.isfinite(self.near_resume_clearance) or not .30 <= self.near_resume_clearance <= .40:
            raise ValueError('invalid near-obstacle resume clearance')
        for key, topic in (('front', '/range/ultrasonic_front_observe'),
                           ('side_left', '/range/ultrasonic_left'),
                           ('side_right', '/range/ultrasonic_right')):
            node.create_subscription(Range, topic,
                lambda msg, k=key: self._sonar_range(k, msg), qos_profile_sensor_data)
        self.source, self.source_at = None, 0.
        self.source_wait_since = None
        self.source_fresh_since = None
        self.fault, self.fault_at = 0, 0.
        self.bridge_connected = True
        self.heartbeat_age = 0.
        self.health_wait_at = None
        self.health_clear_at = None
        self.active = False
        self.phase = 'idle'
        self.zones = []
        self.pending = []
        self.last_zones = 0.
        self.manual_at = 0.
        self.last_success = -math.inf
        self.owner = None
        self.owner_token = None
        self.budget_owner = None
        self.attempts = 0
        self.nav_owner = NavigationRecoveryClient(node)
        self.applied = {}
        map_file = node.declare_parameter('hazard_recovery.map_file', '').value
        prefix = str(Path(map_file).with_suffix('')) if map_file else ''
        self.overlay = ObstacleOverlay(self, prefix)
        if prefix:
            self.zones, self.overlay.observations = load_sidecar(prefix)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.zpub = node.create_publisher(PoseArray, '/safety/hazard_zones', qos)
        self.modepub = node.create_publisher(Bool, '/safety/recovery_active', 10)
        self.cmdpub = node.create_publisher(Twist, '/cmd_vel_recovery', 10)
        for layer in ('local_costmap', 'global_costmap'):
            node.create_subscription(Header, '/' + layer + '/hazard_zones_applied',
                                     lambda m, k=layer: self._applied(k, m), 10)
        for key, topic in [('left', '/range/tof_left'), ('right', '/range/tof_right'),
                           ('sonar', '/range/ultrasonic')]:
            node.create_subscription(Range, topic, lambda m, k=key: self._range(k, m),
                                     qos_profile_sensor_data)
        node.create_subscription(UInt8, '/chassis/control_source', self._source, 10)
        node.create_subscription(Twist, '/cmd_vel_manual', self._manual, 10)
        node.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostic, 10)
        self.mode(False)

    def _sonar_range(self, key, msg):
        if key != 'front' and not self.side_enabled:
            return
        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        age = self.n.get_clock().now().nanoseconds / 1e9 - stamp
        if not (0 <= age <= MAX_AGE_S and msg.header.frame_id and
                (math.isnan(msg.range) or .02 <= msg.range <= 4.0)):
            self.sonar_samples.pop(key, None)
            return
        old = self.sonar_samples.get(key)
        if old is not None and stamp <= old[2]:
            return
        self.sonar_samples[key] = (msg, now-age, stamp)

    def near_sources(self, now):
        return [key for key, (msg, acquired, _) in self.sonar_samples.items()
                if (key == 'front' or self.side_enabled) and
                0 <= now-acquired <= MAX_AGE_S and near_echo(key, msg.range)]

    def sides_clear(self, now):
        if not self.side_enabled:
            return True
        for key in ('side_left', 'side_right'):
            sample = self.sonar_samples.get(key)
            if sample is None or not 0 <= now-sample[1] <= MAX_AGE_S:
                raise ValueError('左右超声波失联，不自动恢复运动')
            value = sample[0].range
            if not (math.isnan(value) or value > SIDE_STOP_M):
                return False
        return True

    def clear_for_resume(self, values, now):
        self.resume_detail = ''
        for label, value in zip(('左TOF','右TOF'), values[:2]):
            if not math.isfinite(value) or not 0 < value <= CLIFF_CLEAR_M:
                self.resume_detail=f'{label}未恢复到26cm以内'
                return False
        near = getattr(self,'kind',None)=='sonar' and bool(self.pending)
        front_limit = FRONT_CLEAR_M
        if not math.isfinite(values[2]) or values[2] < front_limit:
            self.resume_detail=f'前超声{values[2]*100:.1f}cm，需达到{front_limit*100:.0f}cm'
            return False
        if not self.sides_clear(now):
            self.resume_detail='左/右超声仍在12cm近障范围内'
            return False
        if near:
            pose = self.n.home.current_pose()
            if pose is None:
                raise ValueError('近障脱离位置无法确认，保持停车')
            x,y,_ = pose_tuple(pose)
            if not all(math.isfinite(v) for v in (x,y)):
                raise ValueError('近障脱离位置无效，保持停车')
            for p in self.pending:
                # Costmap footprint .22+.02, obstacle radius, plus grid margin.
                # This is collision clearance, not a universal 40cm stand-off.
                required=max(self.near_resume_clearance,.24+p.get('radius',.04)+.05)
                distance=math.hypot(x-p['x'],y-p['y'])
                if distance < required:
                    label={'front':'前方','side_left':'左侧','side_right':'右侧'}.get(p.get('source'),'已确认')
                    self.resume_detail=f'{label}障碍距车体中心{distance*100:.1f}cm，需达到{required*100:.0f}cm'
                    return False
        self.resume_detail='测距与障碍碰撞间距已解除，等待稳定确认'
        return True

    def prefer_front_margin(self, values, now, progress):
        # Optional comfort margin only; never extend a segment or latch a fault
        # just to reach 35cm after all mandatory release conditions are satisfied.
        if (self.kind!='sonar' or self.phase!='reversing' or not self.pending or
                not ('front' in self.trigger_sources or any(p.get('source')=='front' for p in self.pending))
                or not FRONT_CLEAR_M<=values[2]<FRONT_PREFERRED_M or self.clear_since is not None
                or self.near_extension_at is not None or now-self.reverse_at>=18.):
            return False
        remaining=self.sonar_reverse_limit-progress
        distance=min(FRONT_PREFERRED_M-values[2],remaining,.05)
        if distance<.01:
            return False
        try:
            # Use only pre-recovery traversed ground; radar alone is insufficient.
            self.near_original_trace.plan(now,self.n.home.dock.odom,distance,distance+.02)
            self.rear_sweep_clear(distance)
        except Exception:
            return False
        self.resume_detail='必要条件已解除，已有安全退路内优先争取前方35cm余量'
        return True

    def start_near_reverse(self, now):
        self.sonar_reverse_limit=.15
        self.sonar_total_limit=.30
        self.near_extension_at=None
        self.clear_since=None
        # Freeze evidence from BEFORE this recovery. A new reverse must not
        # certify its own next segment just by adding points while moving.
        self.near_ground_history=list(self.ground_history)
        self.near_original_trace=copy.deepcopy(self.trace)
        self.phase='reversing';self.reverse_at=now
        self.start_odom=self.n.home.dock.odom;self.progress_at=now
        self.last_progress=0.

    def extend_near_reverse(self, now, progress):
        self.zero()
        if progress >= self.sonar_total_limit-.005:
            raise ValueError(f'累计后退已达{self.sonar_total_limit*100:.0f}cm上限；'+self.resume_detail)
        if self.near_extension_at is None:
            self.near_extension_at=now
        self.n._reason='当前后退段结束，停车检查下一段；'+self.resume_detail
        if now-self.near_extension_at>3.:
            raise ValueError('后退分段检查未能停稳；'+self.resume_detail)
        if now-self.near_extension_at<.4 or now-self.n.home.last_motion<.4:
            return True
        distance=min(.05,self.sonar_total_limit-progress)
        try:
            # Both checks are necessary: known ground and the complete live sweep.
            try:
                available=self.near_original_trace.plan(now,self.n.home.dock.odom,distance,distance+.02)
                if available<distance:
                    raise ValueError('原路长度不足')
                self.rear_sweep_clear(distance)
            except ValueError:
                self.short_rear_plan(now,(distance,),self.near_ground_history)
        except Exception as error:
            raise ValueError(f'下一段后退未获许可：{error}；恢复条件：{self.resume_detail}') from error
        self.sonar_reverse_limit=min(self.sonar_total_limit,progress+distance)
        self.near_extension_at=None
        self.progress_at=now;self.last_progress=progress
        self.n.get_logger().info(f'下一段后退已获地面/雷达许可，累计上限{self.sonar_reverse_limit*100:.0f}cm')
        return True

    def confirm_near(self, now):
        self.confirmed_points = {}
        if not self.overlay.stable(now):
            self.confirmation.reset()
            if hasattr(self,'record_confirmation'): self.record_confirmation.reset()
            self.clear_confirmation.reset()
            self.confirm_detail = '里程计或定位尚未连续停稳'
            return False
        # Include any additional probe triggered while waiting; do not resume
        # because one direction cleared while another still has a near return.
        self.trigger_sources = list(dict.fromkeys(self.trigger_sources+self.near_sources(now)))
        resolved = []
        reasons = []
        labels={'front':'前方','side_left':'左侧','side_right':'右侧'}
        for key in self.trigger_sources:
            sample=self.sonar_samples.get(key)
            reason='等待三次稳定新回波'
            if sample is None or not 0 <= now-sample[1] <= MAX_AGE_S:
                self.confirmation.reset(key); self.clear_confirmation.reset(key)
                reasons.append(labels[key]+'测距过期或失联'); continue
            msg,acquired,stamp=sample
            if acquired <= max(self.confirm_at,self.n.home.last_motion+.4,
                               self.overlay.gate.after(),self.overlay.tf_stable_at+.9):
                self.confirmation.reset(key); self.clear_confirmation.reset(key)
                reasons.append(labels[key]+'仍是停稳前样本'); continue
            if not recovery_near_echo(key,msg.range):
                if not math.isfinite(msg.range):
                    self.confirmation.reset(key);self.clear_confirmation.reset(key)
                    reasons.append(labels[key]+'无回波，继续停车等待有效证据')
                    continue
                confirmed=self.confirmation.observe(key,stamp,None)
                cleared=self.clear_confirmation.observe(key,stamp,msg.range)
                if confirmed is not None:
                    self.confirmed_points[key]=confirmed
                    resolved.append(key)
                elif cleared:
                    self.confirmation.reset(key)
                    resolved.append(key)
                else:
                    reasons.append(labels[key]+'近远回波交替，等待窗口内近障证据或连续远回波')
                continue
            self.clear_confirmation.reset(key)
            try:
                tf=self.n.home.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                q=tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                point=(tf.transform.translation.x+msg.range*math.cos(yaw),
                       tf.transform.translation.y+msg.range*math.sin(yaw))
            except Exception:
                self.confirmation.reset(key)
                reasons.append(labels[key]+'采样时刻定位变换未就绪'); continue
            confirmed=self.confirmation.observe(key,stamp,point)
            if confirmed is not None:
                self.confirmed_points[key]=confirmed
                resolved.append(key)
            else:
                reasons.append(labels[key]+reason)
        # Wider recording has independent evidence; never changes release/stop logic.
        if not hasattr(self, 'record_confirmation'):
            self.record_confirmation=WindowedConfirmation()
        for key,(msg,acquired,stamp) in self.sonar_samples.items():
            if key!='front' and not self.side_enabled: continue
            if (not 0<=now-acquired<=MAX_AGE_S or
                acquired<=max(self.confirm_at,self.n.home.last_motion+.4,
                              self.overlay.gate.after(),self.overlay.tf_stable_at+.9)):
                self.record_confirmation.reset(key);continue
            if not record_echo(msg.range):
                self.record_confirmation.reset(key);continue
            try:
                tf=self.n.home.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                q=tf.transform.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                point=(tf.transform.translation.x+msg.range*math.cos(yaw),
                       tf.transform.translation.y+msg.range*math.sin(yaw))
                confirmed=self.record_confirmation.observe(key,stamp,point)
                if confirmed is not None and key not in self.confirmed_points:
                    self.confirmed_points[key]=confirmed
            except Exception:
                self.record_confirmation.reset(key)
        self.confirm_detail='；'.join(reasons) or '静止确认完成'
        # Publish each confirmed probe immediately; another probe may still wait.
        recorded = getattr(self, 'stationary_recorded', {})
        fresh=[dict(x=p[0],y=p[1],radius=.04,kind='near',expires=0.,source=key)
               for key,p in self.confirmed_points.items() if key not in recorded]
        if fresh:
            self.zones=merge_session_obstacles(self.zones,fresh)
            recorded.update({p['source']:p for p in fresh})
            self.stationary_recorded=recorded
            self.near_recorded=True
            self.publish_zones(now,force=True)
        self.pending=list(recorded.values())
        return bool(self.trigger_sources) and all(k in resolved for k in self.trigger_sources)

    def resume_clear_observation(self, values, now):
        # This branch is only reached with three finite cleared echoes from
        # every triggering probe, and no confirmed obstacle to record.
        if not self.clear_for_resume(values,now): return False
        if self.owner=='navigation':
            self.phase='resuming_owner';self.resume_at=now
            self.nav_owner.send('resume',self.owner_token)
            return True
        if self.owner=='home_return':
            if not self.n.home.resume_after_hazard(self.owner_token):
                raise ValueError('原返航目标已失效，保持停车')
        else:
            self.n._state=self.n.RUNNING
            self.n._reason='连续有效回波确认近障已解除，不创建障碍，继续探索'
            self.n._publish_resume()
        self.active=False;self.phase='idle';self.mode(False)
        self.owner=None;self.owner_token=None
        return True

    def _applied(self, layer, header):
        if header.frame_id == 'map':
            stamp = header.stamp.sec*1000000000 + header.stamp.nanosec
            self.applied[layer] = (stamp, time.monotonic())

    def zones_applied(self):
        return all(k in self.applied and self.applied[k][0] >= self.marked_stamp
                   and self.applied[k][1] >= self.marked_at
                   for k in ('local_costmap', 'global_costmap'))

    def eligible_owner(self, now):
        n = self.n
        if n._state == n.RUNNING:
            return 'exploration'
        if n._state == 'returning_home' and n.home.phase == 'returning':
            return 'home_return'
        if (n._state == 'navigation_ready' and n.home.phase == 'navigation_ready'
                and self.nav_owner.available(now)):
            return 'navigation'
        return None

    def cancel(self, reason, preserve_trace=False):
        """Revoke a recovery immediately; never revive it on a later callback."""
        if self.owner == 'navigation' and self.owner_token is not None:
            self.nav_owner.send('abort', self.owner_token)
        self.end_corridor()
        self.health_wait_at = None
        self.health_clear_at = None
        self.active = False
        self.escape_requested = False
        if preserve_trace:
            self.trace.hold(time.monotonic(),self.n.home.dock.odom)
        else:
            self.trace.clear()
        self.motion_window.update(None,time.monotonic(),(0.,0.,0.))
        self.phase = 'idle'
        self.owner = None
        self.owner_token = None
        self.zero()
        self.mode(False)

    def _range(self, key, msg):
        age = (self.n.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
        value = msg.range
        if not (0 <= age <= .5 and math.isfinite(value)
                and msg.min_range <= value <= msg.max_range):
            self.ranges.pop(key, None)
        else:
            self.ranges[key] = (value, time.monotonic())
            if key in ('left', 'right'):
                self.last_good_tof[key] = self.ranges[key]

    def _source(self, msg):
        self.source, self.source_at = msg.data, time.monotonic()
        if msg.data >= 2 and self.active:
            self.fail('手柄或调试串口接管，恢复已取消，请人工处理')

    def _manual(self, msg):
        self.manual_at = time.monotonic()
        if self.active:
            self.fail('手动遥控接管，自动恢复已取消')

    def _diagnostic(self, msg):
        for status in msg.status:
            if status.name == 'rk3576_footbath/stm32_serial_bridge':
                values = {v.key: v.value for v in status.values}
                try:
                    self.bridge_connected = values.get('connected') == 'true'
                    self.heartbeat_age = float(values.get('heartbeat_age_s', 'inf'))
                    self.fault = int(values['fault_flags'])
                    self.fault_at = time.monotonic()
                except (KeyError, ValueError):
                    self.fault_at = 0.

    def values(self, now):
        if any(k not in self.ranges or now-self.ranges[k][1] > .5
               for k in ('left', 'right', 'sonar')):
            raise ValueError('ToF/超声波失联或数据无效，请人工处理；不自动后退')
        return tuple(self.ranges[k][0] for k in ('left', 'right', 'sonar'))

    def gate(self, healthy, now):
        d = self.n.home.dock
        if not healthy or self.n.home.current_pose() is None or d.odom is None or now-d.odom_at > .5:
            raise ValueError('传感器、定位或里程计失效，恢复已停止')
        if (self.source is not None and self.source >= 2) or now-self.manual_at < .5:
            raise ValueError('人工控制占用，自动恢复已停止')
        if now-self.fault_at > 2.5 or (self.fault & ~1024):
            raise ValueError('底盘通信或非障碍故障，恢复已停止')
        # Source frames are sent independently at 10 Hz. A gap stops motion
        # immediately, but does not immediately destroy the recovery session.
        stale = self.source is None or now-self.source_at > .8
        if stale and self.source_wait_since is None:
            self.source_wait_since = now
        if self.source_wait_since is not None:
            self.zero()
            self.clear_since = None
            self.n._reason = '控制来源遥测短暂中断，已停车等待恢复（最多3秒）'
            if now-self.source_wait_since >= 3.:
                raise ValueError('控制来源遥测持续异常超过3秒，停车等待人工处理')
            if stale:
                self.source_fresh_since = None
                return False
            if self.source_fresh_since is None:
                self.source_fresh_since = now
            if now-self.source_fresh_since < .3:
                return False
            self.source_wait_since = None
            self.source_fresh_since = None
            self.progress_at = now
        return True

    def mode(self, active):
        msg = Bool(); msg.data = active
        self.modepub.publish(msg)

    def zero(self):
        self.cmdpub.publish(Twist())
        self.n._publish_lease(False)

    def fail(self, reason):
        save_partial = self.owner == 'exploration' and getattr(self, 'kind', '') in ('escape', 'tof_invalid')
        now=time.monotonic()
        preserve=(self.source in (0,1) and now-self.source_at<=.5 and now-self.manual_at>1.
                  and now-self.n.home.dock.odom_at<=.25 and now-self.fault_at<=2.5
                  and not (self.fault & ~1024))
        self.cancel(reason,preserve_trace=preserve)
        self.phase = 'failed'
        self.n._fault_latched = True
        self.n._pause(self.n.PAUSED_FAULT, reason)
        self.n.home.message = reason
        if save_partial:
            self.n._request_map_save('退出失败，保存未完成地图')

    def reset_allowed(self):
        try:
            if not self.clear_for_resume(self.values(time.monotonic()), time.monotonic()):
                return False, '危险尚未解除，不能继续探索；请人工恢复位置'
        except ValueError as e:
            return False, str(e)
        self.phase = 'idle'
        self.source_wait_since = None
        self.source_fresh_since = None
        return True, ''

    def configure_kind(self, kind, values, now, owner):
        n = self.n
        self.corridor_near_handoff=False
        if kind != 'health_wait' and not self.assume_rear_safe:
            raise ValueError('未允许假定后方地面安全，请人工处理')
        if kind not in ("sonar", "health_wait") and not (kind == "escape" and owner == "exploration") and now-self.last_success < 10.:
            raise ValueError('短时间重复触发，停止重试，请人工处理')
        pose = n.home.current_pose()
        if pose is None and kind != 'health_wait':
            raise ValueError('无法记录危险位置，保持停车')
        x, y, yaw = pose_tuple(pose) if pose is not None else (0., 0., 0.)
        points = []
        if kind == 'cliff':
            if values[0] >= CLIFF_STOP_M: points.append((.205, .205))
            if values[1] >= CLIFF_STOP_M: points.append((.205, -.205))
        elif kind == 'sonar':
            self.stationary_recorded = {}
            self.record_confirmation = WindowedConfirmation()
            self.sonar_reverse_limit = .15
            self.near_recorded = False
            self.clear_confirmation.reset()
            self.trigger_sources = self.near_sources(now)
            self.confirmation.reset()
            self.confirmed_points = {}
        self.pending = [dict(x=x+px*math.cos(yaw)-py*math.sin(yaw),
                             y=y+px*math.sin(yaw)+py*math.cos(yaw),
                             radius=.04 if kind == 'cliff' else .06,
                             kind=kind, expires=0. if kind == 'cliff' else now+30.)
                        for px, py in points]
        if kind in ('escape', 'tof_invalid'):
            self.prepare_retreat(kind, now, owner)
            self.retreat_wait_at = None
        self.escape_requested = False
        self.motion_window.update(None,now,n.home.dock.odom)
        self.kind = kind

    def begin(self, kind, values, now, owner='exploration'):
        n = self.n
        self.configure_kind(kind, values, now, owner)
        self.owner = owner
        if owner == 'home_return':
            self.owner_token = n.home.suspend_for_hazard()
            if self.owner_token is None:
                raise ValueError('当前返航阶段不允许台阶恢复')
            key = (owner, n.home.started)
        elif owner == 'navigation':
            self.owner_token = self.nav_owner.capture(now)
            key = (owner, self.owner_token['session'], self.owner_token['token'])
        else:
            self.owner_token = None
            key = (owner,)
        if key != self.budget_owner:
            self.budget_owner, self.attempts = key, 0
        if kind != 'health_wait' and owner != 'exploration' and self.attempts >= 3:
            raise ValueError('同一导航任务危险恢复已达3次上限，请人工处理')
        if kind != 'health_wait':
            self.attempts += 1
        n.get_logger().warning(
            f'hazard recovery: owner={owner} kind={kind} attempt={self.attempts} '
            f'tof_left={values[0]:.3f} tof_right={values[1]:.3f} '
            f'ultrasonic={values[2]:.3f} fault_flags={self.fault}')
        reason = {'health_wait': '通信或数据异常，停车等待恢复后继续原任务', 'escape': '检测到往返或导航受阻，取消目标并准备退出死胡同',
                  'tof_invalid': 'TOF仍有通信但测量无效，暂停建图并准备有界后退'}.get(kind, '检测到台阶/近障，暂停原任务并等待导航退出')
        n._pause('hazard_recovery', reason)
        if not n._cancel_client.service_is_ready():
            raise ValueError('导航取消服务不可用，保持停车')
        self.marking_retry_active = False
        self.active = True
        self.phase = 'acquiring_owner' if owner == 'navigation' else 'cancelling'
        if owner == 'navigation':
            n.home.phase = 'hazard_suspended'
            self.nav_owner.send('suspend', self.owner_token)
        else:
            self.cancel_future = n._cancel_client.call_async(CancelGoal.Request())
        self.started = now
        self.cancel_at = now
        self.start_odom = n.home.dock.odom
        self.last_progress = 0.
        self.progress_at = now
        self.clear_since = None
        self.mode(True)
        self.zero()

    def publish_zones(self, now, force=False):
        self.zones = [z for z in self.zones if not z['expires'] or z['expires'] > now]
        if not force and now-self.last_zones < .5:
            return
        self.last_zones = now
        msg = PoseArray(); msg.header.frame_id = 'map'
        msg.header.stamp = self.n.get_clock().now().to_msg()
        for z in self.zones:
            p = Pose(); p.position.x=float(z['x']); p.position.y=float(z['y'])
            p.position.z=float(z['radius']); p.orientation.w=1.
            msg.poses.append(p)
        self.zpub.publish(msg)
        return msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec

    def save(self, prefix):
        # Persistent overlay is tied to the exact saved map image/geometry.
        write_sidecar(prefix, self.zones, self.overlay.observations)
        # Navigation edits persist immediately to the loaded map. Mapping edits
        # are written on each explicit map save, never into an older snapshot.

    def tick(self, healthy):
        n = self.n; now = time.monotonic()
        self.publish_zones(now)
        # Multi-frame swept-path evidence is needed only for supervised exits.
        # Collect during cancellation/settling too, before any exit can move.
        if self.escape_requested or (self.active and self.kind == 'escape'):
            try:
                self.collect_corridor_scan(now)
            except Exception:
                pass  # The motion gate still rejects incomplete evidence.
        owner = self.eligible_owner(now) if not self.active else self.owner
        if not self.active and owner is None:
            if n._state==n.PAUSED_FAULT:
                d=n.home.dock
                trusted=(healthy and self.source in (0,1) and now-self.source_at<=.5
                         and now-self.manual_at>1. and now-d.odom_at<=.25
                         and now-self.fault_at<=2.5 and not (self.fault & ~1024))
                if trusted:
                    self.trace.hold(now,d.odom)
                else:
                    self.trace.clear()
            return False
        if self.active and n._state != 'hazard_recovery':
            self.cancel('任务状态已改变')
            return False
        try:
            if self.wait_for_health(healthy, now, owner):
                return True
            values = self.retreat_values(now, owner)
            if self.active and self.kind=='escape' and self.new_retreat_kind=='tof_invalid':
                # Retain task/episode ownership, but reacquire cancellation + settling.
                self.zero()
                self.end_corridor()
                self.kind='tof_invalid'
                self.phase='cancelling'
                self.cancel_future=n._cancel_client.call_async(CancelGoal.Request())
                self.cancel_at=now
                self.clear_since=None
                self.retreat_wait_at=None
                n._reason='TOF通信正常但近距测量失效，停稳后切换最多10cm的已走直线路径短退'
                return True
            if not self.active:
                kind = self.new_retreat_kind or hazard_kind(*values)
                if kind not in ('cliff', 'tof_invalid'):
                    kind = 'sonar' if self.near_sources(now) else None
                if kind is None and self.navigation_stalled(owner, now, healthy):
                    kind = 'escape'
                if kind is None and self.escape_requested and owner == 'exploration':
                    kind = 'escape'
                if kind is None:
                    self.observe_trace(now, values, healthy)
                    return False
                if not self.gate(healthy, now):
                    return True
                self.begin(kind, values, now, owner)
                return True
            owner_phase = None
            if self.owner == 'home_return' and not n.home.hazard_resume_valid(self.owner_token):
                raise ValueError('返航已取消或总时间超限，不自动恢复')
            if self.owner == 'navigation':
                owner_phase = self.nav_owner.phase(self.owner_token, now)
                if self.phase not in ('acquiring_owner', 'resuming_owner') and owner_phase != 'suspended':
                    raise ValueError('普通导航恢复所有权丢失，保持停车')
            if not self.gate(healthy, now):
                n._publish_pause()
                return True
            # Maintain actual traversed ground through cancellation, reverse,
            # map acknowledgement and departure waiting. Ownership, live health
            # and odometry gates above must pass first. Cliff/invalid TOF never
            # create ground evidence; distances and timestamps are not invented.
            if all(math.isfinite(v) and 0<v<=.26 for v in values[:2]):
                self.remember_ground(now,n.home.dock.odom)
                previous=self.trace.last_observation
                if previous is not None and math.dist(previous[1][:2],n.home.dock.odom[:2])<1e-6:
                    self.trace.hold(now,n.home.dock.odom)
                else:
                    self.trace.observe(now,n.home.dock.odom,True)
            n._publish_pause()
            # Reassert ownership; repeated True does not pulse the motor output.
            self.mode(True)
            if self.phase in ('marking', 'marking_recheck'):
                return self.marking_tick(values,now,reverse_progress(self.start_odom,n.home.dock.odom))
            if self.phase == 'waiting_start_space':
                return self.wait_for_start_space(values,now)
            if self.phase == 'acquiring_owner':
                self.zero()
                if now-self.started > 2.:
                    raise ValueError('普通导航目标未确认暂停，保持停车')
                if owner_phase != 'suspended':
                    return True
                self.cancel_future = n._cancel_client.call_async(CancelGoal.Request())
                self.phase = 'cancelling'
                self.cancel_at = now
            if self.phase == 'resuming_owner':
                self.zero()
                if not self.clear_for_resume(values, now):
                    raise ValueError('重规划前测距再次异常，保持停车')
                if now-self.resume_at > 4.:
                    raise ValueError('原导航目标未确认恢复，保持停车')
                if owner_phase == 'active':
                    self.active = False; self.phase = 'idle'; self.last_success = now
                    self.mode(False)
                    n.home.phase = 'navigation_ready'
                    n._state = 'navigation_ready'
                    n._reason = ('退出完成，继续原导航目标' if self.kind=='escape' else '近障/台阶已解除并记录，继续原导航目标')
                    self.owner = None; self.owner_token = None
                return True
            if self.phase in ('cancelling','confirming'):
                self.trace.hold(now,n.home.dock.odom)
            if self.phase in ('cancelling', 'health_ready'):
                self.zero()
                cancel_at = getattr(self, 'cancel_at', self.started)
                if self.phase == 'cancelling' and now-cancel_at > 4.:
                    raise ValueError('导航未在4秒内退出，请人工处理')
                if not self.cancel_future.done(): return True
                result = self.cancel_future.result()
                if result is None or result.return_code not in (0, 2):
                    raise ValueError('取消导航失败，保持停车')
                if n.home.active_goals or (n.home.last_status < cancel_at and
                        getattr(result,'goals_canceling',[True])): return True
                if now-cancel_at < .6: return True
                if now-n.home.last_motion < .4: return True
                if self.kind == 'health_wait':
                    kind = self.new_retreat_kind or hazard_kind(*values)
                    if kind not in ('cliff', 'tof_invalid'):
                        kind = 'sonar' if self.near_sources(now) else None
                    if kind is None:
                        # Require the existing release hysteresis; never resume
                        # toward an obstacle just because serial traffic is back.
                        if not self.resume_clear_observation(values, now):
                            self.n._reason = '通信已恢复，等待保护解除后继续原任务'
                            self.phase = 'health_ready'
                        return True
                    self.configure_kind(kind, values, now, self.owner)
                if self.kind == 'sonar':
                    self.phase = 'confirming'; self.confirm_at = now; self.confirm_attempt = 1
                    self.confirmation.reset(); self.clear_confirmation.reset(); self.confirmed_points = {}
                    return True
                if self.kind=='escape' and self.begin_corridor(values,now):
                    return True
                self.phase = 'reversing'; self.reverse_at = now
                self.start_odom = n.home.dock.odom; self.progress_at = now
                if self.kind in ('escape', 'tof_invalid'):
                    if self.kind=='escape' and self.owner=='exploration' and self.retreat_skip_reason:
                        return self.select_other_frontier(values,now)
                    try:
                        self.retreat_distance = self.choose_retreat(self.kind,now)
                    except ValueError as error:
                        if self.kind!='escape' or self.owner!='exploration':
                            raise
                        n.get_logger().warning('停车后直退条件改变：'+str(error))
                        return self.select_other_frontier(values,now)
                    self.retreat_map_start = pose_tuple(n.home.current_pose())
            if self.phase == 'confirming':
                self.zero()
                if now-self.confirm_at > 3.:
                    attempt=getattr(self,'confirm_attempt',1)
                    # Unstable echoes do not finish mapping. Keep zero speed and
                    # independently recorded points; retry only fresh settled data.
                    began=getattr(n,'_exploration_started_at',None)
                    if self.owner=='exploration' and isinstance(began,(int,float)):
                        n._exploration_started_at=began+(now-self.confirm_at)
                    self.confirm_attempt=attempt+1;self.confirm_at=now
                    self.confirmation.reset();self.clear_confirmation.reset();self.confirmed_points={}
                    n._reason=f'近障回波不稳定，保持停车自动重试（第{self.confirm_attempt}轮）：'+self.confirm_detail
                    return True
                if values[0] >= CLIFF_STOP_M or values[1] >= CLIFF_STOP_M:
                    raise ValueError('近障确认期间出现台阶，保持停车')
                if not self.confirm_near(now):
                    n._reason='已暂停导航，静止确认近障：'+self.confirm_detail
                    return True
                if not self.pending:
                    if not self.resume_clear_observation(values,now):
                        self.confirm_detail='触发探头已解除，但其他测距尚未达到恢复安全范围'
                    return True
                distance=None
                if getattr(self,'corridor_near_handoff',False) and self.clear_for_resume(values,now):
                    distance=0.  # Already clear: mark/resume without requiring a needless reverse.
                elif getattr(self,'corridor_near_handoff',False):
                    # The robot may already have turned. Never reuse the old
                    # straight heading or let this new reverse certify its floor.
                    if self.corridor_near_retry_at is not None and now-self.corridor_near_retry_at<3.:
                        return True
                    self.corridor_near_retry_at=now
                    try:
                        try:
                            distance=self.corridor_near_trace.plan(now,n.home.dock.odom,.03,.15)
                            self.rear_sweep_clear(distance)
                        except ValueError:
                            distance=self.short_rear_plan(now,(.10,.08,.05,.03),self.corridor_near_ground)
                    except ValueError as error:
                        n._reason='近障已停稳确认，安全退路暂不可用，保留任务每3秒重评估：'+str(error)
                        return True
                self.start_near_reverse(now)
                if distance is not None:
                    self.sonar_reverse_limit=min(.15,distance)
                    self.near_original_trace=copy.deepcopy(self.corridor_near_trace)
                    self.near_ground_history=list(self.corridor_near_ground)
                n.get_logger().info('近障已确认，首段最多15cm；必要时停稳复核后每段追加5cm，总上限30cm')
            if self.corridor_active:
                return self.corridor_tick(values,now)
            progress = reverse_progress(self.start_odom, n.home.dock.odom)
            if self.kind in ('escape', 'tof_invalid'):
                return self.retreat_tick(values, now, progress)
            reverse_limit = getattr(self,'sonar_reverse_limit',.15) if self.kind=='sonar' else .15
            if progress > reverse_limit+.010:
                raise ValueError(f'后退超过{reverse_limit*100:.0f}cm上限，保持停车')
            # Pure reverse, checked against live rear lidar. Ground is assumed
            # safe only because the operator explicitly selected this policy.
            d = n.home.dock
            release_clear=self.clear_for_resume(values,now)
            prefer_margin=release_clear and self.prefer_front_margin(values,now,progress)
            if not release_clear:
                if self.kind=='sonar' and not self.rear_observed():
                    raise ValueError('近障后退的后方雷达覆盖不足或数据过期，保持停车')
                old_direction = getattr(d, 'direction', 1)
                try:
                    d.direction = -1
                    d._obstacle()
                finally:
                    d.direction = old_direction
            if self.kind == 'sonar':
                if values[0] >= CLIFF_STOP_M or values[1] >= CLIFF_STOP_M:
                    raise ValueError('超声恢复期间新出现台阶，保持停车')
                new_sources=set(self.near_sources(now))-set(self.trigger_sources)
                if new_sources:
                    raise ValueError('后退期间其他方向新增近障，需重新停稳确认：'+','.join(sorted(new_sources)))
                if reverse_limit>.15 and not release_clear:
                    self.rear_sweep_clear(max(0.,reverse_limit-progress))
            if self.kind=='sonar' and all(math.isfinite(v) and 0<v<=.26 for v in values[:2]):
                self.trace.observe(now,d.odom,True,self.turning_room())
            if release_clear and not prefer_margin:
                self.zero()
                if self.clear_since is None: self.clear_since = now
                if now-self.clear_since >= .5 and self.phase != 'marking':
                    if self.kind == 'sonar':
                        if not getattr(self,'near_recorded',False):
                            self.zones = merge_session_obstacles(self.zones, self.pending)
                    else:
                        self.zones.extend(self.pending)
                    self.marked_stamp = self.publish_zones(now, force=True)
                    # Hold until both layers actually apply this update, not
                    # merely until a message has been sent onto DDS.
                    self.phase = 'marking'; self.marked_at = now
            else:
                self.clear_since = None
            if self.phase == 'marking':
                return self.marking_tick(values,now,progress)
            # A clear sample at the segment boundary must get its full 0.5s
            # stopped confirmation before considering any extra reverse motion.
            if self.clear_since is not None:
                return True
            if self.kind=='sonar':
                n._reason='近障直退中；'+self.resume_detail
                if now-self.reverse_at>=20.:
                    raise ValueError('近障恢复达到20秒总时限；'+self.resume_detail)
                if progress>=reverse_limit-.005 or self.near_extension_at is not None:
                    return self.extend_near_reverse(now,progress)
            elif now-self.reverse_at>=6. or progress>=.15:
                raise ValueError('台阶恢复已达15cm或6秒上限，保持停车')
            if progress > self.last_progress+.002:
                self.last_progress=progress; self.progress_at=now
            if now-self.progress_at > 2.:
                raise ValueError('后退无里程计进展，请人工处理')
            msg=Twist(); msg.linear.x=-.03
            n._reason=('近障低速直退；'+self.resume_detail) if self.kind=='sonar' else '台阶低速直退中；禁止转向'
            n._publish_lease(True); self.cmdpub.publish(msg)
            return True
        except Exception as error:
            detail=str(error)
            # A live safety gate can close again during an approved short retry.
            # Stop and re-evaluate without replenishing its distance/time budget.
            recoverable=('近障恢复达到20秒总时限','累计后退已达','下一段后退未获许可',
                         '后退分段检查未能停稳','近障后退的后方雷达覆盖不足',
                         '超声恢复期间新出现台阶','后退期间其他方向新增近障')
            if (getattr(self,'marking_retry_active',False) and self.active and
                    self.kind=='sonar' and self.phase=='reversing' and
                    isinstance(error,ValueError) and detail.startswith(recoverable)):
                self.begin_marking_recheck(now,detail)
                return True
            self.fail(detail)
            return True

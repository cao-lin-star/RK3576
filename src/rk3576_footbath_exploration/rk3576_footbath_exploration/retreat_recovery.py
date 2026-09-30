"""Extensions used by HazardRecovery's existing cancel/ownership/lease gates."""
import math
import time
from geometry_msgs.msg import Twist
from rclpy.time import Time
from std_msgs.msg import UInt8, Bool
from .dock_motion import pose_tuple, first_swept_obstacle
from .retreat_trace import RetreatTrace, RetreatEpisode, MotionWindow, tof_retreat_allowed


class RetreatRecoveryMixin:
    def init_retreat(self):
        self.trace = RetreatTrace()
        self.ground_history = []
        self.rear_short = False
        self.motion_window = MotionWindow()
        self.escape_requested = False
        self.retreat_episode = RetreatEpisode()
        self.retreat_counted = False
        self.tof_state = 0
        self.tof_state_at = -math.inf
        self.last_good_tof = {}
        self.retreat_max = float(self.n.declare_parameter('retreat.maximum_distance_m', 1.0).value)
        if not .25 <= self.retreat_max <= 1.5:
            raise ValueError('invalid retreat maximum distance')
        self.n.create_subscription(UInt8, '/chassis/tof_state', self._tof_state, 10)
        self.start_space=False;self.start_space_at=-math.inf
        self.departure_clear_at=None;self.departure_wait_last=None;self.departure_recheck_at=None
        self.n.create_subscription(Bool,'/explore/start_space',self._start_space,10)

    def _start_space(self, msg):
        self.start_space=bool(msg.data);self.start_space_at=time.monotonic()

    def start_space_ready(self, now):
        return self.start_space and 0<=now-self.start_space_at<=1.5

    def _tof_state(self, msg):
        self.tof_state, self.tof_state_at = msg.data, time.monotonic()
        for i,k in enumerate(('left','right')):
            if not msg.data & (1<<i):
                self.ranges.pop(k, None)

    def retreat_values(self, now, owner):
        self.new_retreat_kind = None
        missing = any(k not in self.ranges or now-self.ranges[k][1] > .5 for k in ('left','right'))
        if not missing:
            return self.values(now)
        sonar = self.ranges.get('sonar')
        if sonar is None or now-sonar[1] > .5:
            return self.values(now)  # fail closed through the normal error
        if self.active and self.kind == 'tof_invalid':
            if now-self.tof_state_at > .35 or self.tof_state & 12 != 12:
                raise ValueError('TOF通信中断，停止后退；测量无效恢复不适用于断线')
            return tuple(self.ranges[k][0] if k in self.ranges and now-self.ranges[k][1]<=.5 else math.nan for k in ('left','right'))+(sonar[0],)
        if (owner == 'exploration' and (not self.active or self.kind=='escape') and
                tof_retreat_allowed(now,self.tof_state,self.tof_state_at,self.last_good_tof,sonar[0])):
            self.new_retreat_kind = 'tof_invalid'
            return tuple(self.ranges[k][0] if k in self.ranges and now-self.ranges[k][1]<=.5 else math.nan for k in ('left','right'))+(sonar[0],)
        if now-self.tof_state_at<=.35 and self.tof_state & 12 == 12:
            raise ValueError('TOF仍有通信但测量无效，缺少可用的近期近距恢复证据')
        raise ValueError('TOF通信中断或帧过期，停车等待通信恢复')

    def turning_room(self):
        d = self.n.home.dock
        scan = d.scan
        if scan is None or time.monotonic()-d.scan_at > .5:
            return False
        age=(self.n.get_clock().now().nanoseconds-Time.from_msg(scan.header.stamp).nanoseconds)/1e9
        if not 0 <= age <= .5 or len(scan.ranges)<20 or abs(scan.angle_increment)*len(scan.ranges)<5.8:
            return False
        try:
            tf=self.n.home.buffer.lookup_transform('base_footprint',scan.header.frame_id,Time())
            t,q=tf.transform.translation,tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            covered=set()
            for i,r in enumerate(scan.ranges):
                angle=scan.angle_min+i*scan.angle_increment+yaw
                if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                    if math.hypot(t.x+r*math.cos(angle),t.y+r*math.sin(angle)) < .30:
                        return False
                    covered.add(int((angle%(2*math.pi))/(math.pi/6)))
                elif r == math.inf:
                    covered.add(int((angle%(2*math.pi))/(math.pi/6)))
            return len(covered)==12
        except Exception:
            return False

    def rear_observed(self):
        d=self.n.home.dock; scan=d.scan
        if scan is None or time.monotonic()-d.scan_at>.35:
            return False
        age=(self.n.get_clock().now().nanoseconds-Time.from_msg(scan.header.stamp).nanoseconds)/1e9
        if not 0<=age<=.5:
            return False
        try:
            tf=self.n.home.buffer.lookup_transform('base_footprint',scan.header.frame_id,Time())
            q=tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            counts={i:[0,0] for i in range(3,9)}
            for i,r in enumerate(scan.ranges):
                sector=int(((scan.angle_min+i*scan.angle_increment+yaw)%(2*math.pi))/(math.pi/6))
                if sector not in counts: continue
                counts[sector][0]+=1
                if r==math.inf or (math.isfinite(r) and scan.range_min<=r<=scan.range_max):
                    counts[sector][1]+=1
            return all(total>=2 and valid/total>=.8 for total,valid in counts.values())
        except Exception:
            return False

    def remember_ground(self, now, pose):
        if pose is None or not all(math.isfinite(v) for v in pose):
            return
        history = self.ground_history
        if history and math.hypot(pose[0]-history[-1][1],pose[1]-history[-1][2]) > .12:
            history.clear()  # discontinuous odometry cannot certify nearby ground
        history[:] = [p for p in history if now-p[0] <= 180.]
        if not history or math.hypot(pose[0]-history[-1][1],pose[1]-history[-1][2]) >= .02:
            history.append((now,pose[0],pose[1]))
        del history[:-600]

    def rear_sweep_clear(self, distance):
        if not self.rear_observed():
            raise ValueError('后方雷达覆盖不足或数据过期')
        d=self.n.home.dock; scan=d.scan
        tf=self.n.home.buffer.lookup_transform('base_footprint',scan.header.frame_id,Time.from_msg(scan.header.stamp))
        t,q=tf.transform.translation,tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        points=[]
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                a=scan.angle_min+i*scan.angle_increment+yaw
                points.append((t.x+r*math.cos(a),t.y+r*math.sin(a)))
        if not points:
            raise ValueError('后方扫掠检查缺少有效雷达测距')
        hit=first_swept_obstacle(points,-1,d.radius,d.margin,distance+.025)
        if hit is not None:
            raise ValueError(f'后退扫掠范围有障碍（x={hit[0]:.2f},y={hit[1]:.2f}m）')

    def short_rear_plan(self, now, distances=(.15,.12,.10,.08), history=None):
        # Radar certifies obstacle clearance, not floor support. Require both
        # current ground readings and a recent traversed neighborhood as well.
        if not self.assume_rear_safe:
            raise ValueError('未启用后方地面安全假定')
        for key in ('left','right'):
            value,stamp=self.ranges.get(key,(math.nan,0.))
            if now-stamp>.5 or not math.isfinite(value) or not 0<value<=.26:
                raise ValueError('TOF地面测量无效，禁止备用短退')
        pose=self.n.home.dock.odom
        if pose is None:
            raise ValueError('没有新鲜里程计')
        history=[p for p in (self.ground_history if history is None else history) if now-p[0]<=180.]
        rear_error = None
        for distance in distances:
            # Spatial evidence tolerates a gentle bend; no historical yaw gate.
            # Require every 1cm of center travel near previously traversed ground.
            covered=True
            for i in range(int(round(distance/.01))+1):
                d=i*.01
                x=pose[0]-d*math.cos(pose[2]); y=pose[1]-d*math.sin(pose[2])
                if not any(math.hypot(x-p[1],y-p[2])<=.06 for p in history):
                    covered=False;break
            if not covered:
                continue
            try:
                self.rear_sweep_clear(distance)
            except ValueError as error:
                rear_error = error
                continue
            self.rear_short=True
            self.n.get_logger().info(f'历史直线不足；后方雷达扫掠及近期已走地面符合短退条件，计划后退{distance*100:.0f}cm')
            return distance
        if rear_error is not None:
            raise rear_error
        raise ValueError('后方可能空旷，但短退范围缺少近期已走地面记录')

    def observe_trace(self, now, values, healthy):
        safe = (healthy and self.n._state in (self.n.RUNNING,'navigation_ready','returning_home') and self.source == 1
                and now-self.source_at<.8 and now-self.fault_at<2.5 and not self.fault
                and all(math.isfinite(v) and 0<v<=.26 for v in values[:2]))
        if self.retreat_episode.observe(now,self.n.home.dock.odom,safe and not self.active):
            self.n.get_logger().info('正常行驶已离开受困点至少80cm并持续取得进展，本轮脱困计数已清零')
        if (not safe and healthy and self.source in (0,1) and now-self.source_at<.8
                and now-self.fault_at<2.5 and not (self.fault & ~1024)
                and all(math.isfinite(v) and 0<v<=.26 for v in values[:2])):
            self.trace.hold(now,self.n.home.dock.odom)
            return
        if safe:
            self.remember_ground(now,self.n.home.dock.odom)
        self.trace.observe(now,self.n.home.dock.odom,safe, self.turning_room() if safe else False)

    def navigation_stalled(self, owner, now, healthy):
        key=None
        if healthy and owner=='navigation':
            c=self.nav_owner.context;key=('navigation',c['session'],c['token'])
        elif healthy and owner=='home_return':
            key=('home_return',self.n.home.started)
        return self.motion_window.update(key,now,self.n.home.dock.odom or (0.,0.,0.))

    def choose_retreat(self, kind, now, owner=None):
        self.rear_short=False
        minimum = .08 if kind=='tof_invalid' else .25
        maximum = .10 if kind=='tof_invalid' else self.retreat_max
        try:
            return self.trace.plan(now,self.n.home.dock.odom,minimum,maximum)
        except ValueError:
            if kind != 'escape':
                raise
            # Short known corridor is useful even when no 25 cm segment exists.
            try:
                return self.trace.plan(now,self.n.home.dock.odom,.08,maximum)
            except ValueError as trace_error:
                if (owner or self.owner) != 'exploration':
                    raise
                try:
                    return self.short_rear_plan(now)
                except Exception as rear_error:
                    raise ValueError(f'历史直退：{trace_error}；备用短退：{rear_error}') from rear_error

    def prepare_retreat(self, kind, now, owner):
        point = pose_tuple(self.n.home.current_pose())
        self.retreat_skip_reason = ''
        odom = self.n.home.dock.odom
        self.retreat_episode.begin(odom)
        self.retreat_counted=False
        try:
            if self.retreat_episode.attempts>=2:
                raise ValueError('本轮连续受困已尝试2次后退，尚未确认驶离；停止重复后退')
            self.retreat_distance = self.choose_retreat(kind,now,owner)
        except ValueError as error:
            if kind != 'escape' or owner != 'exploration':
                raise
            self.retreat_skip_reason = str(error)
            self.retreat_distance = 0.
            self.n.get_logger().warning('不执行直退，取消目标后等待起步空间恢复：'+str(error))
        self.retreat_map_start=point
        self.escape_requested=False
        # A blocked departure pose must not penalize the distant frontier.

    def select_other_frontier(self, values, now):
        # Historical name retained for callers. First recover departure space;
        # a local blockage is not a reason to cycle through distant frontiers.
        self.zero();self.end_corridor();self.n._publish_lease(False)
        if self.phase!='waiting_start_space':
            self.departure_clear_at=None;self.departure_wait_last=now;self.departure_recheck_at=now
            self.departure_block_detail=''
        self.phase='waiting_start_space'
        return self.wait_for_start_space(values,now)

    def wait_for_start_space(self, values, now):
        self.zero();self.n._publish_lease(False)
        self.escape_motion_pub.publish(UInt8(data=0))
        if self.departure_wait_last is not None:
            began=getattr(self.n,'_exploration_started_at',None)
            if isinstance(began,(int,float)):
                self.n._exploration_started_at=began+max(0.,now-self.departure_wait_last)
        self.departure_wait_last=now
        # Near echoes take priority over planning, including the release band
        # of a recovery already blocked by the front sensor.
        if self.near_sources(now) or (math.isfinite(values[2]) and values[2]<.28):
            self.configure_kind('sonar',values,now,self.owner)
            if values[2]<.28 and 'front' not in self.trigger_sources:self.trigger_sources.append('front')
            self.phase='confirming';self.confirm_at=now;self.confirm_attempt=1
            self.n._reason='起步近障尚未解除，保持停车确认障碍，暂不更换探索点'
            return True
        if (self.fault or not self.clear_for_resume(values,now) or
                not self.start_space_ready(now) or now-self.n.home.last_motion<.6):
            self.departure_clear_at=None
            self.n._reason='起步空间尚未恢复，保持停车等待地图或保护恢复，不重复更换探索点'
            if getattr(self,'departure_block_detail',''):
                self.n._reason+='；'+self.departure_block_detail
            if (self.kind=='escape' and not self.fault and self.clear_for_resume(values,now)
                    and 0<=now-self.start_space_at<=1.5 and not self.start_space
                    and self.retreat_episode.attempts<2 and now-self.n.home.last_motion>=.6
                    and (self.departure_recheck_at is None or now-self.departure_recheck_at>=3.)):
                self.departure_recheck_at=now
                if self.begin_corridor(values,now):return True
                try:
                    distance=self.choose_retreat('escape',now)
                except ValueError as error:
                    self.departure_block_detail=(self.corridor_detail+'；直退：'+str(error))
                    self.n._reason='退出方案均未通过，保留任务每3秒复核；'+self.departure_block_detail
                    return True
                self.departure_block_detail=''
                self.retreat_distance=distance;self.retreat_counted=False
                self.start_odom=self.n.home.dock.odom
                self.retreat_map_start=pose_tuple(self.n.home.current_pose())
                self.reverse_at=now;self.progress_at=now;self.last_progress=0.;self.clear_since=None
                self.phase='retreating'
                self.n._reason='起步仍受阻，安全退路复核通过，继续本轮有界退出'
            return True
        if self.departure_clear_at is None:self.departure_clear_at=now
        if now-self.departure_clear_at<.6:return True
        self.escape_requested=False
        self.motion_window.update(None,now,self.n.home.dock.odom)
        self.active=False;self.phase='idle';self.owner=None;self.owner_token=None
        self.mode(False);self.n._state=self.n.RUNNING
        self.n._reason='起步空间已恢复，优先重新规划原探索点'
        self.n._publish_resume()
        return True

    def retreat_tick(self, values, now, progress):
        # Called only AFTER cancellation acknowledgement and normal health gates.
        n=self.n; d=n.home.dock
        if any(math.isfinite(v) and v>=.28 for v in values[:2]):
            raise ValueError('退出期间检测到台阶，停止本次退出并保留地图')
        if self.kind=='tof_invalid' and (now-self.tof_state_at>.35 or self.tof_state & 12 != 12):
            raise ValueError('TOF通信中断，停止后退并保留地图')
        current=pose_tuple(n.home.current_pose())
        map_shift=math.hypot(current[0]-self.retreat_map_start[0],current[1]-self.retreat_map_start[1])
        if abs(map_shift-progress)>.10:
            raise ValueError('退出期间定位与里程计不一致，保持停车')
        if progress>self.retreat_distance+.015:
            raise ValueError('沿原路退出超过距离上限，保持停车')
        if not self.rear_observed():
            raise ValueError('后方雷达覆盖不足或数据过期，禁止继续退出')
        if self.rear_short:
            self.rear_sweep_clear(max(0.,self.retreat_distance-progress))
        old_direction=d.direction if hasattr(d,'direction') else 1
        try:
            d.direction=-1
            d._obstacle()
        finally:
            d.direction=old_direction
        if all(math.isfinite(v) and 0<v<=.26 for v in values[:2]):
            self.remember_ground(now,d.odom)
            self.trace.observe(now,d.odom,True,self.turning_room())
        reached=progress>=self.retreat_distance-.005
        # Share the mandatory release gate with sonar/cliff recovery.
        # A planned escape distance remains necessary; 35cm front clearance does not.
        clear=self.clear_for_resume(values, now)
        if self.kind=='tof_invalid':
            ready=clear and progress>=.02 and self.tof_state & 3 == 3
        else:
            ready=reached and clear and (self.owner=='exploration' or self.turning_room())
        if ready and self.kind=='escape' and self.owner=='exploration' and not self.start_space_ready(now):
            return self.select_other_frontier(values,now)
        if ready:
            self.zero()
            if self.clear_since is None: self.clear_since=now
            if now-self.clear_since>=.6:
                self.trace.hold(now,d.odom)
                self.motion_window.update(None,now,d.odom)
                if self.owner=='navigation':
                    self.phase='resuming_owner';self.resume_at=now
                    self.nav_owner.send('resume',self.owner_token)
                    return True
                if self.owner=='home_return':
                    if not n.home.resume_after_hazard(self.owner_token):
                        raise ValueError('退出完成但原返航目标失效，保持停车')
                else:
                    n._state=n.RUNNING;n._reason='退出完成，优先重新规划原探索点'
                    n._publish_resume()
                self.active=False; self.phase='idle'; self.last_success=now
                self.mode(False); self.owner=None; self.owner_token=None
            return True
        self.clear_since=None
        if now-self.reverse_at > (6. if self.kind=='tof_invalid' else self.retreat_max/.03+3.):
            raise ValueError('退出时间已到仍未恢复，暂停建图并保留地图')
        if reached:
            if self.kind=='escape' and self.owner=='exploration':
                return self.select_other_frontier(values,now)
            self.zero()
            if not hasattr(self,'retreat_wait_at') or self.retreat_wait_at is None: self.retreat_wait_at=now
            if now-self.retreat_wait_at>1.:
                raise ValueError('已到退出距离上限，测距或转向空间仍不满足，暂停并保留地图')
            return True
        if progress>self.last_progress+.002:
            self.last_progress=progress; self.progress_at=now
        if now-self.progress_at>2.:
            raise ValueError('沿原路退出无里程计进展，保持停车')
        self.phase='retreating'
        n._reason=('TOF测量无效，沿已走路线低速后退恢复' if self.kind=='tof_invalid'
                   else ('后方扫掠检查通过，正在低速短退' if self.rear_short else '检测到往返/导航受阻，正在沿已走直线路段退出死胡同'))+f'（{progress*100:.0f}/{self.retreat_distance*100:.0f}cm）'
        msg=Twist();msg.linear.x=-.03
        if not self.retreat_counted:
            self.retreat_episode.attempts+=1
            self.retreat_counted=True
            n.get_logger().info(f'本轮连续受困开始第{self.retreat_episode.attempts}次后退')
        n._publish_lease(True);self.cmdpub.publish(msg)
        return True

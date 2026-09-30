"""Turn in verified free space, otherwise follow a frozen traversed curve."""
import math
import copy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from rclpy.time import Time
from .dock_motion import pose_tuple
from .corridor_route import FrozenRoute, wrap, swept_centers


class RadarEvidencePending(ValueError):
    """Missing live evidence is a stopped wait, never free space."""


def required_sectors(centers, turn=False):
    # Rotation/alignment checks the complete body. Translation checks the
    # leading half-plane of every step, widened one sector at both sides.
    directions=[math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in zip(centers,centers[1:])
                if math.dist(a,b)>1e-7]
    if turn or not directions:
        return set(range(24))
    return {i for i in range(24) if any(
        math.cos((i+.5)*math.pi/12-a)>=-math.sin(math.pi/12) for a in directions)}


class CorridorEscapeMixin:
    def init_corridor(self):
        self.corridor_active=False
        self.corridor_handoff_clear_at=None
        self.corridor_scans=[]
        self.corridor_scan_pose=None
        self.corridor_detail=""
        self.corridor_wait_at=None
        self.corridor_wait_last=None
        self.corridor_recheck_at=None
        self.corridor_wait_total=0.
        self.corridor_evidence_progress=0.
        self.corridor_alternative_detail=""
        self.escape_motion_pub=self.n.create_publisher(UInt8,'/safety/escape_motion',10)

    def end_corridor(self):
        self.corridor_handoff_clear_at=None
        self.corridor_active=False
        self.escape_motion_pub.publish(UInt8(data=0))

    def collect_corridor_scan(self, now):
        d=self.n.home.dock;scan=d.scan
        if scan is None or now-d.scan_at>.35:raise RadarEvidencePending('高位雷达过期')
        stamp=Time.from_msg(scan.header.stamp).nanoseconds/1e9
        age=self.n.get_clock().now().nanoseconds/1e9-stamp
        if not 0<=age<=.35:raise RadarEvidencePending('高位雷达时间戳过期')
        pose=d.odom
        if pose is None or not all(math.isfinite(v) for v in pose):raise ValueError('雷达证据缺少有效里程计')
        previous=self.corridor_scan_pose
        if previous is not None and (math.dist(pose[:2],previous[:2])>.12 or abs(wrap(pose[2]-previous[2]))>.25):
            self.corridor_scans=[]
        self.corridor_scan_pose=pose
        self.corridor_scans=[f for f in self.corridor_scans if now-f[0]<=.8]
        if self.corridor_scans and stamp<=self.corridor_scans[-1][1]:return
        tf=self.n.home.buffer.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp))
        t,q=tf.transform.translation,tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        rays=[];points=[]
        for i,r in enumerate(scan.ranges):
            a=scan.angle_min+i*scan.angle_increment+yaw
            valid=math.isfinite(r) and scan.range_min<=r<=scan.range_max
            # Preserve the raw scan. Only a positive-infinity no-return can
            # count as rear clearance; NaN/masked/invalid samples remain unknown.
            rays.append((a,1 if valid else (2 if r==math.inf else 0)))
            if valid:points.append((t.x+r*math.cos(a),t.y+r*math.sin(a)))
        self.corridor_scans.append((now-age,stamp,rays,points))
        self.corridor_scans=self.corridor_scans[-12:]

    def corridor_scan_points(self, now, required=None):
        d=self.n.home.dock
        points=[]
        for scan,at,full in ((d.scan,d.scan_at,True),(d.low_scan,d.low_at,False)):
            if scan is None or now-at>.35:raise RadarEvidencePending('转向退出需要两路新鲜雷达')
            age=(self.n.get_clock().now().nanoseconds-Time.from_msg(scan.header.stamp).nanoseconds)/1e9
            if not 0<=age<=.35:raise RadarEvidencePending('转向退出雷达时间戳过期')
            tf=self.n.home.buffer.lookup_transform('base_footprint',scan.header.frame_id,Time.from_msg(scan.header.stamp))
            t,q=tf.transform.translation,tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            sectors=[[0,0] for _ in range(24)];valid=0
            for i,r in enumerate(scan.ranges):
                a=scan.angle_min+i*scan.angle_increment+yaw
                sector=int((a%(2*math.pi))/(math.pi/12))%24
                sectors[sector][0]+=1
                if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                    sectors[sector][1]+=1;valid+=1
                    points.append((t.x+r*math.cos(a),t.y+r*math.sin(a)))
            if valid<10:raise RadarEvidencePending('转向退出雷达有效测距不足')
        self.collect_corridor_scan(now)
        history=[f for f in self.corridor_scans if now-f[0]<=.8]
        if len(history)<3 or history[-1][1]-history[0][1]<.2:
            raise RadarEvidencePending('正在收集至少3帧周边雷达证据，保持停车')
        pose=d.odom
        counts=[[0,0] for _ in range(24)]
        for _,_,rays,hits in history:
            for a,evidence in rays:
                sector=int(((a-pose[2])%(2*math.pi))/(math.pi/12))%24
                rear_no_return=evidence==2 and 6<=sector<18
                counts[sector][0]+=1
                counts[sector][1]+=int(evidence==1 or rear_no_return)
            for x,y in hits:
                dx,dy=x-pose[0],y-pose[1]
                points.append((dx*math.cos(pose[2])+dy*math.sin(pose[2]),-dx*math.sin(pose[2])+dy*math.cos(pose[2])))
        required=set(range(24)) if required is None else required
        missing=[str(i*15) for i,(total,good) in enumerate(counts)
                 if i in required and (total<6 or good<3 or good/total<.5)]
        if missing:
            raise RadarEvidencePending('本次运动方向雷达覆盖不足，未确认空地的扇区起始角：'+','.join(missing)+'度')
        return points

    def corridor_clear(self, now, centers, turn=False):
        # The documented outer envelope is a circle, including protrusions.
        radius=max(.22,self.n.home.dock.radius)+(.08 if turn else .03)
        points=self.corridor_scan_points(now,required_sectors(centers,turn))
        if any(math.hypot(px-x,py-y)<=radius for x,y in centers for px,py in points):
            raise ValueError('车身扫掠范围内有实时雷达障碍')
        pose=self.n.home.current_pose()
        if pose is None:raise ValueError('退出定位失效')
        x,y,a=pose_tuple(pose)
        g=self.overlay.grid
        if g is None or g.info.resolution<=0:raise ValueError('退出地图未就绪')
        origin=g.info.origin;q=origin.orientation
        ga=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        res=g.info.resolution
        for cx,cy in centers:
            mx=x+cx*math.cos(a)-cy*math.sin(a);my=y+cx*math.sin(a)+cy*math.cos(a)
            for label, records in (('已确认障碍',self.zones),('待确认观察点',self.overlay.observations)):
                for z in records:
                    distance=math.hypot(mx-z['x'],my-z['y'])
                    if distance<=radius+z['radius']:
                        raise ValueError(f'{label}阻挡退出：位置({z["x"]:.2f},{z["y"]:.2f})m，'
                                         f'中心间距{distance:.2f}m，所需{radius+z["radius"]:.2f}m')
            dx,dy=mx-origin.position.x,my-origin.position.y
            gx=(dx*math.cos(ga)+dy*math.sin(ga))/res;gy=(-dx*math.sin(ga)+dy*math.cos(ga))/res
            # Include every cell intersecting the footprint, including unknown.
            cells=math.ceil(radius/res)+1
            for ix in range(math.floor(gx)-cells,math.floor(gx)+cells+1):
                for iy in range(math.floor(gy)-cells,math.floor(gy)+cells+1):
                    if math.hypot(ix+.5-gx,iy+.5-gy)*res>radius+res*.7072:continue
                    if not (0<=ix<g.info.width and 0<=iy<g.info.height):raise ValueError('退出扫掠范围超出地图')
                    value=g.data[iy*g.info.width+ix]
                    if value<0 or value>=50:raise ValueError('退出扫掠范围存在占用或未知栅格')

    def begin_corridor(self, values, now):
        if self.kind!='escape' or self.retreat_episode.attempts>=2:return False
        # Near/cliff protection must be resolved before permitting ANY steering.
        if self.fault or not self.clear_for_resume(values,now):return False
        pose=self.n.home.dock.odom
        route=None;route_error=''
        try:route=FrozenRoute(self.trace.points,now,pose)
        except ValueError as error:route_error=str(error)
        try:
            self.corridor_clear(now,[(0.,0.)],turn=True)
            turn=True
        except Exception as error:
            turn=False;turn_error=str(error)
        alignment=None
        forward=False
        # A robot that reversed into this pose may already face its traversed
        # exit. Test translation along that same route without demanding a turn.
        if not turn and route is not None:
            try:
                candidate=copy.deepcopy(route)
                v,w,done=candidate.command(pose,1)
                if done:raise ValueError('已到历史路径终点')
                self.corridor_clear(now,swept_centers(v,w))
                route=candidate;forward=True
            except ValueError:
                pass
        if not turn and not forward:
            self.corridor_detail='掉头未通过：'+turn_error
            if route is None:
                self.corridor_detail+='；曲线退路：'+route_error
                self.n.get_logger().warning(self.corridor_detail)
                if getattr(self,'retreat_skip_reason',''):
                    self.retreat_skip_reason=self.corridor_detail
                return False
            try:
                v,w,_=route.command(pose,-1)
                self.corridor_clear(now,swept_centers(v,w))
            except Exception as error:
                try:
                    target=route.at(.10)
                    alignment=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])+math.pi-pose[2])
                    if not .04<=abs(alignment)<=math.pi/6:
                        raise ValueError('曲线入口需要对齐超过30度或没有可行的小角度修正')
                    # Circular outer envelope: a small alignment still checks
                    # the complete body disk, live sensors and occupied/unknown map.
                    self.corridor_clear(now,[(0.,0.)])
                    test_pose=(pose[0],pose[1],pose[2]+alignment)
                    saved=route.progress
                    try:route.command(test_pose,-1)
                    finally:route.progress=saved
                except Exception as align_error:
                    self.corridor_detail+='；曲线退路未通过：'+str(error)+'；'+str(align_error)
                    self.n._reason=self.corridor_detail
                    self.n.get_logger().warning(self.corridor_detail)
                    if getattr(self,'retreat_skip_reason',''):
                        self.retreat_skip_reason=self.corridor_detail
                    return False
        self.corridor_wait_at=None;self.corridor_wait_last=None
        self.corridor_recheck_at=None
        self.corridor_wait_total=0.
        self.corridor_evidence_progress=0.
        self.corridor_alternative_detail=""
        self.corridor_route=route
        self.corridor_active=True
        self.corridor_alignment=alignment
        self.phase='escape_turn' if turn else ('escape_forward' if forward else ('escape_align' if alignment is not None else 'escape_curve'))
        self.corridor_started=now;self.corridor_progress_at=now
        self.corridor_progress=0.;self.corridor_clear_at=None
        self.corridor_anchor=pose;self.corridor_last=pose
        self.corridor_yaw=0.
        self.corridor_map_anchor=pose_tuple(self.n.home.current_pose())
        self.zero()
        self.n.get_logger().warning('巷道退出策略：'+('空间检查通过，先原地掉头，再沿来路正向驶离' if turn else ('无需掉头，沿冻结的已走路径正向驶离' if forward else '旋转空间不足，沿冻结的已走曲线倒退')))
        return True

    def finish_corridor(self, values, now):
        self.zero()
        if not self.clear_for_resume(values,now):
            self.corridor_clear_at=None
            raise ValueError('退出到达终点，但保护未解除，保持停车；'+self.resume_detail)
        if self.corridor_clear_at is None:self.corridor_clear_at=now
        if now-self.corridor_clear_at<.6:return True
        self.end_corridor()
        if self.owner=='exploration' and not self.start_space_ready(now):
            return self.select_other_frontier(values,now)
        if self.owner=='navigation':
            self.phase='resuming_owner';self.resume_at=now;self.nav_owner.send('resume',self.owner_token)
        else:
            if self.owner=='home_return':
                if not self.n.home.resume_after_hazard(self.owner_token):raise ValueError('原返航目标失效')
            else:
                self.n._state=self.n.RUNNING;self.n._reason='巷道退出完成，优先重新规划原探索点';self.n._publish_resume()
            self.active=False;self.phase='idle';self.mode(False);self.owner=None;self.owner_token=None
        self.last_success=now
        return True

    def reconsider_corridor(self, values, now):
        """Recheck translation on the same traversed ground; never infer a shortcut.

        A partially completed turn may make reverse infeasible. The route's
        heading/cross-track checks must pass at the actual current pose, then
        the normal live radar, recorded obstacle and map sweep must also pass.
        Rejected candidates cannot advance the original route's progress.
        """
        if self.fault or not self.clear_for_resume(values, now):
            return False, '保护尚未解除'
        pose = self.n.home.dock.odom
        errors = []
        for direction, phase, label in ((-1, 'escape_curve', '沿已走曲线倒退'),
                                         (1, 'escape_forward', '沿已走曲线正向驶离')):
            if phase == self.phase:
                continue  # The current strategy was already checked this tick.
            try:
                route = copy.deepcopy(self.corridor_route)
                if route is None:
                    route = FrozenRoute(self.trace.points, now, pose)
                v, w, done = route.command(pose, direction)
                if done:
                    raise ValueError('已到历史路径终点，无替代平移段')
                self.corridor_clear(now, swept_centers(v, w))
            except ValueError as error:
                errors.append(label+'：'+str(error))
                continue
            # Selection grants no velocity. The following tick must revalidate
            # ownership, health, localization continuity and the new live sweep.
            self.corridor_route = route
            self.phase = phase
            self.corridor_started = now
            self.corridor_progress_at = now
            self.corridor_progress = route.progress
            self.corridor_clear_at = None
            self.corridor_wait_at = self.corridor_wait_last = None
            self.corridor_recheck_at = None
            self.corridor_wait_total = 0.
            self.corridor_evidence_progress = route.progress
            self.corridor_alternative_detail = ""
            self.n._reason = '原退出方向持续缺少雷达证据；替代方案检查通过，准备'+label
            self.n.get_logger().warning(self.n._reason)
            return True, ''
        return False, '；'.join(errors)

    def corridor_streams_ready(self, now):
        d=self.n.home.dock
        clock=self.n.get_clock().now().nanoseconds/1e9
        for scan,at in ((d.scan,d.scan_at),(d.low_scan,d.low_at)):
            if scan is None or not 0<=now-at<=.35:return False
            stamp=Time.from_msg(scan.header.stamp).nanoseconds/1e9
            if not 0<=clock-stamp<=.35:return False
            if sum(math.isfinite(r) and scan.range_min<=r<=scan.range_max
                   for r in scan.ranges)<10:return False
        return True

    def yield_corridor_to_exploration(self, values, now):
        # Only relinquish a recovery plan; never authorize a blind manoeuvre.
        if (not self.active or not self.corridor_active or self.kind!='escape' or
                self.owner!='exploration' or self.n._state!='hazard_recovery'):
            self.corridor_handoff_clear_at=None
            return False
        self.zero()
        self.escape_motion_pub.publish(UInt8(data=0))
        if (self.fault or not self.clear_for_resume(values,now) or self.near_sources(now)
                or not self.corridor_streams_ready(now) or now-self.n.home.last_motion<.6):
            self.corridor_handoff_clear_at=None
            return False
        if self.corridor_handoff_clear_at is None:self.corridor_handoff_clear_at=now
        if now-self.corridor_handoff_clear_at<.6:return False
        self.end_corridor()
        return self.select_other_frontier(values,now)

    def corridor_tick(self, values, now):
        # Keep ownership and frozen path during missing radar evidence. No timer
        # expiry or capability lease may turn a temporary blind sector into motion.
        if self.corridor_wait_last is not None:
            elapsed=max(0.,now-self.corridor_wait_last)
            self.corridor_wait_total+=elapsed
            self.corridor_started+=elapsed
            self.corridor_progress_at+=elapsed
            if self.owner=='exploration':
                began=getattr(self.n,'_exploration_started_at',None)
                if isinstance(began,(int,float)):self.n._exploration_started_at=began+elapsed
            self.corridor_wait_last=now
        try:
            result=self.corridor_tick_ready(values,now)
        except RadarEvidencePending as error:
            self.zero()
            self.escape_motion_pub.publish(UInt8(data=0))
            self.n._publish_lease(False)
            self.corridor_clear_at=None
            if self.corridor_wait_at is None:self.corridor_wait_at=now
            self.corridor_wait_last=now
            self.n._reason=str(error)+'；已停车保留建图任务，短暂等待雷达证据'
            # Three seconds is a stopped grace period, never a motion timeout
            # bypass. If all alternatives are unsafe, keep stopped and reassess
            # at a bounded rate; original evidence recovery can still resume it.
            if self.corridor_wait_total >= 3.:
                self.n._reason=str(error)+'；替代退出方案尚无充分证据，保持停车并每3秒重新评估'
                if self.corridor_recheck_at is None or now-self.corridor_recheck_at >= 3.:
                    self.corridor_recheck_at = now
                    changed, detail = self.reconsider_corridor(values, now)
                    self.corridor_alternative_detail = detail
                if self.corridor_alternative_detail:
                    self.n._reason += '；'+self.corridor_alternative_detail
            if self.corridor_wait_total>=12.:
                self.yield_corridor_to_exploration(values,now)
            return True
        self.corridor_handoff_clear_at=None
        self.corridor_wait_at=None;self.corridor_wait_last=None
        # An isolated good scan is not progress. Reset accumulated waiting only
        # after measurable travel/rotation; keep the alternative retry cadence.
        threshold=.08 if self.phase in ('escape_turn','escape_align') else .03
        if self.corridor_progress-self.corridor_evidence_progress>=threshold:
            self.corridor_wait_total=0.
            self.corridor_evidence_progress=self.corridor_progress
            self.corridor_recheck_at=None
            self.corridor_alternative_detail=""
        return result

    def corridor_tick_ready(self, values, now):
        if self.fault:
            raise ValueError('转向退出期间故障触发，停车')
        if not self.clear_for_resume(values,now):
            # Sensor health/ownership is checked by tick before entering here.
            # A sonar hysteresis violation is recoverable, not a terminal fault.
            if (all(math.isfinite(v) and 0<v<=.26 for v in values[:2]) and
                    math.isfinite(values[2])):
                self.zero()
                self.end_corridor()
                self.configure_kind('sonar',values,now,self.owner)
                if values[2]<.28 and 'front' not in self.trigger_sources:
                    self.trigger_sources.append('front')
                self.corridor_near_handoff=True
                self.corridor_near_trace=copy.deepcopy(self.trace)
                self.corridor_near_ground=list(self.ground_history)
                self.corridor_near_retry_at=None
                self.phase='confirming';self.confirm_at=now;self.confirm_attempt=1
                self.n._reason='脱困中再次出现近障，已停车；停稳确认后重新检查安全退路'
                return True
            raise ValueError('转向退出期间地面保护未解除，停车；'+self.resume_detail)
        pose=self.n.home.dock.odom
        if math.dist(pose[:2],self.corridor_last[:2])>.08 or abs(wrap(pose[2]-self.corridor_last[2]))>.15:
            raise ValueError('退出里程计不连续，停车')
        current=pose_tuple(self.n.home.current_pose())
        # A map correction must not silently move the route or known glass.
        relative=lambda p,a:((p[0]-a[0])*math.cos(a[2])+(p[1]-a[1])*math.sin(a[2]),-(p[0]-a[0])*math.sin(a[2])+(p[1]-a[1])*math.cos(a[2]))
        if (math.dist(relative(current,self.corridor_map_anchor),relative(pose,self.corridor_anchor))>.10 or
                abs(wrap((current[2]-self.corridor_map_anchor[2])-(pose[2]-self.corridor_anchor[2])))>.15):
            raise ValueError('退出时定位与里程计不一致，停车')
        self.corridor_yaw+=wrap(pose[2]-self.corridor_last[2]);self.corridor_last=pose
        if self.phase in ('escape_turn','escape_align'):
            aligning=self.phase=='escape_align'
            if math.dist(pose[:2],self.corridor_anchor[:2])>.025:raise ValueError('掉头时车体平移超过2.5cm')
            if now-self.corridor_started>(15. if aligning else 60.):raise ValueError('旋转退出超时')
            self.corridor_clear(now,[(0.,0.)],turn=not aligning)
            target=self.corridor_alignment if aligning else math.pi
            error=target-self.corridor_yaw
            if abs(error)<.04:
                self.zero()
                if self.corridor_clear_at is None:self.corridor_clear_at=now
                if now-self.corridor_clear_at<.6:return True
                if self.corridor_route is None:return self.finish_corridor(values,now)
                self.phase='escape_curve' if aligning else 'escape_forward';self.corridor_started=now;self.corridor_progress_at=now
                self.corridor_progress=0.;self.corridor_clear_at=None
                return True
            self.corridor_clear_at=None
            if (self.corridor_yaw*target<-.10 or abs(self.corridor_yaw)>abs(target)+.12):raise ValueError('旋转方向或角度异常')
            v=0.;w=math.copysign(max(.04,min(.08 if aligning else .12,.5*abs(error))),error);capability=1;progress=abs(self.corridor_yaw)
            self.n._reason='正在小角度对齐已走曲线入口，完成后倒退' if aligning else '已确认完整旋转空间，正在低速掉头后正向驶离'
        else:
            if now-self.corridor_started>self.corridor_route.length/.015+10.:raise ValueError('沿已走曲线退出超时')
            direction=1 if self.phase=='escape_forward' else -1
            v,w,done=self.corridor_route.command(pose,direction)
            if done:return self.finish_corridor(values,now)
            self.corridor_clear(now,swept_centers(v,w))
            capability=3 if direction>0 else 2;progress=self.corridor_route.progress
            self.n._reason=f'沿已走曲线{ "正向驶离" if direction>0 else "低速倒退" }（{progress:.2f}/{self.corridor_route.length:.2f}m）'
        if progress>self.corridor_progress+.005:
            self.corridor_progress=progress;self.corridor_progress_at=now
        if now-self.corridor_progress_at>3.:raise ValueError('巷道退出无实际进展，停车')
        if not self.retreat_counted:
            self.retreat_episode.attempts+=1;self.retreat_counted=True
        self.escape_motion_pub.publish(UInt8(data=capability))
        self.n._publish_lease(True)
        command=Twist();command.linear.x=v;command.angular.z=w;self.cmdpub.publish(command)
        return True

"""Short, supervised straight segments for forward exit and reverse entry."""
import math
import time

from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_tuple(pose):
    p, q = pose.pose.position, pose.pose.orientation
    return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def forward_point(home, distance):
    x, y, yaw = home
    return x+distance*math.cos(yaw), y+distance*math.sin(yaw), yaw


def segment_error(start, current, direction):
    x, y, yaw = start
    dx, dy = current[0]-x, current[1]-y
    return (direction*(dx*math.cos(yaw)+dy*math.sin(yaw)),
            -dx*math.sin(yaw)+dy*math.cos(yaw), wrap(yaw-current[2]))


def entry_segment_distance(home, current, nominal_distance, longitudinal_tolerance,
                           cross_limit, heading_limit):
    """Validate the dock corridor and return the measured reverse distance."""
    progress, cross, heading = segment_error(home, current, 1)
    if (abs(progress-nominal_distance)>longitudinal_tolerance
            or abs(cross)>cross_limit or abs(heading)>heading_limit):
        raise ValueError('基站等待点不在允许走廊内，保持停车；禁止直接倒车')
    return progress


def alignment_translation_unsafe(map_start, map_current, odom_start, odom_current,
                                 map_limit, odom_limit, map_hard_limit,
                                 odom_hard_limit):
    """Classify alignment shift; only hard limits stop before corridor recheck."""
    values = (*map_start[:2], *map_current[:2], *odom_start[:2], *odom_current[:2])
    if not all(math.isfinite(value) for value in values):
        raise ValueError('对齐位姿包含无效值，已停车')
    map_shift = math.hypot(map_current[0]-map_start[0], map_current[1]-map_start[1])
    odom_shift = math.hypot(odom_current[0]-odom_start[0], odom_current[1]-odom_start[1])
    suspect = map_shift>map_limit and odom_shift>odom_limit
    unsafe = map_shift>map_hard_limit or odom_shift>odom_hard_limit
    return unsafe, suspect, map_shift, odom_shift


def straight_command(start, current, direction, distance, speed, tolerance,
                     cross_limit, heading_limit):
    if direction not in (-1, 1) or not all(math.isfinite(v) for v in (*start, *current)):
        raise ValueError('invalid segment pose/direction')
    progress, cross, heading = segment_error(start, current, direction)
    if abs(cross)>cross_limit or abs(heading)>heading_limit or progress<-.03:
        raise ValueError('偏离基站直线/方向，已停车；请检查定位或人工接管')
    if progress>distance+.03:
        raise ValueError('超过直线距离上限，已停车')
    remaining=distance-progress
    if remaining<=tolerance:
        return 0., 0., progress, True
    v=direction*min(speed, max(.02, .5*remaining))
    # Small steering only while translating: never turn in place inside dock.
    w=max(-.08,min(.08,1.0*heading-direction*1.2*cross))
    return v,w,progress,False


def swept_obstacle(points, direction, radius, margin, lookahead):
    """Circle footprint swept along a short straight stopping corridor."""
    return first_swept_obstacle(points, direction, radius, margin, lookahead) is not None


def first_swept_obstacle(points, direction, radius, margin, lookahead):
    """Return body-frame evidence, never discard a close front/side obstacle."""
    for x,y in points:
        if direction*x < 0:
            continue
        center=direction*min(lookahead,max(0.,direction*x))
        if math.hypot(x-center,y)<radius+margin:
            return x, y
    return None


class DockMotion:
    def __init__(self, owner):
        self.owner=owner
        n=owner.node
        def param(name, default):
            return n.declare_parameter('dock.'+name,default).value
        self.enabled=bool(param('enabled',True))
        self.auto_exit=bool(param('auto_exit',False))
        self.distance=float(param('distance_m',.50))
        self.speed=float(param('speed_mps',.05))
        self.tolerance=float(param('distance_tolerance_m',.01))
        self.cross_limit=float(param('cross_track_limit_m',.04))
        self.heading_limit=float(param('heading_limit_rad',.10))
        self.timeout=float(param('segment_timeout_s',45.))
        self.radius=float(param('robot_radius_m',.22))
        self.margin=float(param('obstacle_margin_m',.01))
        self.align_speed=float(param('align_speed_rad_s',.15))
        self.align_tolerance=float(param('align_tolerance_rad',.04))
        self.align_timeout=float(param('align_timeout_s',60.))
        self.align_progress_timeout=float(param('align_progress_timeout_s',10.))
        self.align_map_shift_limit=float(param('align_map_shift_limit_m',.02))
        self.align_odom_shift_limit=float(param('align_odom_shift_limit_m',.02))
        self.align_map_hard_limit=float(param('align_map_hard_limit_m',.08))
        self.align_odom_hard_limit=float(param('align_odom_hard_limit_m',.05))
        self.staging_longitudinal_tolerance=float(
            param('staging_longitudinal_tolerance_m',.06))
        if not (0<self.align_speed<=.3 and 0<self.align_tolerance<=.05
                and 10<=self.align_timeout<=120 and 2<=self.align_progress_timeout<=20):
            raise ValueError('invalid alignment limits')
        if not (0<self.align_odom_shift_limit<self.align_odom_hard_limit<=.10
                and 0<self.align_map_shift_limit<self.align_map_hard_limit<=.15
                and 0<self.staging_longitudinal_tolerance<=.10):
            raise ValueError('invalid alignment translation limits')
        if not (0<self.distance<=1 and 0<self.speed<=.1 and 0<self.tolerance<self.distance
                and 0<self.cross_limit<=.1 and 0<self.heading_limit<=.2
                and 10<=self.timeout<=120 and 0<self.radius<1 and 0<=self.margin<=.1):
            raise ValueError('invalid dock limits')
        self.exit_complete=False
        self.active=False
        self.kind=None
        self.odom=None
        self.odom_at=0.
        self.scan=None
        self.scan_at=0.
        self.low_scan=None
        self.low_at=0.
        self.progress=0.
        self.wait_since=None
        self.stable_since=None
        self.last_progress=0.
        self.start=None
        self.map_start=None
        n.create_subscription(LaserScan,'/scan_high',self._scan,qos_profile_sensor_data)
        n.create_subscription(LaserScan,'/scan_low_front',self._low_scan,qos_profile_sensor_data)

    def _low_scan(self,msg):
        self.low_scan=msg
        self.low_at=time.monotonic()

    def _scan(self,msg):
        self.scan=msg
        self.scan_at=time.monotonic()

    def observe_odom(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        self.odom=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.odom_at=time.monotonic()

    def pause(self):
        self.active=False

    def begin(self,kind):
        h=self.owner
        now=time.monotonic()
        current=h.current_pose()
        if self.odom is None or now-self.odom_at>.5 or current is None:
            raise ValueError('基站直线运动所需里程计/定位未就绪')
        home=pose_tuple(h.pose)
        target=home
        if kind in ('entry','align'):
            target=forward_point(home,self.distance)
        point=pose_tuple(current)
        if self.kind is not None:
            # An interrupted reverse segment must not be replaced by a new
            # 50 cm segment or by navigation inside the dock.
            raise ValueError('此前直线段已中断，请人工恢复位置并重新开始会话')
        segment_distance=self.distance
        if kind=='entry':
            segment_distance=entry_segment_distance(
                home,point,self.distance,self.staging_longitudinal_tolerance,
                self.cross_limit,.06)
        elif (math.hypot(point[0]-target[0],point[1]-target[1])>.04
                or (kind!='align' and abs(wrap(point[2]-target[2]))>.06)):
            raise ValueError('基站出入口未对齐（位置需≤4cm、朝向≤0.06rad），保持停车')
        self.kind=kind
        self.direction=0 if kind=='align' else (1 if kind=='exit' else -1)
        self.align_target=target
        self.segment_distance=segment_distance
        self.align_best=abs(wrap(target[2]-point[2]))
        self.start=self.odom
        self.map_start=point
        self.started=now
        self.progress_at=now
        self.progress=0.
        self.last_progress=0.
        self.wait_since=None
        self.stable_since=None
        self.settled_since=None
        self.align_shift_reported=False
        self.align_guard_since=None
        self.align_guard_count=0
        self.align_guard_key=None
        self.active=True
        h.phase='undocking' if kind=='exit' else 'docking'
        h.message='正在直行50cm驶出基站' if kind=='exit' else '正在按实测距离倒车返回基站起点'
        if kind=='align':
            h.phase='aligning'; h.message='已到等待点，正在独立低速对齐车头'
        h.node._state='dock_motion'
        h.node._reason=h.message
        h.node._publish_pause(force=True)
        h.node._publish_zero()
        h.node._publish_lease(False)

    def _obstacle(self):
        h=self.owner
        scan=self.scan
        if scan is None or time.monotonic()-self.scan_at>.5:
            raise ValueError('高位雷达失联，基站直线段已停车')
        age=(h.node.get_clock().now().nanoseconds-Time.from_msg(scan.header.stamp).nanoseconds)/1e9
        if not 0<=age<=1:
            raise ValueError('高位雷达时间戳异常，已停车')
        tf=h.buffer.lookup_transform('base_footprint',scan.header.frame_id,Time())
        t,q=tf.transform.translation,tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        points=[]
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
                angle=scan.angle_min+i*scan.angle_increment+yaw
                points.append((t.x+r*math.cos(angle),t.y+r*math.sin(angle)))
        if not points:
            raise ValueError('高位雷达无有效测距，已停车')
        hit=first_swept_obstacle(points,self.direction,self.radius,self.margin,self.speed*.5+.025)
        if hit is not None:
            raise ValueError(f'高位雷达阻挡直线通道：车体系x={hit[0]:.3f}m,y={hit[1]:.3f}m')
        if self.direction>=0:
            low=self.low_scan
            if low is None or time.monotonic()-self.low_at>.5:
                raise ValueError('低位雷达失联，暂停正向出站')
            age=(h.node.get_clock().now().nanoseconds-Time.from_msg(low.header.stamp).nanoseconds)/1e9
            if not 0<=age<=1:
                raise ValueError('低位雷达时间戳异常，暂停出站')
            tf=h.buffer.lookup_transform('base_footprint',low.header.frame_id,Time())
            t,q=tf.transform.translation,tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            points=[]
            for i,r in enumerate(low.ranges):
                if math.isfinite(r) and low.range_min<=r<=low.range_max:
                    angle=low.angle_min+i*low.angle_increment+yaw
                    points.append((t.x+r*math.cos(angle),t.y+r*math.sin(angle)))
            hit=first_swept_obstacle(points,self.direction,self.radius,self.margin,self.speed*.5+.025)
            if hit is not None:
                raise ValueError(f'低位雷达阻挡直线通道：车体系x={hit[0]:.3f}m,y={hit[1]:.3f}m')
        return False

    def stop(self):
        self.owner.node._state='dock_waiting'
        self.owner.node._publish_lease(False)
        self.owner.node._publish_zero()

    def tick(self,healthy):
        h=self.owner; n=h.node; now=time.monotonic()
        if not self.active:
            return
        if now-self.started>(self.align_timeout if self.kind=='align' else self.timeout):
            h._fail('基站直线运动超时，已停车')
            return
        issue=''
        current=h.current_pose()
        if not healthy or current is None or now-self.odom_at>.5:
            issue='基站直线段定位/传感器失效'
        try:
            if not issue and self._obstacle():
                issue='基站直线段雷达检测到障碍物'
        except Exception as error:
            issue=str(error)
        if issue:
            self.stop()
            self.wait_since=now if self.wait_since is None else self.wait_since
            self.stable_since=None
            h.phase='dock_waiting'; h.message=issue+'；停车等待恢复'
            n._state='dock_waiting'
            if now-self.wait_since>=h.recovery_timeout:
                h._fail('基站直线段等待恢复超时: '+issue)
            return
        if self.wait_since is not None:
            self.stop()
            if self.stable_since is None:
                self.stable_since=now
            if now-self.stable_since<h.stable_time:
                return
            self.wait_since=None; self.progress_at=now
        try:
            if self.kind=='align':
                self._align_tick(now,current)
                return
            v,w,progress,done=straight_command(self.start,self.odom,self.direction,
                self.segment_distance,self.speed,self.tolerance,self.cross_limit,self.heading_limit)
            map_progress,map_cross,map_heading=segment_error(self.map_start,pose_tuple(current),self.direction)
            if abs(map_progress-progress)>.06 or abs(map_cross)>self.cross_limit or abs(map_heading)>self.heading_limit:
                raise ValueError('基站直线段定位与轮式里程计不一致，已停车')
            self.progress=progress
            if progress-self.last_progress>=.005:
                self.last_progress=progress; self.progress_at=now
            if done:
                self.stop()
                if self.settled_since is None:
                    self.settled_since=now
                if now-self.settled_since<.6:
                    return
                self.active=False
                if self.kind=='exit':
                    self.exit_complete=True; self.kind=None
                    h.phase='ready'; h.message='已驶出基站，开始规划探索路线'
                    n._begin_exploration()
                else:
                    h.phase='arrived'; h.message='已倒车返回基站起点并停车'
                    n._pause(n.COMPLETE,h.message)
                return
            if now-self.progress_at>5.:
                raise ValueError('基站直线段5秒无有效进展，已停车')
            h.phase='undocking' if self.kind=='exit' else 'docking'
            h.message=('正向出站' if self.kind=='exit' else '倒车入站')+f'：{max(0.,progress):.2f}/{self.segment_distance:.2f}m'
            n._state='dock_motion'; n._reason=h.message
            n._publish_lease(True)
            msg=Twist(); msg.linear.x=v; msg.angular.z=w
            n._stop_publisher.publish(msg)
        except Exception as error:
            h._fail(str(error))

    def _align_tick(self,now,current):
        h=self.owner; n=h.node; point=pose_tuple(current)
        unsafe,suspect,map_shift,odom_shift=alignment_translation_unsafe(
            self.map_start,point,self.start,self.odom,
            self.align_map_shift_limit,self.align_odom_shift_limit,
            self.align_map_hard_limit,self.align_odom_hard_limit)
        if unsafe:
            reason=(f'对齐时位置偏移超过硬限（map={map_shift:.3f}m, '
                    f'odom={odom_shift:.3f}m），已停车；禁止直接倒车')
            if self._hold_alignment_guard(now,'position',reason):
                return
        if suspect and not self.align_shift_reported:
            self.align_shift_reported=True
            n.get_logger().warning(
                f'对齐中检测到小幅位置变化（map={map_shift:.3f}m, '
                f'odom={odom_shift:.3f}m）；继续低速对齐，倒车前将严格复核基站走廊')
        map_turn=wrap(point[2]-self.map_start[2])
        odom_turn=wrap(self.odom[2]-self.start[2])
        if abs(wrap(map_turn-odom_turn))>.10:
            disagreement=abs(wrap(map_turn-odom_turn))
            reason=f'对齐时定位与轮式转角持续不一致（{disagreement:.3f}rad），已停车'
            if self._hold_alignment_guard(now,'heading',reason):
                return
        elif self.align_guard_since is not None:
            self.align_guard_since=None
            self.align_guard_count=0
            self.align_guard_key=None
            self.progress_at=now
        error=wrap(self.align_target[2]-point[2])
        remaining=abs(error)
        if remaining<=self.align_tolerance:
            self.stop()
            if self.settled_since is None:
                self.settled_since=now
            if now-self.settled_since>=1.0:
                self.active=False; self.kind=None
                self.begin('entry')
            return
        self.settled_since=None
        if self.align_best-remaining>=.015:
            self.align_best=remaining; self.progress_at=now
        if now-self.progress_at>self.align_progress_timeout:
            raise ValueError('对齐角度未持续改善，已停车；不因左右摇晃重置进度')
        msg=Twist()
        msg.angular.z=math.copysign(min(self.align_speed,max(.04,.5*remaining)),error)
        h.phase='aligning'; h.message=f'等待点独立对齐：剩余角度{math.degrees(error):.1f}°'
        n._state='dock_motion'; n._reason=h.message
        n._publish_lease(True)
        n._stop_publisher.publish(msg)

    def _hold_alignment_guard(self,now,key,reason):
        """Stop immediately; fail only when the same anomaly persists."""
        h=self.owner; n=h.node
        self.stop()
        if self.align_guard_key!=key:
            self.align_guard_key=key
            self.align_guard_since=now
            self.align_guard_count=1
        else:
            self.align_guard_count+=1
        h.phase='aligning'
        h.message='对齐保护确认中：'+reason
        n._state='dock_waiting'
        n._reason=h.message
        if (self.align_guard_count>=3
                and now-self.align_guard_since>=.6):
            raise ValueError(reason)
        return True

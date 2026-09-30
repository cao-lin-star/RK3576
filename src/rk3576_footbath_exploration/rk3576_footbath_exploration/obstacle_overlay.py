"""Stationary evidence and operator-managed map overlay; no motion commands."""
import hashlib
import json
import math
import os
import time
import uuid
from pathlib import Path
import yaml
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from .near_obstacle import StationaryConfirmation, near_echo


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class StopGate:
    """Fresh odometry, low measured twist AND bounded accumulated displacement."""
    def __init__(self):
        self.since = None
        self.anchor = None
        self.last = -math.inf
        self.stamp = -math.inf

    def update(self, now, stamp, age, pose, speed):
        valid = (all(math.isfinite(v) for v in (now, stamp, age, *pose, *speed))
                 and 0 <= age <= .25 and stamp > self.stamp)
        if not valid:
            self.since = None
            return
        gap = now-self.last
        self.last, self.stamp = now, stamp
        moving = math.hypot(*speed[:2]) > .005 or abs(speed[2]) > .01
        displaced = self.anchor is not None and (
            math.hypot(pose[0]-self.anchor[0], pose[1]-self.anchor[1]) > .008 or
            abs(angle(pose[2]-self.anchor[2])) > .015)
        if moving or gap > .25 or displaced or self.since is None:
            self.since = None if moving else now
            self.anchor = pose

    def ready(self, now):
        return self.since is not None and now-self.last <= .25 and now-self.since >= .8

    def after(self):
        # Settling + one full 66 ms long echo flight; echoes timestamped before
        # this boundary may never become static evidence.
        return math.inf if self.since is None else self.since + .9


def fingerprint(prefix):
    p = Path(str(prefix)+'.yaml')
    grid = yaml.safe_load(p.read_text(encoding='utf-8'))
    image = (p.parent / grid['image']).resolve()
    return dict(geometry=dict(resolution=grid['resolution'], origin=grid['origin']),
                image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())


def write_sidecar(prefix, zones, observations):
    data = dict(version=2, frame_id='map', zones=zones, observations=observations,
                **fingerprint(prefix))
    target = Path(str(prefix)+'.hazards.json')
    tmp = target.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    os.replace(tmp, target)


def load_sidecar(prefix):
    p = Path(str(prefix)+'.hazards.json')
    if not p.exists():
        return [], []
    data = json.loads(p.read_text(encoding='utf-8'))
    if data.get('frame_id') != 'map':
        raise ValueError('障碍坐标系无效')
    if data.get('version') == 2:
        fp = fingerprint(prefix)
        if any(data.get(k) != v for k,v in fp.items()):
            raise ValueError('障碍记录与当前地图不匹配，拒绝加载')
    zones, observations = data.get('zones', []), data.get('observations', [])
    for z in zones + observations:
        if (z.get('kind') not in ('cliff','near','manual','observe') or
                not all(isinstance(z.get(k), (int,float)) and math.isfinite(z[k]) for k in ('x','y','radius')) or
                not .01 <= z['radius'] <= .3):
            raise ValueError('障碍记录无效')
        if data.get('version') != 2 and z['kind'] != 'cliff':
            raise ValueError('旧版只允许加载台阶记录')
        # Older map sidecars may contain the retired manual-review flag.
        z.pop('needs_review', None)
        z['expires'] = 0.
        z.setdefault('id', uuid.uuid4().hex)
    if any(z['kind'] == 'observe' for z in zones) or any(z['kind'] != 'observe' for z in observations):
        raise ValueError('观察点不能混入规划障碍层')
    return zones, observations


def line_geometry(data):
    points=data.get('points')
    if not isinstance(points,list) or len(points)!=2:
        raise ValueError('请选择线段的起点和终点')
    a,b=[(float(p['x']),float(p['y'])) for p in points]
    r=float(data['radius'])
    if not all(math.isfinite(v) for v in (*a,*b,r)) or not .02<=r<=.15:
        raise ValueError('线条半宽应为2～15cm，坐标必须有效')
    length=math.hypot(b[0]-a[0],b[1]-a[1])
    return a,b,r,length


def segment_distance(x,y,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(x-a[0]-t*dx,y-a[1]-t*dy)


class ObstacleOverlay:
    def __init__(self, hazard, prefix=''):
        self.h = hazard
        self.n = hazard.n
        self.prefix = prefix
        self.session = uuid.uuid4().hex
        self.started = time.time()
        self.gate = StopGate()
        self.confirm = StationaryConfirmation()
        self.observations = []
        self.scans = {}
        self.grid = None
        self.grid_at = 0.
        self.tf_previous = None
        self.tf_anchor = None
        self.tf_stable_at = time.monotonic()
        self.tf_fresh_at = 0.
        self.review_required = False
        self.pending = None
        self.revision = 0
        self.last_publish = 0.
        self.deleted = []
        self.error = ''
        self.unapplied = None
        self.saved_signature = None
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.n.create_publisher(String, '/mapping/obstacle_overlay', qos)
        self.ack = self.n.create_publisher(String, '/mapping/obstacle_edit_result', 10)
        self.n.create_subscription(String, '/mapping/obstacle_edit', self.edit, 10)
        self.n.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.n.create_subscription(OccupancyGrid, '/map', self.map, qos)
        for key, topic in (('high','/scan_high'),('low','/scan_low_front')):
            self.n.create_subscription(LaserScan, topic, lambda m,k=key:self.scan(k,m), qos_profile_sensor_data)
        self.n.create_timer(.1, self.tick)

    def map(self, msg):
        self.grid, self.grid_at = msg, time.monotonic()

    def scan(self, key, msg):
        self.scans[key] = msg

    def odom(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds/1e9
        now = time.monotonic()
        p,q = msg.pose.pose.position, msg.pose.pose.orientation
        v,w = msg.twist.twist.linear, msg.twist.twist.angular
        self.gate.update(now,t,self.n.get_clock().now().nanoseconds/1e9-t,
                         (p.x,p.y,yaw(q)),(v.x,v.y,w.z))

    def stable(self, now):
        return (self.gate.ready(now) and now-self.tf_fresh_at <= .25 and
                now-self.tf_stable_at >= .8)

    def update_tf(self, now):
        try:
            tf = self.n.home.buffer.lookup_transform('map', 'odom',
                Time(nanoseconds=self.n.get_clock().now().nanoseconds-150000000))
            p,q = tf.transform.translation,tf.transform.rotation
            current = (p.x,p.y,yaw(q))
            if not all(math.isfinite(v) for v in current):
                raise ValueError('invalid transform')
            age = (self.n.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
            if not 0 <= age <= .5:
                return
            self.tf_fresh_at = now
            if self.tf_anchor is None:
                self.tf_anchor, self.tf_stable_at = current, now
            elif (math.hypot(current[0]-self.tf_anchor[0],current[1]-self.tf_anchor[1])>.03 or
                    abs(angle(current[2]-self.tf_anchor[2]))>.03):
                self.tf_anchor, self.tf_stable_at = current,now
                self.confirm.reset()
            # SLAM loop closure can correct map->odom during normal mapping.
            # Recollect settled evidence using the corrected TF automatically;
            # retain existing map marks and let the operator edit them if needed.
        except Exception:
            pass

    def lidar_clear(self, point, stamp):
        """Both live scanners must actually cover the ray with a farther finite hit.
        No return, missing coverage or stale data is UNKNOWN, never clear space.
        This label is evidence disagreement, not glass/material identification.
        """
        for key in ('high','low'):
            s = self.scans.get(key)
            if s is None or not s.ranges or s.angle_increment <= 0:
                return False
            st = Time.from_msg(s.header.stamp).nanoseconds/1e9
            if abs(st-stamp) > .25:
                return False
            try:
                tf = self.n.home.buffer.lookup_transform('map',s.header.frame_id,Time.from_msg(s.header.stamp))
                p,q = tf.transform.translation,tf.transform.rotation
                dx,dy = point[0]-p.x,point[1]-p.y
                distance = math.hypot(dx,dy)
                a = angle(math.atan2(dy,dx)-yaw(q))
                i = round((a-s.angle_min)/s.angle_increment)
                if i < 1 or i >= len(s.ranges)-1:
                    return False
                hits = s.ranges[i-1:i+2]
                if not all(math.isfinite(v) and s.range_min <= v <= s.range_max and v>distance+.10 for v in hits):
                    return False
            except Exception:
                return False
        return True

    def observe(self, now):
        if not self.stable(now):
            self.confirm.reset()
            return
        for key,(msg,acquired,stamp) in self.h.sonar_samples.items():
            if key != 'front' and not self.h.side_enabled:
                continue
            if not (0 <= now-acquired <= .6 and acquired > max(self.gate.after(), self.tf_stable_at+.9)
                    and near_echo(key,msg.range)):
                self.confirm.reset(key)
                continue
            try:
                tf = self.n.home.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                p,q = tf.transform.translation,tf.transform.rotation
                point = (p.x+msg.range*math.cos(yaw(q)),p.y+msg.range*math.sin(yaw(q)))
            except Exception:
                self.confirm.reset(key)
                continue
            point = self.confirm.observe(key,stamp,point)
            if point is None:
                continue
            if any(math.hypot(z['x']-point[0],z['y']-point[1]) <= .06 for z in self.h.zones+self.observations+self.deleted):
                continue
            if len(self.observations)>=128:
                self.error = '观察点已达128个上限，请暂停整理'
                continue
            self.observations.append(dict(id=uuid.uuid4().hex,kind='observe',x=point[0],y=point[1],
                radius=.04,expires=0.,source=key,lidar_clear=self.lidar_clear(point,stamp)))
            self.revision += 1

    def motion_blocked(self):
        return bool(self.pending or self.unapplied)

    def cancellation_done(self):
        future=getattr(self.n,'_overlay_cancel_future',None)
        try:
            if future is None or not future.done():
                return False
            result=future.result()
            return (result.return_code==0 and not self.n.home.active_goals and
                    (not result.goals_canceling or self.n.home.last_status>=self.n._overlay_cancel_at))
        except Exception:
            return False

    def editable(self, now):
        home = self.n.home
        return (self.stable(now) and self.n._state in ('paused_operator','paused_fault') and
                not self.h.active and not home.dock.active and not home.active_goals and
                self.cancellation_done() and self.h.source is not None and self.h.source < 2 and
                now-self.h.source_at <= .5 and now-self.h.manual_at > 1.)

    def reply(self, token, ok, message):
        self.ack.publish(String(data=json.dumps(dict(id=token,ok=ok,message=message),ensure_ascii=False)))

    def edit(self, msg):
        token = ''
        try:
            data = json.loads(msg.data)
            token = str(data['id'])
            if len(token)>64 or data.get('session')!=self.session or data.get('revision')!=self.revision:
                raise ValueError('地图会话或障碍版本已改变，请刷新后重试')
            now=time.monotonic()
            if abs(time.time()-float(data['sent_at']))>2.:
                raise ValueError('编辑请求已过期')
            if self.pending or self.unapplied or not self.editable(now):
                raise ValueError('请先暂停任务、松开遥控，等待停稳且导航取消完成')
            zones=[dict(z) for z in self.h.zones]
            observations=[dict(z) for z in self.observations]
            removed=[]
            op=data['op']
            if op in ('add','add_line'):
                if op=='add_line':
                    a,b,r,length=line_geometry(data)
                    self.validate_add(*a,r)
                    self.validate_add(*b,r)
                    count=max(1,math.ceil(length/r))
                    additions=[(a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count,r) for i in range(count+1)]
                else:
                    additions=[tuple(float(data[k]) for k in ('x','y','radius'))]
                # Validate the complete batch before publishing or persisting it.
                for x,y,r in additions:
                    self.validate_add(x,y,r)
                zones.extend(dict(id=uuid.uuid4().hex,kind='manual',x=x,y=y,radius=r,expires=0.) for x,y,r in additions)
            elif op=='delete_line':
                a,b,r,length=line_geometry(data)
                removed=[z for z in zones+observations if segment_distance(z['x'],z['y'],a,b)<=r+z['radius']]
                if not removed:
                    raise ValueError('线段未经过任何补充障碍记录')
                ids={z['id'] for z in removed}
                zones=[z for z in zones if z['id'] not in ids]
                observations=[z for z in observations if z['id'] not in ids]
            elif op=='delete':
                target=next((z for z in zones+observations if z['id']==data.get('obstacle_id')),None)
                if target is None:
                    raise ValueError('障碍不存在，请刷新')
                zones=[z for z in zones if z['id']!=target['id']]
                observations=[z for z in observations if z['id']!=target['id']]
                removed=[target]
            else:
                raise ValueError('未知编辑操作')
            if self.prefix:
                write_sidecar(self.prefix,zones,observations)
            if removed:
                self.deleted=(self.deleted+removed)[-128:]
            self.h.zones,self.observations=zones,observations
            self.revision+=1
            self.review_required=False
            stamp=self.h.publish_zones(now,force=True)
            self.pending=dict(id=token,stamp=stamp,at=now)
            self.unapplied=dict(stamp=stamp,at=now)
        except Exception as e:
            self.reply(token,False,str(e))

    def validate_add(self,x,y,r):
        if not all(math.isfinite(v) for v in (x,y,r)) or not .02<=r<=.15:
            raise ValueError('手动障碍半径应为2～15cm')
        g=self.grid
        if g is None or g.header.frame_id!='map' or g.info.resolution<=0:
            raise ValueError('当前地图未就绪')
        a=yaw(g.info.origin.orientation); dx=x-g.info.origin.position.x; dy=y-g.info.origin.position.y
        gx=(dx*math.cos(a)+dy*math.sin(a))/g.info.resolution
        gy=(-dx*math.sin(a)+dy*math.cos(a))/g.info.resolution
        if not 0<=gx<g.info.width or not 0<=gy<g.info.height:
            raise ValueError('位置超出当前地图')
        pose=self.n.home.current_pose()
        if pose is None or math.hypot(x-pose.pose.position.x,y-pose.pose.position.y)<r+.25:
            raise ValueError('不能在小车车身和安全边缘内添加障碍')

    def tick(self):
        if not hasattr(self.n,'home'):
            return
        now=time.monotonic()
        self.update_tf(now)
        self.observe(now)
        # Keep stop/echo observation at 10 Hz; persistence, deduplication and
        # full-list serialization need only the existing 2 Hz UI cadence.
        if now-self.last_publish < .5:
            return
        for z in self.h.zones:
            if 'id' not in z:
                z['id']=uuid.uuid4().hex
                self.revision+=1
        if self.unapplied:
            p=self.unapplied
            if all(k in self.h.applied and self.h.applied[k][0]>=p['stamp'] and
                   self.h.applied[k][1]>=p['at'] for k in ('local_costmap','global_costmap')):
                self.unapplied=None
        if self.pending:
            p=self.pending
            applied=all(k in self.h.applied and self.h.applied[k][0]>=p['stamp'] and
                        self.h.applied[k][1]>=p['at'] for k in ('local_costmap','global_costmap'))
            if applied or now-p['at']>4.:
                self.reply(p['id'],applied,'障碍修改已写入两张代价地图；请人工选择继续任务' if applied else
                           '修改已保存，但代价地图确认超时；保持暂停并检查，不要重复提交')
                self.pending=None
        retained=[z for z in self.observations if not any(
            math.hypot(z['x']-p['x'],z['y']-p['y'])<=.06 for p in self.h.zones)]
        if len(retained)!=len(self.observations):
            self.observations=retained
            self.revision+=1
        if self.prefix:
            signature=json.dumps([self.h.zones,self.observations],sort_keys=True,allow_nan=False)
            if signature!=self.saved_signature:
                try:
                    write_sidecar(self.prefix,self.h.zones,self.observations)
                    self.saved_signature=signature
                    self.error=''
                except Exception as e:
                    message='障碍保存失败：'+str(e)
                    if message!=self.error:
                        self.error=message
                        self.n._pause(self.n.PAUSED_FAULT,self.error)
        if now-self.last_publish<.5:
            return
        self.last_publish=now
        self.pub.publish(String(data=json.dumps(dict(session=self.session,started_at=self.started,
            revision=self.revision,mode='observe',editable=self.editable(now) and not self.pending and not self.unapplied,
            pending=bool(self.pending or self.unapplied),review_required=self.review_required,
            message=self.error,
            obstacles=self.h.zones,observations=self.observations),ensure_ascii=False,allow_nan=False)))

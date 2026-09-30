"""Session-scoped visualization and acknowledged operator edits."""
import json
import math
import time
import uuid
from nav_msgs.msg import Path
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class ObstacleView:
    def __init__(self, gateway):
        self.g=gateway
        self.overlay={}
        self.seen=0.
        self.explore={}
        self.explore_seen=0.
        self.route=None
        self.pending={}
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        gateway.create_subscription(String,'/mapping/obstacle_overlay',self.on_overlay,qos)
        gateway.create_subscription(String,'/explore/view',self.on_explore,10)
        gateway.create_subscription(Path,'/plan',self.on_plan,10)
        gateway.create_subscription(String,'/mapping/obstacle_edit_result',self.on_result,10)
        self.pub=gateway.create_publisher(String,'/mapping/obstacle_edit',10)
        gateway.create_timer(.2,self.expire)

    def on_overlay(self,msg):
        try:
            data=json.loads(msg.data)
            if self.g.mode=='idle' or data['started_at']<self.g.mode_started_at:
                return
            self.overlay,self.seen=data,time.monotonic()
        except (ValueError,KeyError,TypeError):
            return

    def on_explore(self,msg):
        try:
            data=json.loads(msg.data)
            now=self.g.get_clock().now().nanoseconds/1e9
            if self.g.mode!='auto_mapping' or not 0<=now-data['stamp']<2 or data['stamp']<self.g.mode_started_at:
                return
            old=(self.explore.get('goal') or {}).get('id')
            new=(data.get('goal') or {}).get('id')
            if old!=new:
                self.route=None
            self.explore,self.explore_seen=data,time.monotonic()
        except (ValueError,KeyError,TypeError):
            return

    def on_plan(self,msg):
        goal=self.explore.get('goal')
        if not goal or self.g.mode!='auto_mapping' or msg.header.frame_id!='map' or not msg.poses:
            self.route=None
            return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        now=self.g.get_clock().now().nanoseconds/1e9
        end=msg.poses[-1].pose.position
        if not (goal['sent_at']<=stamp<=now and now-stamp<2 and
                math.hypot(end.x-goal['x'],end.y-goal['y'])<=.5):
            self.route=None
            return
        if any(p.header.frame_id not in ('','map') or not math.isfinite(p.pose.position.x) or
               not math.isfinite(p.pose.position.y) for p in msg.poses):
            self.route=None
            return
        step=max(1,math.ceil(len(msg.poses)/800))
        points=[dict(x=p.pose.position.x,y=p.pose.position.y) for p in msg.poses[::step]]
        points.append(dict(x=end.x,y=end.y))
        self.route=dict(goal_id=goal['id'],at=time.monotonic(),points=points)

    def view(self):
        now=time.monotonic()
        active=(self.g.mode!='idle' and now-self.seen<2. and
                self.overlay.get('started_at',0)>=self.g.mode_started_at)
        data=dict(self.overlay) if active else dict(obstacles=[],observations=[],editable=False)
        data['fresh']=active
        data['editable']=bool(active and data.get('editable') and not self.pending and not self.g.manual_active)
        explore=(self.explore if self.g.mode=='auto_mapping' and now-self.explore_seen<2.
                 and self.explore.get('stamp',0)>=self.g.mode_started_at else {})
        goal=explore.get('goal')
        # Hide stopped/cancelled targets immediately, even before explorer heartbeat.
        if self.g.home_status.get('supervisor_state')!='running':
            goal=None
        data['goal']=goal
        data['unreachable']=explore.get('unreachable',[])
        data['route']=(self.route['points'] if goal and self.route and
                       self.route['goal_id']==goal['id'] else [])
        return data

    def submit(self,data,event,box):
        view=self.view()
        if not view.get('editable'):
            raise ValueError('请先暂停并等待小车停稳；当前不能编辑障碍')
        if data.get('session')!=view.get('session') or data.get('revision')!=view.get('revision'):
            raise ValueError('显示的地图或障碍已更新，请刷新后重试')
        token=uuid.uuid4().hex
        payload={k:data[k] for k in ('session','revision','op','x','y','radius','obstacle_id','points') if k in data}
        payload.update(id=token,sent_at=time.time())
        self.pending[token]=(event,box,time.monotonic())
        self.pub.publish(String(data=json.dumps(payload,allow_nan=False)))

    def on_result(self,msg):
        try:
            d=json.loads(msg.data)
            item=self.pending.pop(d['id'],None)
            if item is None:
                return
            event,box,_=item
            if d['ok']:
                box['result']=d
            else:
                box['error']=d['message']
            event.set()
        except (ValueError,KeyError,TypeError):
            pass

    def expire(self):
        for token,(event,box,at) in list(self.pending.items()):
            if time.monotonic()-at>5.:
                box['error']='障碍编辑确认超时，请刷新核对结果，保持暂停'
                event.set()
                del self.pending[token]

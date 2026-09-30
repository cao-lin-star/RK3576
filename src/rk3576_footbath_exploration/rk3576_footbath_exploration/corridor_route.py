"""Frozen, monotonic traversal of actual odometry positions; no inferred shortcut."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FrozenRoute:
    def __init__(self, records, now, current, maximum=6.):
        self.points=[tuple(current[:2])]
        length=0.
        for stamp,pose,room in reversed(list(records)):
            if not all(math.isfinite(v) for v in (stamp,*pose)) or not 0<=now-stamp<=180.:
                break
            last=self.points[-1];gap=math.hypot(pose[0]-last[0],pose[1]-last[1])
            if gap>.08: break
            if gap<.005: continue
            if length+gap>maximum: break
            self.points.append(tuple(pose[:2]));length+=gap
            if length>=.4 and room: break
        if length<.08: raise ValueError('连续已走曲线不足8cm，不能凭雷达空旷推断后方地面')
        self.arc=[0.]
        for a,b in zip(self.points,self.points[1:]):self.arc.append(self.arc[-1]+math.dist(a,b))
        self.progress=0.
        self.length=self.arc[-1]

    def at(self, distance):
        distance=max(0.,min(self.length,distance))
        for i,(lo,hi) in enumerate(zip(self.arc,self.arc[1:])):
            if distance<=hi:
                t=(distance-lo)/(hi-lo);a,b=self.points[i:i+2]
                return (a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
        return self.points[-1]

    def command(self, pose, direction):
        if direction not in (-1,1) or not all(math.isfinite(v) for v in pose):
            raise ValueError('曲线路径位姿无效')
        best=None
        for i,(a,b) in enumerate(zip(self.points,self.points[1:])):
            if self.arc[i+1]<self.progress-.02 or self.arc[i]>self.progress+.12:continue
            dx,dy=b[0]-a[0],b[1]-a[1];length=self.arc[i+1]-self.arc[i]
            t=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/(length*length)))
            progress=self.arc[i]+t*length
            if progress>self.progress+.12:continue
            error=math.hypot(pose[0]-a[0]-t*dx,pose[1]-a[1]-t*dy)
            if best is None or error<best[0]:best=(error,progress,i)
        if best is None or best[0]>.025:raise ValueError('偏离已走曲线超过2.5cm，停车')
        if best[1]<self.progress-.025:raise ValueError('退出方向错误，停车')
        self.progress=max(self.progress,best[1])
        if self.length-self.progress<.02 and math.dist(pose[:2],self.points[-1])<.025:
            return 0.,0.,True
        target=self.at(self.progress+.10)
        dx,dy=target[0]-pose[0],target[1]-pose[1]
        local_x=dx*math.cos(pose[2])+dy*math.sin(pose[2])
        local_y=-dx*math.sin(pose[2])+dy*math.cos(pose[2])
        error=wrap(math.atan2(dy,dx)-pose[2]-(math.pi if direction<0 else 0.))
        if abs(error)>.6 or direction*local_x<=0:
            raise ValueError('车头与已走曲线偏差过大，不能强行转入退路')
        v=direction*.03
        curvature=2*local_y/max(.0025,dx*dx+dy*dy)
        # Both wheels retain the commanded translation direction (track .33m).
        if abs(curvature)>4.:raise ValueError('历史曲线弯曲过急，停车')
        return v,v*curvature,False


def swept_centers(v,w,seconds=1.5):
    result=[]
    for i in range(16):
        t=seconds*i/15
        result.append((v*t,0.) if abs(w)<1e-6 else (v/w*math.sin(w*t),v/w*(1-math.cos(w*t))))
    return result

"""Recorded straight-corridor retreat. No blind turns or unseen reverse paths."""
import math
from collections import deque
from .dock_motion import segment_error


class RetreatTrace:
    def __init__(self):
        self.points = deque()
        self.last_observation = None

    def clear(self):
        self.points.clear()
        self.last_observation = None

    def observe(self, now, pose, safe, turning_room=False):
        if not safe or pose is None or not all(math.isfinite(v) for v in pose):
            self.clear()
            return
        if self.last_observation is not None:
            t, previous = self.last_observation
            if now-t > .6 or math.hypot(pose[0]-previous[0],pose[1]-previous[1]) > .12:
                self.clear()
        self.last_observation = (now, pose)
        while self.points and now-self.points[0][0] > 180.:
            self.points.popleft()
        if not self.points or math.hypot(pose[0]-self.points[-1][1][0],pose[1]-self.points[-1][1][1]) >= .02:
            # A return along the same path must not turn oscillations into extra retreat length.
            match = next((i for i,p in enumerate(list(self.points)[:-2])
                          if math.hypot(pose[0]-p[1][0],pose[1]-p[1][1]) < .025), None)
            if match is not None:
                while len(self.points) > match:
                    self.points.pop()
            self.points.append((now, tuple(pose), turning_room))
        while len(self.points) > 400:
            self.points.popleft()

    def hold(self, now, pose):
        """Preserve the safe suffix only while stationary during obstacle stop."""
        if pose is None or self.last_observation is None:
            self.clear()
            return
        _, previous = self.last_observation
        if (not all(math.isfinite(v) for v in pose) or
                math.hypot(pose[0]-previous[0],pose[1]-previous[1]) > .025):
            self.clear()
            return
        # Keep the anchor fixed: small repeated movements cannot bridge unsafe ground.
        self.last_observation = (now, previous)
        while self.points and now-self.points[0][0] > 180.:
            self.points.popleft()

    def plan(self, now, current, minimum=.25, maximum=1.):
        candidates = []
        previous = current
        reason = "安全轨迹为空或长度不足"
        longest = 0.
        for stamp, point, room in reversed(self.points):
            if now-stamp > 180.:
                reason = "安全轨迹已超过180秒"
                break
            if math.hypot(point[0]-previous[0], point[1]-previous[1]) > .08:
                reason = "轨迹点间隔超过8cm"
                break
            progress, cross, heading = segment_error(current, point, -1)
            # Historical yaw is not the direction of this reverse command.
            # The actual recorded positions must still fit the current straight corridor.
            if progress < -.025 or abs(cross) > .025:
                reason = f"轨迹不在当前后退走廊内（横偏{abs(cross)*100:.1f}cm）"
                break
            if progress > maximum+.005:
                break
            previous = point
            longest = max(longest, progress)
            if progress >= minimum:
                candidates.append((progress, room))
                if room:
                    return min(progress, maximum)
        if candidates:
            return min(candidates[-1][0], maximum)
        raise ValueError(f"{reason}；可用直退{longest*100:.1f}cm，需要至少{minimum*100:.0f}cm")


def tof_retreat_allowed(now, state, state_at, last_good, sonar):
    if now-state_at > .35 or state & 12 != 12:
        return False  # no decoded frames: unplugged/unpowered/serial failure
    missing = [k for i,k in enumerate(('left','right')) if not state & (1<<i)]
    if not missing:
        return False
    if any(k not in last_good or now-last_good[k][1] > 2. or not 0 < last_good[k][0] <= .26
           for k in ('left','right')):
        return False
    return sonar <= .30 or any(last_good[k][0] <= .08 for k in missing)


class MotionWindow:
    """Fixed-time displacement/oscillation guard; goal ownership resets the window."""
    def __init__(self):
        self.samples=deque()
        self.key=None

    def update(self, key, now, pose, duration=45.):
        if key is None or key!=self.key:
            self.samples.clear();self.key=key
        if key is None: return False
        if self.samples and now<self.samples[-1][0]:self.samples.clear()
        self.samples.append((now,pose[0],pose[1]))
        while len(self.samples)>2 and now-self.samples[1][0]>=duration:
            self.samples.popleft()
        if now-self.samples[0][0]<duration:return False
        start=self.samples[0]
        net=math.hypot(pose[0]-start[1],pose[1]-start[2])
        extent=max(math.hypot(s[1]-start[1],s[2]-start[2]) for s in self.samples)
        travelled=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(self.samples,list(self.samples)[1:]))
        return extent<.05 or (travelled>=.4 and extent<=.6 and net<.20 and net<travelled*.2)


class RetreatEpisode:
    """Retry budget ends only after normal, healthy travel leaves the stuck pose."""
    def __init__(self):
        self.anchor=None
        self.attempts=0
        self.fallbacks=0
        self.travel_start=None
        self.previous=None

    def begin(self,pose):
        if self.anchor is None:
            self.anchor=tuple(pose)
        self.travel_start=None
        self.previous=None

    def observe(self,now,pose,safe):
        if self.anchor is None:
            return False
        if not safe or pose is None or not all(math.isfinite(v) for v in (now,*pose)):
            self.travel_start=None;self.previous=None
            return False
        if self.previous is not None:
            t,p=self.previous
            if now<t or now-t>.6 or math.hypot(pose[0]-p[0],pose[1]-p[1])>.12:
                self.travel_start=None
        self.previous=(now,tuple(pose))
        if self.travel_start is None:
            self.travel_start=(now,tuple(pose))
            return False
        t,p=self.travel_start
        progressed=math.hypot(pose[0]-p[0],pose[1]-p[1])>=.30
        departed=math.hypot(pose[0]-self.anchor[0],pose[1]-self.anchor[1])>=.80
        if now-t>=3. and progressed and departed:
            self.__init__()
            return True
        return False

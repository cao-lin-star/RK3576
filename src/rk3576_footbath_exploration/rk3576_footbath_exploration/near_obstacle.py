"""Material-agnostic near-obstacle evidence. No lidar comparison or cone painting."""
import math

SIDE_STOP_M = 0.12  # distance measured from probe face
FRONT_STOP_M = 0.20  # stable firmware threshold
MAX_AGE_S = 0.60


def near_echo(key, distance):
    limit = FRONT_STOP_M if key == 'front' else SIDE_STOP_M
    return math.isfinite(distance) and 0.02 <= distance <= limit


def recovery_near_echo(key, distance):
    """Only for a probe already being confirmed after a recovery stop.

    Front echoes in the stop/release hysteresis band still describe an
    unresolved obstacle. This must never be used to trigger normal travel stops.
    """
    if key == 'front':
        return math.isfinite(distance) and .02 <= distance < FRONT_CLEAR_M
    return near_echo(key, distance)


STATIONARY_RECORD_M = .50  # Record range only; never changes motion protection.

def record_echo(distance):
    return math.isfinite(distance) and .02 <= distance <= STATIONARY_RECORD_M


class StationaryConfirmation:
    """Three distinct post-stop echoes at a spatially stable point.

    A moving scan, repeated timer tick, no echo, or missing exact-time TF
    cannot authorize a remembered obstacle. Points are already in map frame.
    """
    def __init__(self):
        self.echoes = {}

    def reset(self, key=None):
        if key is None:
            self.echoes.clear()
        else:
            self.echoes.pop(key, None)

    def observe(self, key, stamp, point):
        if not all(math.isfinite(v) for v in (stamp, *point)):
            self.reset(key)
            return None
        previous = self.echoes.get(key, [])
        if previous and stamp <= previous[-1][0]:
            if stamp == previous[-1][0] and len(previous) == 3:
                return (sum(p[1] for p in previous)/3, sum(p[2] for p in previous)/3)
            return None
        if previous and (stamp - previous[-1][0] > MAX_AGE_S or any(
                math.hypot(point[0]-p[1], point[1]-p[2]) > .025 for p in previous)):
            previous = []
        previous = (previous + [(stamp, *point)])[-3:]
        self.echoes[key] = previous
        if len(previous) < 3:
            return None
        return (sum(p[1] for p in previous)/3, sum(p[2] for p in previous)/3)


def merge_session_obstacles(zones, pending):
    """Keep a fixed, small mark; never expand a cloud on repeated encounters.

    Confirmed sonar marks live only in this running map session. Cliff save/load
    remains separate. At most two completed recoveries at one remembered point.
    """
    result = [dict(z) for z in zones]
    unique = []
    for point in pending:
        if not any(math.hypot(p['x']-point['x'], p['y']-point['y']) <= .06 for p in unique):
            unique.append(point)
    for point in unique:
        match = next((z for z in result if z['kind'] == 'near' and
                      math.hypot(z['x']-point['x'], z['y']-point['y']) <= .06), None)
        if match is not None:
            if match.get('encounters', 1) >= 2:
                raise ValueError('同一近障位置重复受阻，停止自动尝试，请人工处理')
            match['encounters'] = match.get('encounters', 1) + 1
        else:
            result.append(dict(point, encounters=1))
    return result


class ClearConfirmation:
    """Three distinct finite farther echoes; no-return is not clear evidence."""
    def __init__(self):
        self.samples = {}

    def reset(self, key=None):
        if key is None: self.samples.clear()
        else: self.samples.pop(key, None)

    def observe(self, key, stamp, distance):
        limit = FRONT_CLEAR_M if key == 'front' else SIDE_STOP_M
        blocked = distance < limit if key == 'front' else distance <= limit
        if not math.isfinite(distance) or blocked or distance > 4.:
            self.reset(key)
            return False
        previous = self.samples.get(key, [])
        if previous and stamp < previous[-1]:
            self.reset(key)
            return False
        if previous and stamp == previous[-1]:
            return len(previous) >= 3 and previous[-1]-previous[0] >= .3
        if previous and stamp-previous[-1] > MAX_AGE_S: previous = []
        previous=(previous+[stamp])[-3:]
        self.samples[key]=previous
        return len(previous)>=3 and previous[-1]-previous[0]>=.3


class WindowedConfirmation:
    """Three spatially consistent near echoes among at most five in two seconds.

    Only finite farther echoes may interrupt the sequence without resetting it.
    Missing data, motion, invalid TF and no-return are reset by the caller.
    """
    def __init__(self):
        self.events={}

    def reset(self,key=None):
        if key is None:self.events.clear()
        else:self.events.pop(key,None)

    def observe(self,key,stamp,point):
        if not math.isfinite(stamp) or (point is not None and not all(math.isfinite(v) for v in point)):
            self.reset(key);return None
        old=self.events.get(key,[])
        if old and stamp<old[-1][0]:
            self.reset(key);return None
        if old and stamp==old[-1][0]:
            return self.result(key,stamp)
        if old and stamp-old[-1][0]>MAX_AGE_S:old=[]
        old=[e for e in old if stamp-e[0]<=2.]
        if point is not None and any(p is not None and math.hypot(point[0]-p[0],point[1]-p[1])>.025 for _,p in old):
            old=[]
        self.events[key]=(old+[(stamp,point)])[-5:]
        return self.result(key,stamp)

    def result(self,key,stamp):
        events=self.events.get(key,[])
        points=[p for t,p in events if p is not None and 0<=stamp-t<=2.]
        if not events or stamp-events[-1][0]>MAX_AGE_S or len(points)<3 or len(points)*5<len(events)*3:
            return None
        return tuple(sum(p[i] for p in points)/len(points) for i in (0,1))

# Shared release threshold for every automatic retreat owner and fallback.
FRONT_CLEAR_M = .28
FRONT_PREFERRED_M = .35

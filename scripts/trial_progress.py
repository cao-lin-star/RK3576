"""Pure trial-only motion window, scoped to a single navigation goal."""
from collections import deque
import math


class TrialProgress:
    def __init__(self):
        self.goal = None
        self.poses = deque(maxlen=5000)

    def set_goal(self, goal):
        if goal != self.goal:
            self.goal = goal
            self.poses.clear()

    def observe(self, now, x, y, yaw):
        self.poses.append((now, x, y, yaw))
        while self.poses and now-self.poses[0][0]>30:
            self.poses.popleft()

    def summary(self):
        p=self.poses
        if len(p)<2:
            return 0., 0., False
        distance=math.hypot(p[-1][1]-p[0][1],p[-1][2]-p[0][2])
        turn=sum(abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3])))
                 for a,b in zip(p,list(p)[1:]))
        # A recovery or goal transition must not inherit another goal's motion.
        looping=self.goal is not None and p[-1][0]-p[0][0]>28 and distance<.20 and turn>math.pi*1.5
        return distance,turn,looping

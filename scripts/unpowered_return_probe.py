#!/usr/bin/env python3
"""Requires explicit operator confirmation that 24V motor power is OFF."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray
from action_msgs.msg import GoalStatusArray


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--motor-power-off-confirmed',action='store_true',required=True)
    p.parse_args()
    rclpy.init(args=[]); n=Node('unpowered_return_probe')
    healthy=[False]; lease=[False]; commands=[]; selected=[]; motion=[0.,0.]
    active={'nav':True,'path':True}
    def diag(m):
        for s in m.status:
            if s.name=='rk3576_footbath/exploration_supervisor':
                healthy[0]=dict((x.key,x.value) for x in s.values).get('inputs_healthy')=='True'
    def odom(m):
        motion[0]=max(motion[0],abs(m.twist.twist.linear.x))
        motion[1]=max(motion[1],abs(m.twist.twist.angular.z))
    n.create_subscription(DiagnosticArray,'/diagnostics',diag,10)
    n.create_subscription(Odometry,'/odom',odom,10)
    n.create_subscription(Twist,'/cmd_vel_nav',lambda m:commands.append((m.linear.x,m.angular.z)),10)
    n.create_subscription(Twist,'/cmd_vel_selected',lambda m:selected.append((m.linear.x,m.angular.z)),10)
    n.create_subscription(Bool,'/safety/auto_motion_lease',lambda m:lease.__setitem__(0,m.data),10)
    for key,topic in [('nav','/navigate_to_pose/_action/status'),('path','/follow_path/_action/status')]:
        n.create_subscription(GoalStatusArray,topic,
            lambda m,k=key:active.__setitem__(k,any(s.status in (1,2,3) for s in m.status_list)),10)
    stop=n.create_client(Trigger,'/exploration/stop')
    start=n.create_client(Trigger,'/exploration/return_home')
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if motion[0]>.01 or motion[1]>.02:
                raise RuntimeError('Unexpected odometry motion; stopping test')
    try:
        end=time.monotonic()+45
        while not healthy[0] and time.monotonic()<end: spin(.2)
        assert healthy[0] and stop.wait_for_service(timeout_sec=3)
        assert start.wait_for_service(timeout_sec=3)
        f=start.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n,f,timeout_sec=4)
        assert f.done() and f.result().success
        spin(5)
    finally:
        if stop.wait_for_service(timeout_sec=3):
            f=stop.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        end=time.monotonic()+4
        while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.1)
        print(json.dumps(dict(nav_samples=len(commands),
            nonzero_nav=sum(abs(v)+abs(w)>1e-5 for v,w in commands),
            nonzero_selected=sum(abs(v)+abs(w)>1e-5 for v,w in selected),
            max_odom_motion=motion,lease_after_stop=lease[0],active_after_stop=active,
            last_selected=selected[-3:]),indent=2))
        n.destroy_node();rclpy.shutdown()
    assert commands and any(abs(v)+abs(w)>1e-5 for v,w in commands), 'No computed control output'
    assert not lease[0] and not any(active.values()), 'Stop not confirmed'


if __name__=='__main__': main()

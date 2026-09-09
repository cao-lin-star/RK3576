#!/usr/bin/env python3
"""Passive return inspection; never publishes commands or calls services."""
import argparse
import json
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray


def main():
    p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=20)
    args=p.parse_args(); rclpy.init(); n=rclpy.create_node('passive_return_observer')
    homes=[]; leases=[]; commands=[]; odom=[]; diagnostics={}
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def home(m):
        d=json.loads(m.data)
        if not homes or (d['phase'],d.get('recovery_count')) != (homes[-1]['phase'],homes[-1].get('recovery_count')):
            print(json.dumps(d,ensure_ascii=False),flush=True)
        homes.append(d)
    def diag(m):
        for s in m.status:
            if 'supervisor' in s.name or 'base' in s.name:
                diagnostics[s.name]=dict(message=s.message,values={v.key:v.value for v in s.values})
    n.create_subscription(String,'/mapping/home_status',home,qos)
    n.create_subscription(Bool,'/safety/auto_motion_lease',lambda m:leases.append(m.data),10)
    n.create_subscription(Twist,'/cmd_vel_selected',lambda m:commands.append((m.linear.x,m.angular.z)),10)
    n.create_subscription(Odometry,'/odom',lambda m:odom.append((m.pose.pose.position.x,m.pose.pose.position.y)),10)
    n.create_subscription(DiagnosticArray,'/diagnostics',diag,10)
    end=time.monotonic()+args.seconds
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.1)
    print(json.dumps(dict(home=homes[-1] if homes else None,lease_last=leases[-1] if leases else None,
        leases_true=sum(leases),commands=len(commands),nonzero_commands=sum(abs(v)+abs(w)>1e-6 for v,w in commands),
        odom_first=odom[0] if odom else None,odom_last=odom[-1] if odom else None,
        nodes=n.get_node_names(),diagnostics=diagnostics),ensure_ascii=False),flush=True)
    n.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()

#!/usr/bin/env python3
"""Read-only scan evidence. No velocity publishers and no base/serial driver."""
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    proc = subprocess.Popen(['ros2', 'launch', 'rk3576_footbath_bringup',
                             'hardware.launch.py', 'start_base:=false'],
                            start_new_session=True, stdout=subprocess.DEVNULL,
                            stderr=subprocess.STDOUT)
    rclpy.init()
    node = Node('dock_passive_scan_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {}

    def receive(msg, topic):
        try:
            tf = buffer.lookup_transform('base_footprint', msg.header.frame_id, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            points = []
            for i, r in enumerate(msg.ranges):
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    a = msg.angle_min+i*msg.angle_increment+yaw
                    points.append((t.x+r*math.cos(a), t.y+r*math.sin(a)))
            hits = [(round(x, 3), round(y, 3)) for x, y in points
                    if x >= 0 and math.hypot(x-min(.05, x), y) < .23]
            samples.setdefault(topic, []).append(dict(blocked=hits[:12],
                rear_count=sum(x < 0 for x, y in points), valid=len(points),
                inside_body=sum(math.hypot(x,y)<.22 for x,y in points)))
        except Exception:
            pass

    for topic in ('/scan_high', '/scan_low_front'):
        node.create_subscription(LaserScan, topic,
            lambda m, topic=topic: receive(m, topic), qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+22
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        print(json.dumps({key: dict(frames=len(value),
            blocked_frames=sum(bool(s['blocked']) for s in value), last=value[-3:])
            for key,value in samples.items()}, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)


if __name__ == '__main__':
    main()

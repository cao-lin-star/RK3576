#!/usr/bin/env python3
"""Save the live, stopped mapping session for an explicit gateway update.

Does not request motion. Restore is separately authorized via a ROS service.
"""
import json
import argparse
import math
import time
from pathlib import Path
import rclpy
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from slam_toolbox.srv import SaveMap
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--saved-map',default='')
    args=parser.parse_args()
    rclpy.init(args=[])
    n = rclpy.create_node('capture_localization_handoff')
    buffer = Buffer()
    listener = TransformListener(buffer, n)
    status = []
    odom = []
    lease = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(String, '/mapping/home_status', lambda m: status.append(json.loads(m.data)), qos)
    n.create_subscription(Odometry, '/odom', lambda m: odom.append((time.monotonic(),m)), 10)
    n.create_subscription(Bool, '/safety/auto_motion_lease', lambda m: lease.append(m.data), 10)

    def wait(seconds):
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(n, timeout_sec=.1)

    wait(3)
    assert status and status[-1]['available'], 'missing original home'
    original = status[-1]['pose']
    stop = n.create_client(Trigger, '/exploration/stop')
    assert stop.wait_for_service(timeout_sec=5)
    future = stop.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(n, future, timeout_sec=8)
    assert future.done() and future.result().success
    odom.clear(); lease.clear(); wait(3)
    assert len(odom)>10 and lease and not any(lease), 'not safely stopped'
    assert all(abs(m.twist.twist.linear.x)<.01 and abs(m.twist.twist.angular.z)<.02 for _,m in odom)
    prefix = '/home/sky/rk3576_footbath_ws/maps/return_saved_'+time.strftime('%Y%m%d_%H%M%S')
    if args.saved_map:
        saved=Path(args.saved_map).resolve()
        assert saved.parent==Path('/home/sky/rk3576_footbath_ws/maps') and saved.suffix=='.yaml' and saved.is_file()
        prefix=str(saved.with_suffix(''))
    else:
        client = n.create_client(SaveMap, '/slam_toolbox/save_map')
        assert client.wait_for_service(timeout_sec=5)
        req = SaveMap.Request(); req.name.data = prefix
        future = client.call_async(req)
        rclpy.spin_until_future_complete(n, future, timeout_sec=45)
        assert future.done() and future.result().result == SaveMap.Response.RESULT_SUCCESS
    wait(1)
    tf = buffer.lookup_transform('map','base_footprint',Time())
    age = (n.get_clock().now().nanoseconds-Time.from_msg(tf.header.stamp).nanoseconds)/1e9
    assert 0<=age<1 and time.monotonic()-odom[-1][0]<1
    assert all(abs(m.twist.twist.linear.x)<.01 and abs(m.twist.twist.angular.z)<.02 for _,m in odom)
    p,q = tf.transform.translation, tf.transform.rotation
    data = dict(pose=original, current_pose=dict(x=p.x,y=p.y,
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))),
        map_path=prefix+'.yaml', updated_at=time.monotonic(), auto_request=False,
        boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    assert Path(data['map_path']).is_file()
    Path('/home/sky/rk3576_footbath_ws/maps/pending_return_handoff.json').write_text(json.dumps(data))
    print(json.dumps(data))
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

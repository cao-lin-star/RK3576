#!/usr/bin/env python3
"""Read only a completed navigation trial bag; no ROS node or robot connection.

Run after the recorder exits. Uses ROS message deserializers but never initializes
DDS. Outputs per-action command/odometry statistics and correlated event windows.
"""
import argparse
from bisect import bisect_right
from collections import defaultdict
from datetime import datetime, timedelta, timezone
import json
import math
from pathlib import Path
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

VELOCITIES = ['/cmd_vel_nav', '/cmd_vel', '/cmd_vel_auto_limited',
              '/cmd_vel_selected', '/cmd_vel_recovery', '/odom']
WANTED = set(VELOCITIES + ['/tf', '/amcl_pose', '/initialpose', '/plan',
    '/local_plan', '/navigate_to_pose/_action/status', '/navigate_to_pose/_action/feedback',
    '/mapping/home_status', '/mobile/navigation_context', '/diagnostics',
    '/range/ultrasonic', '/range/ultrasonic_front_observe', '/range/ultrasonic_left',
    '/range/ultrasonic_right', '/range/tof_left', '/range/tof_right',
    '/range/suspected_glass', '/range/suspected_glass_left', '/range/suspected_glass_right',
    '/safety/auto_motion_lease', '/safety/recovery_active', '/safety/suspected_glass_valid',
    '/chassis/control_source', '/rosout'])


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def stamp(t):
    return t.sec+t.nanosec/1.e9


def wall(t):
    return datetime.fromtimestamp(t, timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')


def quantiles(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    return {str(p): round(values[min(len(values)-1, int((len(values)-1)*p))], 5)
            for p in (0, .1, .5, .9, 1)}


def read(folder):
    if not (folder/'metadata.yaml').is_file():
        raise ValueError('completed bag metadata.yaml required; do not inspect an active bag')
    series = defaultdict(list)
    counts = defaultdict(int)
    classes = {}
    for file in sorted(folder.glob('*.db3')):
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro', uri=True) as db:
            topics = {i: (name, kind) for i, name, kind in db.execute('select id,name,type from topics')}
            for i, count in db.execute('select topic_id,count(*) from messages group by topic_id'):
                counts[topics[i][0]] += count
            tids = [i for i, (name, _) in topics.items() if name in WANTED]
            query = 'select topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in tids)+') order by timestamp'
            for i, ns, raw in db.execute(query, tids):
                name, kind = topics[i]
                if kind not in classes:
                    classes[kind] = get_message(kind)
                msg = deserialize_message(raw, classes[kind])
                t = ns/1.e9
                if name in VELOCITIES:
                    twist = msg.twist.twist if name == '/odom' else msg
                    value = dict(v=float(twist.linear.x), w=float(twist.angular.z))
                    if name == '/odom':
                        value.update(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                                     yaw=yaw(msg.pose.pose.orientation), stamp=stamp(msg.header.stamp))
                elif name == '/tf':
                    for tf in msg.transforms:
                        child = tf.child_frame_id; parent = tf.header.frame_id
                        if (parent, child) in (('map', 'odom'), ('odom', 'base_footprint')):
                            series[parent+'->'+child].append((t, dict(x=tf.transform.translation.x,
                                y=tf.transform.translation.y, yaw=yaw(tf.transform.rotation),
                                stamp=stamp(tf.header.stamp))))
                    continue
                elif name in ('/amcl_pose', '/initialpose'):
                    value = dict(x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                                 yaw=yaw(msg.pose.pose.orientation), stamp=stamp(msg.header.stamp),
                                 covariance=[float(msg.pose.covariance[k]) for k in (0, 7, 35)])
                elif name in ('/plan', '/local_plan'):
                    points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
                    value = dict(length=length, points=len(points), frame=msg.header.frame_id,
                                 start=points[0] if points else None, end=points[-1] if points else None)
                    if len(points)>1:
                        aim = next((p for p in points if math.dist(p, points[0])>.5), points[-1])
                        value['heading'] = math.atan2(aim[1]-points[0][1], aim[0]-points[0][0])
                elif name.endswith('/_action/status'):
                    value = [dict(uuid=bytes(s.goal_info.goal_id.uuid).hex(), status=int(s.status),
                                  stamp=stamp(s.goal_info.stamp)) for s in msg.status_list]
                elif name.endswith('/_action/feedback'):
                    fb = msg.feedback
                    value = dict(uuid=bytes(msg.goal_id.uuid).hex(), distance=float(fb.distance_remaining),
                                 recoveries=int(fb.number_of_recoveries),
                                 x=fb.current_pose.pose.position.x, y=fb.current_pose.pose.position.y,
                                 yaw=yaw(fb.current_pose.pose.orientation))
                elif name in ('/mapping/home_status', '/mobile/navigation_context'):
                    try:
                        value = json.loads(msg.data)
                    except ValueError:
                        continue
                elif name == '/diagnostics':
                    value = {s.name: dict(message=s.message, level=(s.level[0] if isinstance(s.level, bytes) else int(s.level)),
                             values={v.key: v.value for v in s.values}) for s in msg.status}
                elif name == '/rosout':
                    if msg.level < 30:
                        continue
                    value = dict(level=int(msg.level), node=msg.name, message=msg.msg)
                elif name.startswith('/range/'):
                    value = dict(range=float(msg.range), stamp=stamp(msg.header.stamp))
                else:
                    value = msg.data[0] if isinstance(msg.data, bytes) else msg.data
                series[name].append((t, value))
    for entries in series.values():
        entries.sort(key=lambda x: x[0])
    return series, dict(counts)


def within(entries, start, end):
    return [(t, v) for t, v in entries if start<=t<=end]


def category(v):
    if abs(v['v'])<.02:
        return 'turn_in_place' if abs(v['w'])>.10 else 'stopped'
    return 'curved' if abs(v['w'])>.10 else 'straight'


def velocity_summary(entries, start, end):
    cut = within(entries, start-.5, end)
    durations = defaultdict(float); runs = []; current = None
    for (a, va), (b, _) in zip(cut, cut[1:]):
        lo, hi = max(start, a), min(end, b, a+.5)
        if hi<=lo:
            continue
        cat = category(va); durations[cat] += hi-lo
        if cat=='turn_in_place':
            if current and lo-current['end']<.6:
                current['end'] = hi; current['angle'] += abs(va['w'])*(hi-lo)
            else:
                if current: runs.append(current)
                current = dict(start=lo, end=hi, angle=abs(va['w'])*(hi-lo))
        elif current:
            runs.append(current); current = None
    if current: runs.append(current)
    return dict(samples=len(cut), seconds={k: round(v, 2) for k, v in durations.items()},
                v_quantiles=quantiles(v['v'] for _, v in cut),
                abs_w_quantiles=quantiles(abs(v['w']) for _, v in cut),
                turn_runs=[dict(start=wall(r['start']), end=wall(r['end']),
                                seconds=round(r['end']-r['start'], 2), angle_rad=round(r['angle'], 3))
                           for r in runs if r['end']-r['start']>=2])


def movement(entries, start, end):
    p = within(entries, start, end)
    if len(p)<2: return None
    path = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for (_, a), (_, b) in zip(p, p[1:]))
    turn = sum(abs(wrap(b['yaw']-a['yaw'])) for (_, a), (_, b) in zip(p, p[1:]))
    return dict(path_m=round(path, 4), absolute_turn_rad=round(turn, 4),
                net_m=round(math.hypot(p[-1][1]['x']-p[0][1]['x'], p[-1][1]['y']-p[0][1]['y']), 4),
                first=p[0][1], last=p[-1][1])


def tracking(command, odom, start, end):
    stamps = [t for t, _ in command]
    errors_v=[]; errors_w=[]; signs=[]
    for t, actual in within(odom, start, end):
        i=bisect_right(stamps, t-.15)-1
        if i<0 or t-stamps[i]>.6: continue
        requested=command[i][1]
        errors_v.append(abs(requested['v']-actual['v']))
        errors_w.append(abs(requested['w']-actual['w']))
        if abs(requested['w'])>.15 and abs(actual['w'])>.1:
            signs.append(requested['w']*actual['w']>0)
    return dict(approx_lag_s=.15, abs_v_error_quantiles=quantiles(errors_v),
                abs_w_error_quantiles=quantiles(errors_w),
                turning_same_sign_fraction=round(sum(signs)/len(signs), 4) if signs else None)


def latest(entries, t):
    i=bisect_right([a for a,_ in entries],t)-1
    return entries[i] if i>=0 else (0,{})


def robot_point(transform, odom):
    c=math.cos(transform['yaw']); s=math.sin(transform['yaw'])
    return (transform['x']+c*odom['x']-s*odom['y'],
            transform['y']+s*odom['x']+c*odom['y'])


def snapshot(series,t):
    pt,pv=latest(series['/plan'],t); _,ov=latest(series['/odom'],t)
    _,tv=latest(series['map->odom'],t)
    heading=wrap(ov['yaw']+tv['yaw']) if ov and tv else None
    return dict(time=wall(t),epoch=t,plan=pv,plan_age_s=t-pt,
        approximate_map_pose=robot_point(tv,ov) if ov and tv else None,
        approximate_heading_error=wrap(pv['heading']-heading) if 'heading' in pv and heading is not None else None,
        velocities={k:latest(series[k],t)[1] for k in VELOCITIES},
        ranges={k:dict(age_s=round(t-latest(v,t)[0],3),**latest(v,t)[1])
                for k,v in series.items() if k.startswith('/range/')})


def report(series, counts):
    goals={}
    for t, states in series['/navigate_to_pose/_action/status']:
        for s in states:
            g=goals.setdefault(s['uuid'], dict(uuid=s['uuid']))
            if s['status'] in (1, 2) and 'start' not in g: g['start']=t
            if s['status'] in (4, 5, 6) and 'start' in g and 'end' not in g:
                g.update(end=t, terminal=s['status'])
    results=[]
    for g in sorted(goals.values(), key=lambda x:x.get('start', float('inf'))):
        if 'start' not in g: continue
        a=g['start']; b=g.get('end', series['/odom'][-1][0])
        r=dict(uuid=g['uuid'], start=wall(a), end=wall(b), seconds=round(b-a, 2), terminal=g.get('terminal'))
        r['velocities']={k:velocity_summary(series[k], a, b) for k in VELOCITIES}
        r['odom_movement']=movement(series['/odom'], a, b)
        r['tracking']=tracking(series['/cmd_vel_selected'], series['/odom'], a, b)
        r['plans']=dict(count=len(within(series['/plan'], a, b)),
                        length_quantiles=quantiles(v['length'] for _,v in within(series['/plan'],a,b)),
                        longest=[dict(time=wall(t),length_m=v['length']) for t,v in
                                 sorted(within(series['/plan'],a,b),key=lambda p:p[1]['length'],reverse=True)[:5]])
        r['range_quantiles']={k:quantiles(v['range'] for _,v in within(entries,a,b))
                               for k,entries in series.items() if k.startswith('/range/')}
        changes=[]; body_shifts=[]; yaw_shifts=[]
        tf=within(series['map->odom'],a,b)
        for (ta,va),(tb,vb) in zip(tf,tf[1:]):
            d=math.hypot(vb['x']-va['x'],vb['y']-va['y']); angle=abs(wrap(vb['yaw']-va['yaw']))
            _,ob=latest(series['/odom'],tb)
            body_shift=math.dist(robot_point(va,ob),robot_point(vb,ob)) if ob else 0.
            body_shifts.append(body_shift);yaw_shifts.append(angle)
            if d>.02 or angle>.02:
                changes.append(dict(time=wall(tb), transform_origin_translation=round(d,4),
                                    robot_position_shift=round(body_shift,4),yaw=round(angle,4)))
        r['map_odom_corrections_over_2cm_or_002rad']=changes
        r['map_odom_correction_at_robot_quantiles']=quantiles(body_shifts)
        r['map_odom_yaw_correction_quantiles']=quantiles(yaw_shifts)
        r['amcl_covariance']=[dict(time=wall(t), covariance=v['covariance'])
                              for t,v in within(series['/amcl_pose'],a,b)][::10]
        r['warnings']=[dict(time=wall(t), **v) for t,v in within(series['/rosout'],a,b)]
        r['turn_snapshots']=[snapshot(series,datetime.fromisoformat(run['start']).timestamp()+run['seconds']/2)
                             for run in r['velocities']['/cmd_vel_nav']['turn_runs'][:6]]
        results.append(r)
    return dict(topic_counts=counts, goals=results,
                home_transitions=transitions(series['/mapping/home_status'],'phase'),
                navigation_transitions=transitions(series['/mobile/navigation_context'],'phase'))


def transitions(entries,key):
    out=[]; previous=None
    for t,v in entries:
        if v.get(key)!=previous:
            previous=v.get(key)
            out.append(dict(time=wall(t), phase=previous, message=v.get('message'), pose=v.get('pose'),
                            progress=v.get('dock_progress_m')))
    return out


def json_safe(value):
    if isinstance(value,float) and not math.isfinite(value):
        return str(value)
    if isinstance(value,dict):
        return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):
        return [json_safe(v) for v in value]
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path); p.add_argument('--output',type=Path)
    args=p.parse_args()
    series,counts=read(args.bag)
    result=report(series,counts)
    text=json.dumps(json_safe(result),ensure_ascii=False,indent=2,allow_nan=False)
    if args.output:
        args.output.write_text(text,encoding='utf-8')
        print(json.dumps(dict(output=str(args.output), goals=len(result['goals']), topic_count=len(counts))))
    else:
        print(text)


if __name__=='__main__': main()

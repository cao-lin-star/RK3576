#!/usr/bin/env python3
"""Read-only sonar/costmap audit of a CLOSED ROS 2 SQLite bag.

No ROS nodes, publishers, services, hardware, or network connections are started.
Use after stopping the recorder, never against a live recording directory.

Example (after sourcing Humble):
  python3 scripts/analyze_closed_bag_glass.py --closed-bag /path/to/bag \
    --start-seconds 76 --end-seconds 382 \
    --counterfactual-seconds 83.319 189.707 198.847 266.339 273.238 \
    --output /path/to/glass_audit_reproduced.json
"""

import argparse
import bisect
import collections
import json
import math
from pathlib import Path
import sqlite3
import sys

import numpy as np
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from scipy.ndimage import distance_transform_edt
from tf2_ros import Buffer
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/rk3576_footbath_safety'))
from rk3576_footbath_safety.glass_geometry import (  # noqa: E402
    EchoArc, EchoArcConfirmation, cone_minimum, glass_candidate,
)

RAW = ['/range/ultrasonic_front_observe', '/range/ultrasonic_left',
       '/range/ultrasonic_right']
GLASS = ['/range/suspected_glass', '/range/suspected_glass_left',
         '/range/suspected_glass_right']
SCANS = ['/scan_high', '/scan_low_front']


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(transform):
    q = transform.transform.rotation
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def load_bag(folder):
    meta = yaml.safe_load((folder / 'metadata.yaml').read_text())[
        'rosbag2_bagfile_information']
    start = meta['starting_time']['nanoseconds_since_epoch'] / 1e9
    data = collections.defaultdict(list)
    wanted = set(RAW + GLASS + SCANS + [
        '/map', '/global_costmap/costmap_raw', '/tf', '/tf_static', '/plan'])
    for relative in meta['relative_file_paths']:
        filename = (folder / relative).resolve()
        if filename.parent != folder.resolve() or filename.suffix != '.db3':
            raise ValueError('Metadata references an unexpected SQLite path')
        # immutable is valid only because --closed-bag explicitly asserts the
        # recorder is stopped. Read-only access cannot mutate the evidence.
        with sqlite3.connect(filename.as_uri() + '?mode=ro&immutable=1', uri=True) as db:
            for topic_id, name, typename in db.execute('select id,name,type from topics'):
                if name not in wanted:
                    continue
                cls = get_message(typename)
                for timestamp, blob in db.execute(
                        'select timestamp,data from messages where topic_id=? '
                        'order by timestamp', (topic_id,)):
                    data[name].append((timestamp * 1e-9, deserialize_message(blob, cls)))
    for values in data.values():
        values.sort(key=lambda item: item[0])
    duration = meta['duration']['nanoseconds'] / 1e9
    tf = Buffer(cache_time=Duration(seconds=max(1000., duration + 10.)))
    for _, message in data['/tf_static']:
        for transform in message.transforms:
            tf.set_transform_static(transform, 'closed_bag')
    for _, message in data['/tf']:
        for transform in message.transforms:
            tf.set_transform(transform, 'closed_bag')
    return start, data, tf


def compare_echo(data, scan_times, tf, message, evaluated_at, compensate):
    minima, differences = [], []
    for topic in SCANS:
        index = bisect.bisect_right(scan_times[topic], evaluated_at) - 1
        if index < 0:
            raise ValueError('No preceding scan')
        scan = data[topic][index][1]
        if not (0 <= evaluated_at - stamp(scan) <= .5 and
                0 <= evaluated_at - stamp(message) <= .5):
            raise ValueError('Stale acquisition')
        if compensate:
            transform = tf.lookup_transform_full(
                message.header.frame_id, Time.from_msg(message.header.stamp),
                scan.header.frame_id, Time.from_msg(scan.header.stamp), 'odom')
        else:
            transform = tf.lookup_transform(message.header.frame_id,
                                            scan.header.frame_id, Time())
        p = transform.transform.translation
        minima.append(cone_minimum(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max, p.x, p.y, yaw(transform),
            min(.261799388, message.field_of_view / 2)))
        differences.append(stamp(message) - stamp(scan))
    return glass_candidate(message.range, *minima, 1.8, .15), minima, differences


def echo_replay(data, tf, origin_time, begin, end, delay):
    scan_times = {topic: [item[0] for item in data[topic]] for topic in SCANS}
    actual, replay = {}, {}
    for topic in GLASS:
        total, failed = 0, 0
        differences, rejected = [], []
        for recorded_at, message in data[topic]:
            if not begin <= recorded_at - origin_time <= end:
                continue
            try:
                old, old_min, dt = compare_echo(
                    data, scan_times, tf, message, recorded_at, False)
                new, new_min, _ = compare_echo(
                    data, scan_times, tf, message, recorded_at, True)
            except Exception:
                failed += 1
                continue
            total += 1
            differences.extend(dt)
            if old and not new:
                rejected.append({
                    'seconds': round(recorded_at-origin_time, 3),
                    'range_m': round(message.range, 3),
                    'old_lidar_minima_m': np.round(old_min, 3).tolist(),
                    'aligned_lidar_minima_m': np.round(new_min, 3).tolist(),
                    'sonar_minus_scan_seconds': np.round(dt, 3).tolist()})
        actual[topic] = {
            'evaluated_published_echoes': total, 'unavailable_comparisons': failed,
            'cross_time_rejections': len(rejected), 'rejected_echoes': rejected,
            'time_difference_quantiles_0_50_95_100_seconds':
                np.quantile(differences, [0, .5, .95, 1]).tolist() if differences else []}

    for topic in RAW:
        counts = collections.Counter()
        confirmation = EchoArcConfirmation()
        old_assert = old_clear = 0
        old_flag = False
        output_times = []
        for recorded_at, message in data[topic]:
            if not begin <= recorded_at-origin_time <= end:
                continue
            evaluated_at = recorded_at + delay
            try:
                before, _, _ = compare_echo(
                    data, scan_times, tf, message, evaluated_at, False)
                candidate, _, _ = compare_echo(
                    data, scan_times, tf, message, evaluated_at, True)
                transform = tf.lookup_transform(
                    'odom', message.header.frame_id, Time.from_msg(message.header.stamp))
                p = transform.transform.translation
                arc = EchoArc(stamp(message), p.x, p.y, yaw(transform),
                              message.range, message.field_of_view / 2)
            except Exception:
                counts['invalid'] += 1
                confirmation.reset()
                old_assert = 0
                old_flag = False
                continue
            if before:
                old_assert += 1
                old_clear = 0
                if old_assert >= 3:
                    old_flag = True
                if old_flag:
                    counts['old_model_marks'] += 1
            else:
                old_assert = 0
                old_clear += 1
                if old_clear >= 3:
                    old_flag = False
            if candidate:
                counts['time_aligned_candidates'] += 1
                if confirmation.observe(arc):
                    counts['new_marks'] += 1
                    output_times.append(round(recorded_at-origin_time, 3))
                else:
                    counts['unconfirmed_spatial'] += 1
            else:
                confirmation.reset()
            counts['raw'] += 1
        replay[topic] = dict(counts, new_mark_seconds=output_times)
    return {'actual_published_echo_alignment': actual, 'approximate_echo_replay': replay}


def bfs_distance(blocked, start, goal, resolution):
    """Four-connected binary reachability, not Navfn's weighted path cost."""
    height, width = blocked.shape
    visited = np.zeros(blocked.shape, bool)
    queue = collections.deque([(int(start[0]), int(start[1]), 0)])
    visited[start[1], start[0]] = True
    while queue:
        x, y, steps = queue.popleft()
        if x == goal[0] and y == goal[1]:
            return round(steps * resolution, 3)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            xx, yy = x + dx, y + dy
            if (0 <= xx < width and 0 <= yy < height and
                    not visited[yy, xx] and not blocked[yy, xx]):
                visited[yy, xx] = True
                queue.append((xx, yy, steps + 1))
    return None


def path_counterfactuals(data, tf, origin_time, requested_times):
    message = data['/map'][0][1]
    static = np.array(message.data).reshape(message.info.height, message.info.width)
    resolution = message.info.resolution
    origin = np.array([message.info.origin.position.x, message.info.origin.position.y])
    arcs = []
    for topic in GLASS:
        for _, message in data[topic]:
            try:
                transform = tf.lookup_transform(
                    'map', message.header.frame_id, Time.from_msg(message.header.stamp))
            except Exception:
                continue
            p = transform.transform.translation
            angles = yaw(transform) + np.linspace(
                -message.field_of_view / 2, message.field_of_view / 2, 9)
            points = np.array([p.x, p.y]) + message.range * np.c_[
                np.cos(angles), np.sin(angles)]
            arcs.append((stamp(message), points))
    cost_times = [t for t, _ in data['/global_costmap/costmap_raw']]
    plan_times = [t for t, _ in data['/plan']]
    results = []
    for requested in requested_times:
        index = min(range(len(plan_times)),
                    key=lambda i: abs(plan_times[i]-origin_time-requested))
        plan_at, plan = data['/plan'][index]
        index = bisect.bisect_right(cost_times, plan_at) - 1
        cost_at, cost = data['/global_costmap/costmap_raw'][index]
        grid = np.array(cost.data).reshape(cost.metadata.size_y, cost.metadata.size_x)
        if (grid.shape != static.shape or
                abs(cost.metadata.resolution-resolution) > 1e-7 or
                not np.allclose([cost.metadata.origin.position.x,
                                 cost.metadata.origin.position.y], origin)):
            raise ValueError('Static and global grids require explicit resampling')
        matching = np.zeros(grid.shape, bool)
        for acquired_at, points in arcs:
            if not 0 <= cost_at-acquired_at <= 5.:
                continue
            cells = np.floor((points-origin)/resolution).astype(int)
            for x, y in cells:
                if (0 <= x < grid.shape[1] and 0 <= y < grid.shape[0] and
                        grid[y, x] == 254 and 0 <= static[y, x] < 65):
                    matching[y, x] = True
        remaining_lethal = (grid == 254) & ~matching
        poses = np.array([(p.pose.position.x, p.pose.position.y) for p in plan.poses])
        start = np.floor((poses[0]-origin)/resolution).astype(int)
        goal = np.floor((poses[-1]-origin)/resolution).astype(int)
        results.append({
            'plan_seconds': round(plan_at-origin_time, 3),
            'costmap_seconds': round(cost_at-origin_time, 3),
            'recorded_plan_length_m': round(np.linalg.norm(
                np.diff(poses, axis=0), axis=1).sum(), 3),
            'exact_glass_cells': int(matching.sum()),
            'original_binary_bfs_m': bfs_distance(grid >= 253, start, goal, resolution),
            'without_matching_glass_binary_bfs_m': bfs_distance(
                (distance_transform_edt(~remaining_lethal)*resolution <= .24) |
                (grid == 255),
                start, goal, resolution)})
    return results


def json_finite(value):
    if isinstance(value, dict):
        return {key: json_finite(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_finite(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0 else '-Infinity')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--closed-bag', type=Path, required=True,
                        help='Explicit assertion that the recorder is stopped')
    parser.add_argument('--start-seconds', type=float, default=76.)
    parser.add_argument('--end-seconds', type=float, default=382.)
    parser.add_argument('--echo-evaluation-delay', type=float, default=.05)
    parser.add_argument('--counterfactual-seconds', type=float, nargs='*',
                        default=[83.319, 189.707, 198.847, 266.339, 273.238])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    folder = args.closed_bag.resolve()
    if list(folder.glob('*.db3-wal')) or list(folder.glob('*.db3-journal')):
        raise SystemExit('Refusing bag with SQLite write-ahead/journal files')
    start, data, tf = load_bag(folder)
    result = {
        'bag': str(folder), 'starting_time_epoch_seconds': start,
        'window_seconds': [args.start_seconds, args.end_seconds],
        'method': {
            'no_robot_or_ros_publishers': True,
            'echo_replay_delay_seconds': args.echo_evaluation_delay,
            'echo_replay_is_approximate_not_exact_executor_schedule': True,
            'counterfactual_is_offline_only_not_a_recommendation_to_delete_obstacles': True,
            'binary_bfs_is_not_navfn_weighted_path_reproduction': True,
            'matching_cells_may_also_contain_lidar_evidence': True}}
    result.update(echo_replay(data, tf, start, args.start_seconds,
                              args.end_seconds, args.echo_evaluation_delay))
    result['path_counterfactuals'] = path_counterfactuals(
        data, tf, start, args.counterfactual_seconds)
    text = json.dumps(json_finite(result), indent=2, ensure_ascii=False, allow_nan=False)
    if args.output:
        args.output.write_text(text + '\n', encoding='utf-8')
    else:
        print(text)


if __name__ == '__main__':
    main()

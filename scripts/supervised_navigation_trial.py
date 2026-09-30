#!/usr/bin/env python3
"""One explicitly authorized HTTP-owned goal followed by one return.

Run on the robot under continuous local supervision. PIN is read once from
stdin; neither it nor the HTTP session token is printed or persisted. This
script never publishes velocity or changes parameters. An independent passive
rosbag recorder is recommended; its lifetime does not control robot motion.
"""
import argparse
from collections import deque
import json
import math
import signal
import sys
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


BUSY = {'sending', 'active', 'suspended', 'resuming'}
DOCK_PHASES = {'undocking', 'aligning', 'docking', 'dock_waiting', 'dock_preparing'}
VELOCITIES = ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_auto_limited',
              '/cmd_vel_selected', '/cmd_vel_recovery')


def check_state(state, initial=False):
    if state.get('mode') != 'navigation' or not state.get('initialized'):
        raise RuntimeError('navigation mode and confirmed localization required')
    if state.get('transitioning') or not state.get('home_status_fresh'):
        raise RuntimeError('mode transition or stale home supervisor')
    if not all(state.get('healthy', {}).get(k) for k in ('scan', 'map', 'odom')):
        raise RuntimeError('critical scan/map/odom health is not ready')
    pose = state.get('pose')
    if not pose or not all(math.isfinite(float(pose[k])) for k in ('x', 'y', 'yaw')):
        raise RuntimeError('fresh map pose unavailable')
    if state.get('manual_active'):
        raise RuntimeError('manual control is active')
    if initial and not state.get('home', {}).get('available'):
        raise RuntimeError('recorded return origin is unavailable')
    if initial and (state.get('nav_state') in BUSY | {'departing', 'hazard_recovery'}
                    or state.get('home', {}).get('phase') in DOCK_PHASES |
                    {'returning', 'preparing', 'sending', 'waiting_health', 'hazard_suspended'}):
        raise RuntimeError('another navigation/return task is already active')


def progress_reason(samples):
    """Samples are (monotonic, x, y, yaw), scoped to one action UUID."""
    if len(samples) < 2 or samples[-1][0] - samples[0][0] < 30.0:
        return None
    net = math.hypot(samples[-1][1]-samples[0][1], samples[-1][2]-samples[0][2])
    path = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(samples, list(samples)[1:]))
    turn = sum(abs(math.atan2(math.sin(b[3]-a[3]), math.cos(b[3]-a[3])))
               for a, b in zip(samples, list(samples)[1:]))
    if net < .20 and turn > 1.5*math.pi:
        return 'same action: 30 seconds turning with <0.20 m net progress'
    if path < .05:
        return 'same action: 30 seconds with <0.05 m cumulative translation'
    return None


def quiet(sample, now):
    """Require fresh measured odometry, selected command and revoked lease."""
    for key in ('odom', 'selected', 'lease'):
        if key not in sample or now-sample[key][0] > 1.0:
            return False
    return (sample['lease'][1] is False
            and max(abs(v) for v in sample['selected'][1]) < 1.e-6
            and abs(sample['odom'][1][0]) < .01
            and abs(sample['odom'][1][1]) < .02)


class LocalAPI:
    def __init__(self):
        self.token = None

    def request(self, path, payload=None):
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['Authorization'] = 'Bearer '+self.token
        body = None if payload is None else json.dumps(payload).encode()
        request = Request('http://127.0.0.1:8080'+path, data=body, headers=headers)
        try:
            with urlopen(request, timeout=5) as response:
                return json.load(response)
        except HTTPError as error:
            # Do not echo response bodies or request headers containing secrets.
            raise RuntimeError(f'HTTP {error.code} for {path}') from None

    def login(self, pin):
        response = self.request('/api/login', {'pin': pin})
        self.token = response.get('token')
        if not self.token:
            raise RuntimeError('login did not yield a session')


def emit(event, **fields):
    print(json.dumps(dict(event=event, monotonic=round(time.monotonic(), 3), **fields),
                     ensure_ascii=False), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--leg-timeout', type=float, default=180)
    parser.add_argument('--return-timeout', type=float, default=240)
    args = parser.parse_args()
    if (not all(math.isfinite(x) for x in args.goal)
            or not 30 <= args.leg_timeout <= 600 or not 30 <= args.return_timeout <= 600):
        parser.error('finite goal and timeouts in [30, 600] seconds required')

    # Lazy ROS imports permit pure safety-logic tests without a ROS installation.
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from action_msgs.msg import GoalStatusArray
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String, UInt8
    from std_srvs.srv import Trigger

    rclpy.init(args=[])
    node = rclpy.create_node('supervised_navigation_trial')
    api = LocalAPI()
    data = {'velocities': {}, 'diagnostics': {}, 'active_uuids': []}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def velocity(topic, msg):
        now = time.monotonic()
        pair = (msg.linear.x, msg.angular.z)
        data['velocities'][topic] = (now, pair)
        if topic == '/cmd_vel_selected':
            data['selected'] = (now, pair)

    for topic in VELOCITIES:
        node.create_subscription(Twist, topic, lambda m, t=topic: velocity(t, m), 10)
    node.create_subscription(Odometry, '/odom', lambda m: data.update(
        odom=(time.monotonic(), (m.twist.twist.linear.x, m.twist.twist.angular.z))), 20)
    node.create_subscription(Bool, '/safety/auto_motion_lease', lambda m: data.update(
        lease=(time.monotonic(), m.data)), 10)
    node.create_subscription(UInt8, '/chassis/control_source', lambda m: data.update(
        source=(time.monotonic(), int(m.data))), 10)

    def context(msg):
        try:
            data['context'] = (time.monotonic(), json.loads(msg.data))
        except (ValueError, TypeError):
            data['context'] = (time.monotonic(), {})

    def home(msg):
        try:
            data['home'] = (time.monotonic(), json.loads(msg.data))
        except (ValueError, TypeError):
            pass

    def diagnostics(msg):
        for entry in msg.status:
            if 'supervisor' in entry.name or 'base' in entry.name:
                values = {v.key: v.value for v in entry.values}
                data['diagnostics'][entry.name] = dict(message=entry.message, values=values)
                if 'supervisor' in entry.name:
                    data['supervisor'] = (time.monotonic(), values)

    def action_status(msg):
        data['status_at'] = time.monotonic()
        data['active_uuids'] = [bytes(s.goal_info.goal_id.uuid).hex()
                                for s in msg.status_list if s.status in (1, 2, 3)]

    node.create_subscription(String, '/mobile/navigation_context', context, 10)
    node.create_subscription(String, '/mapping/home_status', home, latched)
    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics, 10)
    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', action_status, latched)
    stop_client = node.create_client(Trigger, '/exploration/stop')

    def spin_for(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)

    def ros_health():
        now = time.monotonic()
        if now-data.get('odom', (0,))[0] > 1.0:
            raise RuntimeError('odometry stream stale')
        stamp, values = data.get('supervisor', (0, {}))
        if now-stamp > 3.0 or str(values.get('inputs_healthy')).lower() != 'true':
            raise RuntimeError('supervisor critical inputs stale/unhealthy')
        if str(values.get('fault_latched')).lower() == 'true':
            raise RuntimeError('supervisor reports a latched safety fault')
        if values.get('state') == 'paused_fault' and values.get('hazard_phase', 'idle') == 'idle':
            raise RuntimeError('supervisor stopped on a fault: '+str(values.get('reason', '')))
        if now-data.get('context', (0,))[0] > 2.0:
            raise RuntimeError('gateway navigation ownership heartbeat stale')
        if now-data.get('source', (0,))[0] > 2.0:
            raise RuntimeError('chassis control source heartbeat stale')
        if data['source'][1] >= 2:
            raise RuntimeError('PS2/debug source took control; trial will not resume')
        if len(data['active_uuids']) > 1:
            raise RuntimeError('multiple active navigation goals')

    def report(state, leg):
        now = time.monotonic()
        owner = data.get('context', (0, {}))[1]
        emit('sample', leg=leg, navigation=state.get('nav_state'),
             feedback=state.get('nav_feedback'), home=state.get('home'), pose=state.get('pose'),
             action_uuids=data['active_uuids'],
             context={k: owner.get(k) for k in ('phase', 'goal', 'message')},
             velocities={k: dict(age=round(now-v[0], 3), linear=v[1][0], angular=v[1][1])
                         for k, v in data['velocities'].items()}, diagnostics=data['diagnostics'])

    def run_leg(leg, timeout, old_token=''):
        deadline = time.monotonic()+timeout
        samples = deque()
        window_key = None
        next_report = 0
        owned_seen = leg == 'return'
        while time.monotonic() < deadline:
            spin_for(.20)
            state = api.request('/api/state')
            check_state(state)
            ros_health()
            now = time.monotonic()
            phase = state.get('home', {}).get('phase')
            owner = data.get('context', (0, {}))[1]
            if owner.get('token') and owner.get('token') != old_token:
                owned_seen = True
            if now >= next_report:
                report(state, leg)
                next_report = now+5
            if leg == 'goal':
                if owned_seen and state.get('nav_state') == 'succeeded':
                    return state
                if state.get('nav_state') in {'rejected', 'aborted', 'error', 'canceled', 'departure_failed', 'return_failed'}:
                    raise RuntimeError('goal ended unsuccessfully: '+str(state.get('nav_message')))
            elif phase == 'arrived':
                return state
            if phase in {'failed', 'canceled', 'unavailable'}:
                raise RuntimeError('home supervisor failed: '+str(state.get('home', {}).get('message')))
            # Dock alignment/reverse and bounded hazard recovery have their own
            # local supervisor limits. Do not classify their intentional turns.
            recovery = data.get('supervisor', (0, {}))[1].get('hazard_phase', 'idle')
            uuid = data['active_uuids'][0] if data['active_uuids'] else None
            key = uuid if phase not in DOCK_PHASES and recovery == 'idle' else None
            if key != window_key or key is None:
                samples.clear()
                window_key = key
            if key:
                pose = state['pose']
                samples.append((now, pose['x'], pose['y'], pose['yaw']))
                while len(samples) > 2 and now-samples[1][0] >= 30:
                    samples.popleft()
                reason = progress_reason(samples)
                if reason:
                    raise RuntimeError(reason)
        raise RuntimeError(leg+' leg exceeded its bounded timeout')

    def stop_and_verify():
        # Both paths revoke the same local supervisor lease. No velocity is
        # published, and an HTTP failure cannot suppress the ROS fallback.
        if api.token:
            try:
                api.request('/api/cancel', {})
            except Exception as error:
                emit('cancel_http_failed', error=type(error).__name__)
        if stop_client.wait_for_service(timeout_sec=2):
            try:
                future = stop_client.call_async(Trigger.Request())
                until = time.monotonic()+5
                while not future.done() and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.05)
                emit('stop_service', accepted=bool(future.done() and future.result() and future.result().success))
            except Exception as error:
                emit('stop_service', accepted=False, error=type(error).__name__)
        else:
            emit('stop_service', accepted=False)
        stable = None
        until = time.monotonic()+6
        while time.monotonic() < until:
            spin_for(.1)
            now = time.monotonic()
            if quiet(data, now):
                stable = now if stable is None else stable
                if now-stable >= 1.0:
                    emit('stopped_verified', lease=False, selected_zero=True, odom_stationary=True)
                    return True
            else:
                stable = None
        emit('STOP_NOT_VERIFIED', instruction='现场人员立即急停；不能确认小车已停车')
        return False

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, interrupted)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, interrupted)
    success = False
    stopped = False
    try:
        pin = sys.stdin.readline().rstrip('\r\n')
        if not pin:
            raise RuntimeError('PIN must be supplied once on stdin')
        api.login(pin)
        del pin
        spin_for(3)
        initial = api.request('/api/state')
        check_state(initial, initial=True)
        ros_health()
        if data['active_uuids'] or data.get('context', (0, {}))[1].get('phase') in BUSY:
            raise RuntimeError('preflight refused: an action/owner is already active')
        if abs(data['odom'][1][0]) >= .01 or abs(data['odom'][1][1]) >= .02:
            raise RuntimeError('preflight refused: robot is not stationary')
        report(initial, 'preflight')
        old_token = data.get('context', (0, {}))[1].get('token', '')
        target = dict(zip(('x', 'y', 'yaw'), args.goal))
        api.request('/api/goal', target)
        emit('goal_requested', target=target)
        run_leg('goal', args.leg_timeout, old_token)
        # Confirm a continuous two-second standstill before the one return call;
        # do not rely on just the last odometry message in a sleep interval.
        stable_since = None
        stop_deadline = time.monotonic()+8
        while time.monotonic() < stop_deadline:
            spin_for(.1)
            now = time.monotonic()
            measured = data.get('odom', (0, (1, 1)))
            selected = data.get('selected', (0, (1, 1)))
            if (now-measured[0] <= 1 and now-selected[0] <= 1
                    and abs(measured[1][0]) < .01 and abs(measured[1][1]) < .02
                    and max(abs(v) for v in selected[1]) < 1.e-6):
                stable_since = now if stable_since is None else stable_since
                if now-stable_since >= 2:
                    break
            else:
                stable_since = None
        else:
            raise RuntimeError('goal succeeded but two-second standstill was not confirmed')
        check_state(api.request('/api/state'))
        ros_health()
        if abs(data['odom'][1][0]) >= .01 or abs(data['odom'][1][1]) >= .02:
            raise RuntimeError('goal succeeded but wheels did not become stationary')
        api.request('/api/return_home', {})
        emit('return_requested')
        run_leg('return', args.return_timeout)
        success = True
        emit('trial_completed')
    except (Exception, KeyboardInterrupt) as error:
        emit('trial_failed', reason=str(error) or type(error).__name__)
    finally:
        try:
            stopped = stop_and_verify()
        except (Exception, KeyboardInterrupt) as error:
            emit('STOP_NOT_VERIFIED', error=type(error).__name__, instruction='现场人员立即急停')
        api.token = None
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success and stopped else 1


if __name__ == '__main__':
    raise SystemExit(main())

"""Bounded reverse recovery for exploration; never rotates over a cliff."""
import json
import math
import time
from pathlib import Path

from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, UInt8
from .dock_motion import pose_tuple, segment_error


def hazard_kind(left, right, sonar):
    if not all(math.isfinite(v) and v > 0 for v in (left, right, sonar)):
        raise ValueError('测距无效，禁止自动恢复，请人工处理')
    if left >= .215 or right >= .220:
        return 'cliff'
    return 'sonar' if sonar <= .20 else None


def ground_clear(left, right, sonar):
    return left <= .195 and right <= .200 and sonar >= .28


def reverse_progress(start, current):
    progress, cross, heading = segment_error(start, current, -1)
    if not all(math.isfinite(v) for v in (progress, cross, heading)):
        raise ValueError('里程计无效')
    if progress < -.015 or abs(cross) > .035 or abs(heading) > .10:
        raise ValueError('后退方向或轨迹异常，请人工处理')
    return progress


class HazardRecovery:
    def __init__(self, node):
        self.n = node
        self.assume_rear_safe = bool(node.declare_parameter(
            'hazard_recovery.assume_rear_ground_safe', True).value)
        self.ranges = {}
        self.source, self.source_at = None, 0.
        self.fault, self.fault_at = 0, 0.
        self.active = False
        self.phase = 'idle'
        self.zones = []
        self.pending = []
        self.last_zones = 0.
        self.manual_at = 0.
        self.last_success = -math.inf
        map_file = node.declare_parameter('hazard_recovery.map_file', '').value
        if map_file:
            sidecar = Path(map_file).with_suffix('.hazards.json')
            if sidecar.exists():
                data = json.loads(sidecar.read_text(encoding='utf-8'))
                if data.get('frame_id') != 'map' or len(data['zones']) > 128:
                    raise ValueError('Invalid hazard map sidecar')
                for z in data['zones']:
                    if (z['kind'] != 'cliff' or not all(math.isfinite(z[k]) for k in ('x','y','radius'))
                            or not 0 < z['radius'] <= .3):
                        raise ValueError('Invalid hazard zone')
                    z['expires'] = 0.
                    self.zones.append(z)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.zpub = node.create_publisher(PoseArray, '/safety/hazard_zones', qos)
        self.modepub = node.create_publisher(Bool, '/safety/recovery_active', 10)
        self.cmdpub = node.create_publisher(Twist, '/cmd_vel_recovery', 10)
        for key, topic in [('left', '/range/tof_left'), ('right', '/range/tof_right'),
                           ('sonar', '/range/ultrasonic')]:
            node.create_subscription(Range, topic, lambda m, k=key: self._range(k, m),
                                     qos_profile_sensor_data)
        node.create_subscription(UInt8, '/chassis/control_source', self._source, 10)
        node.create_subscription(Twist, '/cmd_vel_manual', self._manual, 10)
        node.create_subscription(DiagnosticArray, '/diagnostics', self._diagnostic, 10)
        self.mode(False)

    def _range(self, key, msg):
        age = (self.n.get_clock().now().nanoseconds-Time.from_msg(msg.header.stamp).nanoseconds)/1e9
        value = msg.range
        if not (0 <= age <= .5 and math.isfinite(value)
                and msg.min_range <= value <= msg.max_range):
            self.ranges.pop(key, None)
        else:
            self.ranges[key] = (value, time.monotonic())

    def _source(self, msg):
        self.source, self.source_at = msg.data, time.monotonic()
        if msg.data >= 2 and self.active:
            self.fail('手柄或调试串口接管，恢复已取消，请人工处理')

    def _manual(self, msg):
        self.manual_at = time.monotonic()
        if self.active:
            self.fail('手动遥控接管，自动恢复已取消')

    def _diagnostic(self, msg):
        for status in msg.status:
            if status.name == 'rk3576_footbath/stm32_serial_bridge':
                values = {v.key: v.value for v in status.values}
                try:
                    self.fault = int(values['fault_flags'])
                    self.fault_at = time.monotonic()
                except (KeyError, ValueError):
                    self.fault_at = 0.

    def values(self, now):
        if any(k not in self.ranges or now-self.ranges[k][1] > .5
               for k in ('left', 'right', 'sonar')):
            raise ValueError('ToF/超声波失联或数据无效，请人工处理；不自动后退')
        return tuple(self.ranges[k][0] for k in ('left', 'right', 'sonar'))

    def gate(self, healthy, now):
        d = self.n.home.dock
        if not healthy or self.n.home.current_pose() is None or d.odom is None or now-d.odom_at > .5:
            raise ValueError('传感器、定位或里程计失效，恢复已停止')
        if now-self.source_at > .8 or self.source is None:
            raise ValueError('控制来源遥测失联，请烧录配套F407固件')
        if self.source >= 2 or now-self.manual_at < .5:
            raise ValueError('人工控制占用，自动恢复已停止')
        if now-self.fault_at > 2.5 or (self.fault & ~1024):
            raise ValueError('底盘通信或非障碍故障，恢复已停止')

    def mode(self, active):
        msg = Bool(); msg.data = active
        self.modepub.publish(msg)

    def zero(self):
        self.cmdpub.publish(Twist())
        self.n._publish_lease(False)

    def fail(self, reason):
        self.active = False
        self.phase = 'failed'
        self.zero()
        self.mode(False)
        self.n._fault_latched = True
        self.n._pause(self.n.PAUSED_FAULT, reason)
        self.n.home.message = reason

    def reset_allowed(self):
        try:
            if not ground_clear(*self.values(time.monotonic())):
                return False, '危险尚未解除，不能继续探索；请人工恢复位置'
        except ValueError as e:
            return False, str(e)
        self.phase = 'idle'
        return True, ''

    def begin(self, kind, values, now):
        n = self.n
        if not self.assume_rear_safe:
            raise ValueError('未允许假定后方地面安全，请人工处理')
        if now-self.last_success < 10.:
            raise ValueError('短时间重复触发，停止重试，请人工处理')
        pose = n.home.current_pose()
        if pose is None:
            raise ValueError('无法记录危险位置，保持停车')
        x, y, yaw = pose_tuple(pose)
        points = []
        if kind == 'cliff':
            if values[0] >= .215: points.append((.205, .205))
            if values[1] >= .220: points.append((.205, -.205))
        else:
            points.append((.135+values[2], 0.))
        self.pending = [dict(x=x+px*math.cos(yaw)-py*math.sin(yaw),
                             y=y+px*math.sin(yaw)+py*math.cos(yaw),
                             radius=.04 if kind == 'cliff' else .06,
                             kind=kind, expires=0. if kind == 'cliff' else now+30.)
                        for px, py in points]
        self.kind = kind
        n._pause('hazard_recovery', '检测到台阶/近障，暂停探索并等待导航退出')
        if not n._cancel_client.service_is_ready():
            raise ValueError('导航取消服务不可用，保持停车')
        self.cancel = n._cancel_client.call_async(CancelGoal.Request())
        self.active = True
        self.phase = 'cancelling'
        self.started = now
        self.start_odom = n.home.dock.odom
        self.last_progress = 0.
        self.progress_at = now
        self.clear_since = None
        self.mode(True)
        self.zero()

    def publish_zones(self, now):
        self.zones = [z for z in self.zones if not z['expires'] or z['expires'] > now]
        if now-self.last_zones < .5:
            return
        self.last_zones = now
        msg = PoseArray(); msg.header.frame_id = 'map'
        msg.header.stamp = self.n.get_clock().now().to_msg()
        for z in self.zones:
            p = Pose(); p.position.x=float(z['x']); p.position.y=float(z['y'])
            p.position.z=float(z['radius']); p.orientation.w=1.
            msg.poses.append(p)
        self.zpub.publish(msg)

    def save(self, prefix):
        # Map-specific sidecar: never load another map's coordinates implicitly.
        with open(prefix+'.hazards.json', 'w', encoding='utf-8') as stream:
            json.dump({'frame_id': 'map', 'zones': [z for z in self.zones
                       if z['kind'] == 'cliff']}, stream, ensure_ascii=False, indent=2)

    def tick(self, healthy):
        n = self.n; now = time.monotonic()
        self.publish_zones(now)
        if not self.active and n._state != n.RUNNING:
            return False
        if self.active and n._state != 'hazard_recovery':
            self.active = False; self.phase = 'idle'
            self.zero(); self.mode(False)
            return False
        try:
            values = self.values(now)
            if not self.active:
                kind = hazard_kind(*values)
                if kind is None:
                    return False
                self.gate(healthy, now)
                self.begin(kind, values, now)
                return True
            self.gate(healthy, now)
            n._publish_pause()
            # Reassert ownership; repeated True does not pulse the motor output.
            self.mode(True)
            if self.phase == 'cancelling':
                self.zero()
                if now-self.started > 4.:
                    raise ValueError('导航未在4秒内退出，请人工处理')
                if not self.cancel.done(): return True
                result = self.cancel.result()
                if result is None or result.return_code not in (0, 2):
                    raise ValueError('取消导航失败，保持停车')
                if n.home.active_goals or n.home.last_status < self.started: return True
                if now-self.started < .6: return True
                if now-n.home.last_motion < .4: return True
                self.phase = 'reversing'; self.reverse_at = now
                self.start_odom = n.home.dock.odom; self.progress_at = now
            progress = reverse_progress(self.start_odom, n.home.dock.odom)
            if progress > .15:
                raise ValueError('后退超过0.15米上限，请人工处理')
            # Pure reverse, checked against live rear lidar. Ground is assumed
            # safe only because the operator explicitly selected this policy.
            d = n.home.dock
            old_direction = getattr(d, 'direction', 1)
            try:
                d.direction = -1
                d._obstacle()
            finally:
                d.direction = old_direction
            if self.kind == 'sonar' and (values[0] >= .215 or values[1] >= .220):
                raise ValueError('超声恢复期间新出现台阶，请人工处理')
            if ground_clear(*values):
                self.zero()
                if self.clear_since is None: self.clear_since = now
                if now-self.clear_since >= .5 and self.phase != 'marking':
                    if len(self.zones)+len(self.pending)>128:
                        raise ValueError('危险区域记录已满，请人工处理')
                    self.zones.extend(self.pending)
                    self.last_zones = 0.; self.publish_zones(now)
                    # Hold still while both costmaps receive the new zones.
                    self.phase = 'marking'; self.marked_at = now
            else:
                self.clear_since = None
            if self.phase == 'marking':
                self.zero()
                if not ground_clear(*values):
                    raise ValueError('恢复后测距再次异常，请人工处理')
                if now-self.marked_at >= 2.:
                    self.active=False; self.phase='idle'; self.last_success=now
                    self.mode(False)
                    n._state=n.RUNNING; n._reason='危险已解除并记录，继续探索规划'
                    n._publish_resume()
                return True
            if now-self.reverse_at >= 6. or progress >= .15:
                raise ValueError('后退达到0.15米或6秒上限仍未恢复，请人工处理')
            if self.clear_since is not None:
                return True
            if progress > self.last_progress+.002:
                self.last_progress=progress; self.progress_at=now
            if now-self.progress_at > 2.:
                raise ValueError('后退无里程计进展，请人工处理')
            msg=Twist(); msg.linear.x=-.03
            n._reason='台阶/超声低速直退中；禁止转向'
            n._publish_lease(True); self.cmdpub.publish(msg)
            return True
        except Exception as error:
            self.fail(str(error))
            return True

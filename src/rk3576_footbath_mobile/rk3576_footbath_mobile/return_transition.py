"""Fail-closed SLAM to saved-map localization handoff owned by the gateway."""
import json
import math
import time
from pathlib import Path


def validate_handoff(data):
    for key in ('pose', 'current_pose'):
        if not all(math.isfinite(float(data[key][k])) for k in ('x', 'y', 'yaw')):
            raise ValueError('non-finite handoff pose')
    if not str(data['map_path']).endswith('.yaml'):
        raise ValueError('handoff requires saved map YAML')
    return data


class ReturnTransition:
    def __init__(self, gateway):
        self.g = gateway
        self.pending = None
        self.seen_session = None

    def cancel(self):
        self.pending = None

    def begin(self, data):
        g = self.g
        data = validate_handoff(data.copy())
        path = g.map_store.checked_yaml(data['map_path'])
        g._launch('navigation', str(path), return_home=data['pose'])
        self.pending = data
        self.deadline = time.monotonic() + 75.0
        self.stage = 'localizing'
        self.stable_since = None
        g.return_requested = True
        g.nav_message = '地图已保存；建图已停止，正在启动定位导航返航'

    def restore(self, request, response):
        try:
            if self.g.mode != 'idle' or self.pending:
                raise ValueError('restore requires idle gateway')
            path = self.g.ws / 'maps/pending_return_handoff.json'
            data = json.loads(path.read_text())
            boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            if data['boot'] != boot or not 0 <= time.monotonic()-data['updated_at'] <= 600:
                raise ValueError('saved handoff expired or board rebooted')
            self.begin(data)
            path.rename(path.with_suffix('.consumed.json'))
            response.success = True
            response.message = '已恢复原起点；等待定位确认后返航'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def tick(self):
        g = self.g
        if self.pending is None:
            status = g.home_status
            if (g.mode in ('mapping', 'auto_mapping')
                    and status.get('phase') == 'handoff_ready'
                    and time.monotonic()-g.home_seen < 3
                    and status.get('session_started_at') != self.seen_session):
                self.seen_session = status['session_started_at']
                try:
                    self.begin(status)
                except Exception as error:
                    g.nav_message = '切换导航失败，保持停车: ' + str(error)
                    g._service('stop')
            return
        if g.mode != 'navigation':
            self.cancel()
            return
        if time.monotonic() > self.deadline:
            self.cancel()
            g._service('stop')
            g.return_requested = False
            g.nav_state = 'return_failed'
            g.nav_message = '定位导航切换超时，未发出返航运动目标'
            return
        if self.stage == 'localizing':
            if (g.initial_state == 'not_set' and g.count_subscribers('/initialpose')
                    and all(g.healthy(k) for k in ('map', 'scan', 'odom'))):
                g._initial(self.pending['current_pose'], handoff=True)
            if g.initial_state == 'failed':
                self.deadline = 0
            if (g.initialized and g.pose_map and g.home_status.get('available')
                    and time.monotonic()-g.home_seen < 3):
                seed = self.pending['current_pose']
                pose = g.pose_map
                error = math.hypot(pose['x']-seed['x'], pose['y']-seed['y'])
                angle = abs(math.atan2(math.sin(pose['yaw']-seed['yaw']),
                                      math.cos(pose['yaw']-seed['yaw'])))
                if error > .35 or angle > .35:
                    g.nav_message = 'AMCL位姿与切换前相差过大，禁止返航'
                    self.deadline = 0
                    return
                if self.stable_since is None:
                    self.stable_since = time.monotonic()
                    return
                if time.monotonic()-self.stable_since < 1.0:
                    return
                if not self.pending.get('auto_request', True):
                    self.pending = None
                    g.return_requested = False
                    g.nav_message = '定位切换静态检查完成，等待明确返航指令'
                    return
                client = g.exp_clients['return_home']
                if client.service_is_ready():
                    from std_srvs.srv import Trigger
                    self.future = client.call_async(Trigger.Request())
                    self.stage = 'requested'
            else:
                self.stable_since = None
        elif self.stage == 'requested' and self.future.done():
            try:
                response = self.future.result()
                if not response.success:
                    self.stage = 'localizing'
                    return
                g.nav_message = response.message
                self.pending = None
            except Exception as error:
                g.nav_message = '返航请求失败: ' + str(error)
                self.deadline = 0

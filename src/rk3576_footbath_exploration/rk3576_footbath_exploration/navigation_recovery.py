"""Fresh, token-bound handshake with the owner of a mobile navigation goal."""
import json
import time
import uuid

from std_msgs.msg import String


class NavigationRecoveryClient:
    def __init__(self, node):
        self.context = None
        self.seen = -float('inf')
        self.publisher = node.create_publisher(String, '/mobile/navigation_recovery', 10)
        node.create_subscription(String, '/mobile/navigation_context', self.receive, 1)

    def receive(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            for key in ('session', 'token', 'phase'):
                if not isinstance(value.get(key), str) or len(value[key]) > 128:
                    return
            self.context, self.seen = value, time.monotonic()
        except (ValueError, TypeError):
            pass

    def available(self, now):
        c = self.context
        return bool(c and now-self.seen <= 1.0 and c['phase'] == 'active'
                    and c['session'] and c['token'])

    def capture(self, now):
        if not self.available(now):
            raise ValueError('普通导航目标上下文不可用，保持停车')
        return dict(session=self.context['session'], token=self.context['token'],
                    recovery_id=uuid.uuid4().hex)

    def send(self, action, identity):
        self.publisher.publish(String(data=json.dumps(dict(identity, action=action))))

    def phase(self, identity, now):
        c = self.context
        if (not c or now-self.seen > 1.0 or
                any(c.get(k) != identity[k] for k in ('session', 'token'))):
            raise ValueError('导航目标已取消、改变或所属进程失联，禁止自动重发')
        if c.get('phase') in ('idle', 'canceled', 'failed', 'succeeded', 'aborted', 'rejected'):
            raise ValueError('原导航任务已结束，禁止自动重发')
        if c.get('recovery_id') != identity['recovery_id']:
            return 'awaiting_owner'
        return c['phase']

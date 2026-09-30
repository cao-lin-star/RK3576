"""Pure goal ownership state for the mobile / supervisor recovery handshake.

A logical goal keeps its token while a cancelled Nav2 attempt is replaced after
recovery. Every cancellation invalidates that attempt before asynchronous action
callbacks can arrive. Tokens never survive user cancellation or a mode change.
"""
import uuid


class NavigationOwner:
    BUSY = frozenset(('sending', 'active', 'suspended', 'resuming'))

    def __init__(self):
        self.session = uuid.uuid4().hex
        self.token = ''
        self.phase = 'idle'
        self.goal = None
        self.recovery_id = ''
        self.attempt = 0
        self.message = ''

    def context(self):
        return dict(session=self.session, token=self.token, phase=self.phase,
                    goal=dict(self.goal) if self.goal else None,
                    recovery_id=self.recovery_id, message=self.message)

    def begin(self, goal):
        self.attempt += 1
        self.token = uuid.uuid4().hex
        self.goal = dict(goal)
        self.phase = 'sending'
        self.recovery_id = ''
        self.message = ''
        return self.token, self.attempt

    def current(self, token, attempt):
        return bool(token and token == self.token and attempt == self.attempt)

    def matches(self, command):
        return (isinstance(command, dict) and bool(self.token)
                and command.get('session') == self.session
                and command.get('token') == self.token
                and isinstance(command.get('recovery_id'), str)
                and bool(command['recovery_id']))

    def suspend(self, command):
        if not self.matches(command):
            return False
        if self.phase == 'suspended':
            return self.recovery_id == command['recovery_id']
        if self.phase not in ('sending', 'active'):
            return False
        # Delayed duplicate suspend must not cancel the replacement attempt.
        if self.recovery_id == command['recovery_id']:
            return False
        self.attempt += 1
        self.phase = 'suspended'
        self.recovery_id = command['recovery_id']
        return True

    def resume(self, command):
        if (not self.matches(command) or self.phase != 'suspended'
                or self.recovery_id != command['recovery_id']):
            return None
        self.attempt += 1
        self.phase = 'resuming'
        return self.token, self.attempt

    def finish(self, phase, message=''):
        self.phase = phase
        self.message = message

    def invalidate(self, message='', phase='canceled'):
        self.attempt += 1
        self.token = ''
        self.goal = None
        self.phase = phase
        self.recovery_id = ''
        self.message = message

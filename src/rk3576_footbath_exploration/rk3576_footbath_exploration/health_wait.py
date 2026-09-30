"""Stop on transient health loss without discarding the current task."""
import math


class HealthWaitMixin:
    def health_problem(self, healthy, now, owner):
        # Malformed commands and emergency stops require operator review.
        # Encoder/overrun faults wait for MCU recovery and stable healthy data.
        # Their occurrence revokes automatic continuation even if the bit clears.
        fatal = self.fault & (256 | 512)
        if fatal:
            raise ValueError(f'急停或控制故障 fault_flags={self.fault}，需人工重新启动任务')
        if (self.source is not None and self.source >= 2) or now-self.manual_at < .5:
            raise ValueError('人工控制接管，取消自动继续任务')
        if now-self.fault_at > 2.5:
            return f'底盘诊断失联 {now-self.fault_at:.1f}秒'
        if not self.bridge_connected or not math.isfinite(self.heartbeat_age) or not 0 <= self.heartbeat_age <= 2.5:
            return '底盘串口断开或心跳超时'
        if self.fault & ~1024:
            return f'底盘故障 fault_flags={self.fault}'
        if self.source is None or now-self.source_at > .8:
            return '底盘控制来源遥测中断'
        d = self.n.home.dock
        if not healthy:
            ages = getattr(self.n, '_ages', {})
            limits = getattr(self.n, '_maximum_age', {})
            if isinstance(ages, dict) and isinstance(limits, dict):
                names = {'scan': '雷达', 'map': '地图', 'odom': '里程计'}
                stale = [f'{names[k]}更新延迟 {ages.get(k, math.inf):.2f}秒'
                         for k in names if ages.get(k, math.inf) > limits.get(k, math.inf)]
                if stale:
                    return '；'.join(stale)
            return '导航服务未就绪或输入尚未恢复'
        if self.n.home.current_pose() is None:
            return '定位更新异常：'+str(self.n.home.pose_error)
        if d.odom is None or now-d.odom_at > .5:
            return f'底盘里程计更新延迟 {now-d.odom_at:.2f}秒'
        try:
            self.retreat_values(now, owner)
            if self.side_enabled and (self.health_wait_at is not None or
                    self.phase not in ('cancelling', 'confirming')):
                self.sides_clear(now)  # False means a real obstacle, not a communication fault.
        except ValueError as error:
            return str(error)
        return None

    def wait_for_health(self, healthy, now, owner):
        problem = self.health_problem(healthy, now, owner)
        if problem and self.health_wait_at is None:
            self.zero()
            if not self.active:
                self.begin('health_wait', (math.nan,)*3, now, owner)
            self.health_wait_at = now
            self.health_clear_at = None
            self.n.get_logger().warning('task health wait: '+problem)
        if self.health_wait_at is None:
            return False
        # Validate ownership even while waiting. Operator cancellation or a new
        # goal must never be undone by a later healthy sample.
        if self.owner == 'home_return' and not self.n.home.hazard_resume_valid(self.owner_token):
            raise ValueError('原返航已取消或超过总时限，不自动恢复')
        if self.owner == 'navigation':
            phase = self.nav_owner.phase(self.owner_token, now)
            if phase not in ('awaiting_owner', 'suspended', 'suspending', 'active', 'resuming'):
                raise ValueError('原导航任务已失效，不自动恢复')
        self.zero()
        self.n._publish_pause()
        if problem:
            self.health_clear_at = None
            self.n._reason = '已停车等待恢复：'+problem+'；恢复稳定2秒后继续原任务'
            return True
        if self.health_clear_at is None:
            self.health_clear_at = now
        self.n._reason = '通信和传感器已恢复，停车确认稳定2秒后继续原任务'
        if now-self.health_clear_at < 2.:
            return True
        elapsed = now-self.health_wait_at
        # Hardware waiting does not consume the mapping exploration budget.
        if self.owner == 'exploration':
            began = getattr(self.n, '_exploration_started_at', None)
            if isinstance(began, (int, float)):
                self.n._exploration_started_at = began + elapsed
        # Freeze stage time budgets, but not original task deadlines, traveled
        # distance, rear-ground evidence age, or ownership tokens. Mapping wait
        # time is excluded above; navigation/home-return deadlines stay bounded.
        for name in ('started', 'reverse_at', 'progress_at', 'confirm_at',
                     'resume_at', 'retreat_wait_at', 'near_extension_at',
                     'corridor_started', 'corridor_progress_at'):
            value = getattr(self, name, None)
            if isinstance(value, (int, float)):
                setattr(self, name, value+elapsed)
        if self.phase == 'cancelling':
            from action_msgs.srv import CancelGoal
            self.cancel_future = self.n._cancel_client.call_async(CancelGoal.Request())
            self.cancel_at = now
        if self.phase == 'marking_recheck':
            self.recheck_last = now
            self.recheck_clear_at = None
            self.recheck_started = now
        if self.phase == 'marking':
            # Require fresh application acknowledgements after a service outage.
            self.marked_at = now
            self.marked_stamp = self.publish_zones(now, force=True)
        if self.phase == 'confirming':
            self.confirmed_points = {}
        self.confirmation.reset()
        self.clear_confirmation.reset()
        self.clear_since = None
        self.source_wait_since = None
        self.source_fresh_since = None
        self.health_wait_at = self.health_clear_at = None
        self.n.get_logger().info('task health restored: resume original owner and recovery stage')
        return False

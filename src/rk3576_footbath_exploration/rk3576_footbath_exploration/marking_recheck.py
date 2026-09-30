"""Keep obstacle application/release rechecks stopped and task-owned."""
import math


class MarkingRecheckMixin:
    def begin_marking_recheck(self, now, detail):
        self.zero()
        self.phase = 'marking_recheck'
        self.marking_retry_active = True
        self.recheck_started = self.recheck_last = now
        self.recheck_clear_at = None
        self.recheck_retry_at = None
        self.clear_since = None
        self.confirm_at = now
        self.confirmation.reset()
        self.clear_confirmation.reset()
        self.confirmed_points = {}
        self.n._reason = '恢复条件再次变化，停车自动复核：'+detail
        self.n.get_logger().warning(self.n._reason)

    def marking_tick(self, values, now, progress):
        n = self.n
        self.zero()
        clear = self.clear_for_resume(values, now)
        if self.phase == 'marking' and not clear:
            self.begin_marking_recheck(now, self.resume_detail)
            return True
        if self.phase == 'marking_recheck':
            # Exclude stationary rechecks from mapping and motion time budgets.
            # Distance, frozen ground evidence and original task ownership remain.
            elapsed = max(0., now-self.recheck_last)
            self.recheck_last = now
            self.reverse_at += elapsed
            if self.owner == 'exploration':
                began = getattr(n, '_exploration_started_at', None)
                if isinstance(began, (int,float)): n._exploration_started_at = began+elapsed
            if clear:
                if self.recheck_clear_at is None: self.recheck_clear_at = now
                n._reason = '恢复条件已解除，停车稳定确认后继续原任务'
                if now-self.recheck_clear_at < .8 or now-n.home.last_motion < .6:
                    return True
                self.phase = 'marking'
                self.marked_at = now
                self.marked_stamp = self.publish_zones(now, force=True)
                self.clear_since = now
                return True
            self.recheck_clear_at = None
            detail = self.resume_detail
            n._reason = '恢复条件尚未解除，保持停车自动复核：'+detail
            # A valid but unsafe TOF is a ground hazard, not a reason to rotate
            # or infer new ground. Await release; existing health wait owns invalid data.
            if (self.kind != 'sonar' or
                    any(not math.isfinite(v) or not 0<v<=.26 for v in values[:2]) or
                    now-self.recheck_started < 1. or now-n.home.last_motion < .6):
                return True
            # New near sources need settled, timestamped confirmation before movement.
            if self.near_sources(now) and not self.confirm_near(now):
                n._reason = '恢复复核发现近障，保持停车确认：'+self.confirm_detail
                return True
            if self.recheck_retry_at is not None and now-self.recheck_retry_at < 3.:
                return True
            self.recheck_retry_at = now
            # Never reset the 30cm aggregate limit or 20s moving-time budget.
            remaining = self.sonar_total_limit-progress
            if remaining < .01 or now-self.reverse_at >= 20.:
                n._reason = '本轮安全后退额度已用完，保留任务停车等待空间恢复：'+detail
                return True
            distance = min(.05,remaining)
            try:
                try:
                    available = self.near_original_trace.plan(now,n.home.dock.odom,distance,distance+.02)
                    if available < distance: raise ValueError('原路长度不足')
                    self.rear_sweep_clear(distance)
                except ValueError:
                    self.short_rear_plan(now,(distance,),self.near_ground_history)
            except ValueError as error:
                n._reason = '安全退路暂不可用，保留任务每3秒复核：'+str(error)+'；'+detail
                return True
            self.sonar_reverse_limit = min(self.sonar_total_limit,progress+distance)
            self.near_extension_at = None
            self.progress_at = now
            self.last_progress = progress
            self.phase = 'reversing'
            n._reason = '复核后安全退路可用，继续本轮最多5cm短退：'+detail
            return True  # No movement until next tick repeats all live safety gates.
        # Lack of costmap acknowledgement is recoverable, but never permission
        # to move. Republish only after checking existing acknowledgements.
        if now-self.marked_at >= 2. and self.zones_applied():
            if self.owner == 'navigation':
                self.phase = 'resuming_owner'; self.resume_at = now
                self.nav_owner.send('resume', self.owner_token)
                return True
            if self.owner == 'home_return':
                if not n.home.resume_after_hazard(self.owner_token):
                    raise ValueError('原返航目标已失效，保持停车')
            else:
                n._state = n.RUNNING
                n._reason = '危险已解除并记录，继续探索规划'
                n._publish_resume()
            self.active = False; self.phase = 'idle'; self.last_success = now
            self.mode(False)
            self.owner = None; self.owner_token = None
            return True
        n._reason = '测距已稳定，等待局部和全局地图确认障碍写入后继续原任务'
        if now-self.marked_at >= 5.:
            self.marked_stamp = self.publish_zones(now, force=True)
            self.marked_at = now
            n._reason = '障碍写入确认延迟，已重新发送，保持停车自动等待'
        return True

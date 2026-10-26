"""Bounded obstacle recovery with no simulator obstacle/target inputs."""
class ProgressWatchdog:
    def __init__(self, reassess_s=8., fail_s=35., motion_m=.75):
        self.reassess_s=reassess_s;self.fail_s=fail_s;self.motion_m=motion_m
        self.anchor=None;self.since=None;self.last_reassessment=None;self.events=[];self.failed=False
        self.clear_since=None;self.ever_failed=False

    def update(self,now,xy,mode,completed=False,safe_to_resume=False):
        import math
        blocked=('wait_' in mode or 'obstacle_stop' in mode or 'stale_stop' in mode)
        if self.failed and not completed:
            # Timeout is a protected hold for the current blocked attempt, not
            # permission to bypass an obstacle and not a permanent brake latch.
            # A new one-second continuous fresh/clear sensor interval can begin
            # another attempt inside the unchanged episode deadline.
            if safe_to_resume and mode in {'cruise','target_approach'}:
                if self.clear_since is None:self.clear_since=now
                if now-self.clear_since>=1.-1e-6:
                    self.failed=False;self.clear_since=None
                    self.anchor=list(xy);self.since=now;self.last_reassessment=now
                    self.events.append(dict(event='blocked_route_resumed_after_fresh_clear',timestamp_s=now,
                        evidence='one_second_continuous_fresh_clear_depth_and_unforced_route_control'))
                    return 'resumed'
            else:self.clear_since=None
            return 'failed'
        if completed or not blocked:
            self.anchor=list(xy);self.since=now;self.last_reassessment=now;return None
        if self.anchor is None or math.dist(xy,self.anchor)>self.motion_m:
            self.anchor=list(xy);self.since=now;self.last_reassessment=now;return None
        if now-self.since>=self.fail_s:
            if not self.failed:self.events.append(dict(event='bounded_blockage_failure',timestamp_s=now,mode=mode))
            self.failed=True;self.ever_failed=True;return 'failed'
        if now-self.last_reassessment>=self.reassess_s:
            self.last_reassessment=now;self.events.append(dict(event='blocked_route_reassessment',timestamp_s=now,mode=mode))
            return 'reassess'
        return None

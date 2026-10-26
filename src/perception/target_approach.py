"""Sensor-only confirmation and stopped-approach evidence for E7 experiments.

This monitor never accepts simulator target identity or actor state. Its claim
is a sensor-relative approach; correctness is audited separately after the run.
"""
from math import hypot


class TargetApproachMonitor:
    def __init__(self, association_radius_m=6., freshness_s=1.5, minimum_depth_m=3.,
                 maximum_depth_m=8., maximum_speed_mps=.3, hold_seconds=2.):
        self.radius=association_radius_m;self.freshness=freshness_s
        self.minimum=minimum_depth_m;self.maximum=maximum_depth_m
        self.speed_limit=maximum_speed_mps;self.hold_required=hold_seconds
        self.latest=None;self.confirmed=False;self.completed=False
        self.hold_start=None;self.events=[];self.reference_xyz=None
        self.braking_started=False

    def observe(self, candidates, timestamp_s, reference_xyz):
        if reference_xyz is None:return
        matches=[c for c in candidates if c.get('temporal_confirmed')
            and c.get('target_color_match',False) and c.get('category_match_score',0)>=.5
            and c.get('depth_m') is not None and c.get('world_position_xyz') is not None
            and hypot(c['world_position_xyz'][0]-reference_xyz[0],
                      c['world_position_xyz'][1]-reference_xyz[1])<=self.radius]
        continuation=False
        if not matches and self.confirmed and self.latest is not None and timestamp_s-self.latest['timestamp_s']<=self.freshness:
            def iou(a,b):
                if not a or not b:return 0.
                inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
                return inter/max(1e-6,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)
            matches=[c for c in candidates if c.get('category') in ['unknown',self.latest['category']]
                and c.get('color') in ['unknown',self.latest['color']]
                and c.get('depth_m') is not None and c.get('world_position_xyz') is not None
                and hypot(c['world_position_xyz'][0]-self.latest['world_xyz'][0],c['world_position_xyz'][1]-self.latest['world_xyz'][1])<=2.5
                and iou(c.get('bbox_xyxy'),self.latest.get('bbox_xyxy'))>=.2]
            continuation=bool(matches)
        if not matches:return
        c=max(matches,key=lambda v:v.get('candidate_score',0))
        previous=self.latest
        self.latest={'timestamp_s':timestamp_s,'depth_m':float(c['depth_m']),
                     'world_xyz':list(c['world_position_xyz']),'candidate_id':c['candidate_id'],
                     'category':previous['category'] if continuation else c.get('category'),
                     'color':previous['color'] if continuation else c.get('color'),
                     'bbox_xyxy':c.get('bbox_xyxy'),
                     'observed_category':c.get('category'),'observed_color':c.get('color'),
                     'tracking_provenance':'geometric_continuation_after_semantic_confirmation' if continuation else 'fresh_semantic_confirmation'}
        if not self.confirmed:
            self.confirmed=True
            self.events.append({'event':'sensor_local_confirmation',**self.latest})

    def update(self, timestamp_s, own_speed_mps, reference_xyz):
        if reference_xyz is not None:
            if self.reference_xyz is not None and hypot(reference_xyz[0]-self.reference_xyz[0],reference_xyz[1]-self.reference_xyz[1])>self.radius:
                self.confirmed=False;self.completed=False;self.hold_start=None;self.latest=None;self.braking_started=False
                self.events.append({'event':'target_reference_changed','timestamp_s':timestamp_s,'reference_xyz':list(reference_xyz)})
            self.reference_xyz=list(reference_xyz)
        if self.completed:return True
        fresh=self.latest is not None and 0<=timestamp_s-self.latest['timestamp_s']<=self.freshness
        associated=fresh and reference_xyz is not None and hypot(
            self.latest['world_xyz'][0]-reference_xyz[0],self.latest['world_xyz'][1]-reference_xyz[1])<=self.radius
        eligible=associated and self.minimum<=self.latest['depth_m']<=self.maximum and own_speed_mps<=self.speed_limit
        if not eligible:self.hold_start=None;return False
        if self.hold_start is None:self.hold_start=timestamp_s
        if timestamp_s-self.hold_start+1e-6>=self.hold_required:
            self.completed=True
            self.events.append({**self.latest,'event':'sensor_safe_approach_hold_completed','timestamp_s':timestamp_s,
                'speed_mps':own_speed_mps,'hold_seconds':timestamp_s-self.hold_start,
                'depth_definition':'UGV RGB-D optical-axis target surface depth in metres'})
        return self.completed

    def braking_request(self, timestamp_s, reference_xyz):
        """Brake before the confirmed target leaves the valid surface range.

        The route endpoint is based on a remote position estimate; it cannot
        guarantee a safe gap to vehicles of different lengths. Use a fresh,
        locally confirmed RGB-D target surface to stop at 4.5m, giving margin
        above the unchanged3m minimum. No simulator target geometry is read.
        This command only strengthens the already-computed safety control.
        """
        if not self.confirmed or self.latest is None or reference_xyz is None:return False
        fresh=0<=timestamp_s-self.latest['timestamp_s']<=self.freshness
        associated=hypot(self.latest['world_xyz'][0]-reference_xyz[0],self.latest['world_xyz'][1]-reference_xyz[1])<=self.radius
        requested=bool(fresh and associated and self.latest['depth_m']<=4.5)
        if requested and not self.braking_started:
            self.braking_started=True
            self.events.append({'event':'sensor_target_braking_started',**self.latest,
                'timestamp_s':timestamp_s,'braking_surface_threshold_m':4.5})
        return requested

    def summary(self):
        return {'local_confirmation':self.confirmed,'sensor_safe_approach':self.completed,
                'correct_identity':'requires independent post-run audit',
                'criterion':{'depth_m':[self.minimum,self.maximum],'maximum_speed_mps':self.speed_limit,
                             'hold_seconds':self.hold_required,'freshness_s':self.freshness},'events':self.events}

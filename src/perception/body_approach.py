"""R10 observable stages and body-clearance completion; no target truth input."""
from collections import deque
from math import hypot


def bbox_iou(a,b):
    inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    return inter/max(1e-6,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter)


class BodyApproachMonitor:
    def __init__(self):
        self.latest=None;self.confirmed=False;self.completed=False;self.hold_start=None
        self.events=[];self.history=deque(maxlen=5);self.hold_observations=deque()
        self.reference_xyz=None;self.braking_started=False;self.observations=[]
        self.hold_samples=[]
        self.last_observe_timestamp=None

    def observe(self,candidates,timestamp_s,reference_xyz):
        if self.last_observe_timestamp is not None and timestamp_s<=self.last_observe_timestamp:return
        self.last_observe_timestamp=timestamp_s
        eligible=[c for c in candidates if c.get('target_color_match')
            and c.get('category_match_score',0)>=.5 and c.get('world_position_xyz') is not None
            and c.get('body_clearance',{}).get('valid')
            and min(c['bbox_xyxy'][2]-c['bbox_xyxy'][0],c['bbox_xyxy'][3]-c['bbox_xyxy'][1])>=24
            and c['body_clearance']['surface_horizontal_distance_m']<=20
            and (reference_xyz is None or hypot(c['world_position_xyz'][0]-reference_xyz[0],
                                               c['world_position_xyz'][1]-reference_xyz[1])<=6)]
        continuation=False
        if not eligible and self.confirmed and self.latest is not None and 0<=timestamp_s-self.latest['timestamp_s']<=.5:
            eligible=[c for c in candidates if c.get('category') in ['unknown',self.latest['category']]
                and c.get('color') in ['unknown',self.latest['color']]
                and c.get('world_position_xyz') is not None and c.get('body_clearance',{}).get('valid')
                and min(c['bbox_xyxy'][2]-c['bbox_xyxy'][0],c['bbox_xyxy'][3]-c['bbox_xyxy'][1])>=24
                and c['body_clearance']['surface_horizontal_distance_m']<=20
                and hypot(c['world_position_xyz'][0]-self.latest['world_xyz'][0],c['world_position_xyz'][1]-self.latest['world_xyz'][1])<=2.5
                and bbox_iou(c['bbox_xyxy'],self.latest['bbox_xyxy'])>=.2]
            continuation=bool(eligible)
        c=max(eligible,key=lambda c:c.get('candidate_score',0),default=None)
        self.history.append((timestamp_s,c))
        if c is None:return
        xyz=c['world_position_xyz']
        hits=[(t,a) for t,a in self.history if a is not None and 0<=timestamp_s-t<=1.
              and hypot(a['world_position_xyz'][0]-xyz[0],a['world_position_xyz'][1]-xyz[1])<=2.5
              and a.get('category')==c.get('category') and a.get('color')==c.get('color')]
        if len(hits)<3 and self.confirmed and self.latest is not None and 0<=timestamp_s-self.latest['timestamp_s']<=.5:
            if (c.get('category')==self.latest['category'] and c.get('color')==self.latest['color']
                    and bbox_iou(c['bbox_xyxy'],self.latest['bbox_xyxy'])>=.2
                    and hypot(xyz[0]-self.latest['world_xyz'][0],xyz[1]-self.latest['world_xyz'][1])<=2.5):continuation=True
        if len(hits)<3 and not continuation:return
        previous=self.latest
        self.latest={'timestamp_s':timestamp_s,'depth_m':float(c['depth_m']),
                     'world_xyz':list(xyz),'candidate_id':c['candidate_id'],
                     'category':previous['category'] if continuation else c.get('category'),
                     'color':previous['color'] if continuation else c.get('color'),
                     'observed_category':c.get('category'),'observed_color':c.get('color'),'bbox_xyxy':c['bbox_xyxy'],
                     'body_clearance':c['body_clearance'],'frame_id':c.get('frame_id'),
                     'estimated_body_clearance_m':c['body_clearance']['estimated_body_clearance_m'],
                     'tracking_provenance':'geometric_continuation_after_near_semantic_confirmation' if continuation else 'three_of_five_fresh_matching_local_observations'}
        self.observations.append(dict(self.latest))
        if not self.confirmed:
            self.confirmed=True;self.events.append({'event':'sensor_local_confirmation',**self.latest})
        if self.hold_start is not None:self.hold_observations.append(timestamp_s)

    def update(self,timestamp_s,own_speed_mps,reference_xyz,legal_stop=True):
        if reference_xyz is not None:
            if self.reference_xyz is not None and hypot(reference_xyz[0]-self.reference_xyz[0],reference_xyz[1]-self.reference_xyz[1])>6:
                self.confirmed=False;self.completed=False;self.hold_start=None;self.latest=None
                self.history.clear();self.hold_observations.clear();self.braking_started=False
                self.events.append(dict(event='target_reference_changed',timestamp_s=timestamp_s))
            self.reference_xyz=list(reference_xyz)
        if self.completed:return True
        fresh=self.latest is not None and 0<=timestamp_s-self.latest['timestamp_s']<=.5
        associated=fresh and reference_xyz is not None and hypot(self.latest['world_xyz'][0]-reference_xyz[0],self.latest['world_xyz'][1]-reference_xyz[1])<=6
        eligible=associated and legal_stop and own_speed_mps<=.3 and 4<=self.latest['estimated_body_clearance_m']<=7
        if not eligible:self.hold_start=None;self.hold_observations.clear();return False
        if self.hold_start is None:self.hold_start=timestamp_s
        self.hold_samples.append(dict(timestamp_s=timestamp_s,hold_start_s=self.hold_start,
            estimated_body_clearance_m=self.latest['estimated_body_clearance_m'],
            speed_mps=own_speed_mps,legal_stop=bool(legal_stop)))
        if timestamp_s-self.hold_start>=2.-1e-6 and len(self.hold_observations)>=6:
            times=[self.hold_start,*self.hold_observations,timestamp_s]
            if max(b-a for a,b in zip(times,times[1:]))<=.5+1e-6:
                self.completed=True;self.events.append({**self.latest,'event':'sensor_safe_approach_hold_completed',
                    'timestamp_s':timestamp_s,'hold_start_s':self.hold_start,'hold_seconds':timestamp_s-self.hold_start,
                    'speed_mps':own_speed_mps,'valid_hold_observations':len(self.hold_observations),
                    'depth_definition':'minimum body clearance estimated from visible RGB-D target surface'})
        return self.completed

    def braking_request(self,timestamp_s,reference_xyz):
        if not self.confirmed or self.latest is None or reference_xyz is None:return False
        associated=hypot(self.latest['world_xyz'][0]-reference_xyz[0],self.latest['world_xyz'][1]-reference_xyz[1])<=6
        requested=bool(0<=timestamp_s-self.latest['timestamp_s']<=.5 and associated and self.latest['estimated_body_clearance_m']<=5.3)
        if requested and not self.braking_started:
            self.braking_started=True;self.events.append({**self.latest,'event':'sensor_target_braking_started',
                'timestamp_s':timestamp_s,'braking_body_clearance_threshold_m':5.3})
        return requested

    def summary(self):
        return dict(protocol='R10-body-clearance-v1',local_confirmation=self.confirmed,sensor_safe_approach=self.completed,
            correct_identity='requires independent post-run audit',criterion=dict(body_clearance_m=[3,8],
                online_hold_m=[4,7],maximum_speed_mps=.3,hold_seconds=2.,freshness_s=.5,
                local_range_m=20.,minimum_local_bbox_side_px=24),events=self.events,observations=self.observations,
            hold_samples=self.hold_samples,
            stopping_semantics='temporary stopped confirmation on drivable footprint; junction location reported separately, not road-parking legality')

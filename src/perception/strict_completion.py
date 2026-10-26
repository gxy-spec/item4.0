"""Independent post-run S3 verifier. Never import this into online inference.

Inputs include evaluation-only body geometry and instance associations. Missing,
duplicate, stale or non-finite evidence fails closed instead of granting success.
"""
import math
import numpy as np
import cv2
from .body_clearance import polygon_clearance


def body_polygon(state):
    vertices=np.asarray(state.get('bbox_world_vertices_xyz',[]),dtype=float)
    if vertices.shape!=(8,3) or not np.isfinite(vertices).all():return None
    p=cv2.convexHull(vertices[:,:2].astype(np.float32)).reshape(-1,2)
    return p.tolist() if len(p)>=3 else None


def verify_completion(event,own_states,target_states,local_observations,*,
                      origin_s,expected_actor_id,episode_collisions,
                      s1_pass,s2_pass,expected_present=True,fixed_delta_s=.05):
    checks={};detail={}
    def check(name,value):checks[name]=bool(value)
    check('positive_task',expected_present is True)
    check('reliable_discovery',s1_pass is True)
    check('correct_near_confirmation',s2_pass is True)
    check('no_episode_collision',episode_collisions==0)
    try:
        start=float(event['hold_start_s']);end=float(event['timestamp_s'])
        check('two_second_claim',math.isfinite(start) and math.isfinite(end) and end-start>=2.-1e-6 and start>=0)
        own=[s for s in own_states if start-1e-6<=float(s['timestamp'])-origin_s<=end+1e-6]
        times=[float(s['timestamp'])-origin_s for s in own];frames=[s['frame'] for s in own]
        check('unique_physical_frames',len(frames)==len(set(frames)))
        check('full_hold_endpoints',bool(times) and abs(times[0]-start)<=1e-6 and abs(times[-1]-end)<=1e-6)
        check('continuous_physical_ticks',len(times)>=41 and all(abs(b-a-fixed_delta_s)<=1e-5 for a,b in zip(times,times[1:])))
        gaps=[];speeds=[];geometry_complete=bool(own);identities=True
        for s in own:
            t=target_states.get(s['frame']);pa=body_polygon(s);pb=body_polygon(t) if t else None
            if pa is None or pb is None:geometry_complete=False;continue
            identities=identities and t.get('actor_id')==expected_actor_id
            gaps.append(polygon_clearance(pa,pb));speeds.append(float(s['speed_mps']))
        check('complete_body_geometry',geometry_complete)
        check('expected_target_geometry',identities and expected_actor_id is not None)
        check('true_body_gap',bool(gaps) and all(math.isfinite(g) and 3<=g<=8 for g in gaps))
        check('stationary_speed',bool(speeds) and all(math.isfinite(v) and 0<=v<=.3 for v in speeds))
        check('drivable_entire_hold',bool(own) and all(s.get('map_drivable_footprint') is True for s in own))
        obs=[a for a in local_observations if start-1e-6<=float(a['timestamp_s'])<=end+1e-6]
        ot=[float(a['timestamp_s']) for a in obs];keys=[(a.get('frame_id'),a.get('candidate_id')) for a in obs]
        check('six_distinct_observations',len(obs)>=6 and len(set(keys))==len(keys) and all(k[0] is not None and k[1] for k in keys))
        check('strictly_increasing_observation_times',all(b>a for a,b in zip(ot,ot[1:])))
        check('fresh_observations_throughout_hold',bool(ot) and all(-1e-6<=b-a<=.5+1e-6 for a,b in zip([start,*ot],[*ot,end])))
        check('correct_expected_instance',bool(obs) and all(a.get('identity_correct') is True and a.get('matched_actor_id')==expected_actor_id for a in obs))
        estimates=[a.get('estimated_body_clearance_m') for a in obs]
        check('valid_online_gap_evidence',bool(estimates) and all(g is not None and math.isfinite(float(g)) and 4<=float(g)<=7 for g in estimates))
        detail.update(start_s=start,end_s=end,physical_frames=len(own),observations=len(obs),
            minimum_true_gap_m=min(gaps,default=None),maximum_true_gap_m=max(gaps,default=None),
            maximum_speed_mps=max(speeds,default=None))
    except (KeyError,TypeError,ValueError,IndexError,OverflowError):
        check('well_formed_evidence',False)
    return dict(passed=all(checks.values()),checks=checks,failed_checks=[k for k,v in checks.items() if not v],detail=detail,
        protocol='L11-strict-independent-completion-v1')

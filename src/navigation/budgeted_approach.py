"""Task-point observation-region alternative, selected only for an over-budget route.

Inputs contain a received sensor point, map, own geometry and prior own speeds.
This module is prepared separately; it has no effect until called by a frozen
runner. Local RGB-D safety and independent S3 evaluation remain mandatory.
"""
from __future__ import annotations
import math,statistics
from .approach_region import plan_observation_region


def length(route):
    return sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:]))


def observed_budget_speed(history,nominal_speed,now):
    recent=[]
    for row in history:
        t=float(row.get('timestamp',-1e12));v=float(row.get('ugv_speed_mps',0.))
        if now-15<=t<=now and row.get('ugv_safety_mode')=='cruise' and .5<=v<=1.5*nominal_speed:
            recent.append((t,v))
    span=max((t for t,v in recent),default=0)-min((t for t,v in recent),default=0)
    mature=len(recent)>=20 and span>=5.
    speed=min(float(nominal_speed),statistics.median(v for t,v in recent)) if mature else float(nominal_speed)
    return max(.5,speed),dict(mature=mature,samples=len(recent),span_s=span)


def select_task_route(world_map,start_location,target_xyz,own_polygon,original_route,
                      remaining_s,nominal_speed,history,now,*,candidate_planner=plan_observation_region):
    speed,speed_evidence=observed_budget_speed(history,nominal_speed,now)
    original_length=length(original_route);reserve=5.
    audit=dict(accepted=False,reason='original_route_within_budget',received_point_xyz=list(target_xyz),
        remaining_s=float(remaining_s),planning_speed_mps=speed,speed_evidence=speed_evidence,
        hold_and_brake_reserve_s=reserve,original_route_length_m=original_length,
        estimated_original_seconds=original_length/speed+reserve,
        chosen_end_stop_tolerance_m=3.,candidate_route_length_m=None)
    if original_length/speed+reserve<=remaining_s:return original_route,audit
    audit['reason']='no_candidate_within_budget'
    try:
        map_audit={}
        candidate=candidate_planner(world_map,start_location,target_xyz,own_polygon,audit=map_audit)
        candidate_length=length(candidate);seconds=candidate_length/speed+reserve
        audit.update(candidate_route_length_m=candidate_length,estimated_candidate_seconds=seconds,map_audit=map_audit)
        if len(candidate)>=2 and candidate_length+1.<original_length and seconds<=remaining_s:
            # Candidate terminal is already an OWN-BODY-to-visible-point gap
            # of5..7m. The old3m route-end tolerance must not stop3m before it.
            audit.update(accepted=True,reason='shorter_observation_region_within_budget',chosen_end_stop_tolerance_m=.5)
            return candidate,audit
    except (RuntimeError,ValueError) as ex:audit['query_error']=str(ex)
    return original_route,audit

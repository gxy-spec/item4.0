"""Narrow stationary-goal release; never disables depth or crossing protection."""
from math import hypot
from perception.body_approach import bbox_iou


def release_stationary_goal_conflict(decision,monitor,reference_xyz,now):
    if decision.get('forced_stop_reason')!='vehicle_conflict':return False
    if monitor is None or not monitor.confirmed or monitor.latest is None or reference_xyz is None:return False
    obs=monitor.latest;hazard=decision.get('hazard') or {}
    if not 0<=now-obs['timestamp_s']<=.5:return False
    if hazard.get('kind')!='vehicle' or not hazard.get('track_confirmed'):return False
    # Require a mature robust window. Raw speed still drives crossing rules;
    # no release is attempted for crossing_vehicle, pedestrian, or depth STOP.
    if hazard.get('lead_motion_window_s',0)<.5 or hazard.get('lead_motion_speed_mps',float('inf'))>.3:return False
    if not hazard.get('bbox_xyxy') or bbox_iou(hazard['bbox_xyxy'],obs['bbox_xyxy'])<.5:return False
    p=hazard.get('world_xyz');q=obs.get('world_xyz')
    if p is None or q is None:return False
    if hypot(q[0]-reference_xyz[0],q[1]-reference_xyz[1])>6:return False
    if hypot(p[0]-q[0],p[1]-q[1])>2.5:return False
    if obs.get('estimated_body_clearance_m',float('inf'))>12:return False
    decision['forced_stop_reason']=''
    decision['stationary_goal_conflict_released']=True
    decision['goal_vehicle_approach']=True
    return True

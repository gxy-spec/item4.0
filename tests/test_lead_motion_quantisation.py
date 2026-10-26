"""Regression of the retained gray-truck stop, and moving-vehicle protection."""
from pathlib import Path
import sys
import numpy as np
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
for p in [ROOT/'src',ROOT/'scripts']:
    sys.path.insert(0,str(p))
from safety.rgbd_hazard_perception import RGBDHazardPerception,HazardDetectorConfig
from run_stage1_dynamic import classify_safety_hazards
from safety.goal_approach_guard import release_stationary_goal_conflict


def track(points,yaw=180.):
    detector=RGBDHazardPerception(HazardDetectorConfig(Path('unused')),model=object())
    history=[]
    for i,xy in enumerate(points):
        row=dict(kind='vehicle',world_xyz=[*xy,1.],forward_m=12.,lateral_m=.06,confidence=.8)
        batch=[row];detector._update_tracks(batch,i*.05,{'yaw':yaw});history.append(row)
    return history


def test_gray_truck_depth_steps_do_not_create_perpetual_moving_lead():
    # World-surface positions actually recorded at the stationary R10 truck.
    values=[64.7067,64.4895,64.6739,64.7067]*20
    history=track([(x,66.336) for x in values])
    assert max(r['world_speed_mps'] for r in history[20:])>.8
    assert max(r['lead_motion_speed_mps'] for r in history[20:])<.8
    # General traffic classification is deliberately unchanged. Only a fresh
    # local task confirmation can release a noisy stationary goal conflict.
    assert any(classify_safety_hazards([r]).get('forced_stop_reason')=='vehicle_conflict' for r in history[20:])


def test_longitudinal_moving_lead_is_still_a_conflict():
    history=track([(20.-i*.1,0.) for i in range(30)])
    assert history[-1]['lead_motion_speed_mps']>1.9
    assert classify_safety_hazards([history[-1]])['forced_stop_reason']=='vehicle_conflict'


def test_lateral_crossing_uses_fast_motion_even_with_stationary_history():
    history=track([(10.,0.)]*20+[(10.,i*.075) for i in range(1,12)],yaw=0.)
    row=history[-1]
    assert row['lateral_speed_mps']>.65 and row['lateral_displacement_m']>.5
    assert classify_safety_hazards([row])['forced_stop_reason']=='crossing_vehicle'


def goal_case():
    obs=dict(timestamp_s=10.,world_xyz=[65.5,66.3,1.],bbox_xyxy=[356,224,446,314],estimated_body_clearance_m=8.8)
    monitor=SimpleNamespace(confirmed=True,latest=obs)
    h=dict(kind='vehicle',track_confirmed=True,world_xyz=[64.7,66.3,1.],bbox_xyxy=[356,224,446,314],lead_motion_window_s=.75,lead_motion_speed_mps=.05)
    return dict(forced_stop_reason='vehicle_conflict',hazard=h),monitor,[61.7,66.2,1.]


def test_confirmed_stationary_goal_quantisation_can_release_conflict():
    d,m,p=goal_case()
    assert release_stationary_goal_conflict(d,m,p,10.1)
    assert d['forced_stop_reason']==''


def test_unconfirmed_or_stale_or_other_vehicle_cannot_release():
    for kind in ['unconfirmed','stale','other_vehicle','moving','short_history']:
        d,m,p=goal_case();now=10.1
        if kind=='unconfirmed':m.confirmed=False
        if kind=='stale':now=10.6
        if kind=='other_vehicle':d['hazard']['bbox_xyxy']=[456,224,546,314]
        if kind=='moving':d['hazard']['lead_motion_speed_mps']=1.
        if kind=='short_history':d['hazard']['lead_motion_window_s']=.2
        assert not release_stationary_goal_conflict(d,m,p,now),kind
        assert d['forced_stop_reason']=='vehicle_conflict'


def test_pedestrian_and_crossing_protection_cannot_be_released_for_goal():
    for reason in ['pedestrian','crossing_vehicle','no_safe_overtake_gap']:
        d,m,p=goal_case();d['forced_stop_reason']=reason
        assert not release_stationary_goal_conflict(d,m,p,10.1)
        assert d['forced_stop_reason']==reason

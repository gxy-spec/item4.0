"""Replay retained hazard observations. Diagnostic, not a new robot trajectory."""
from pathlib import Path
import sys,json,csv,collections
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from safety.rgbd_hazard_perception import RGBDHazardPerception,HazardDetectorConfig
from run_stage1_dynamic import classify_safety_hazards
from recovery_stage import OUT,save,digest


def main():
    run=Path('D:/CI10/acceptance/upgraded_gray_truck_C/CI_E7_R10_acc_04_20261023T093027Z')
    poses={int(r['frame']):r for r in csv.DictReader((run/'trajectories/ugv_trajectory.csv').open(encoding='utf-8-sig'))}
    frames=collections.defaultdict(list)
    for r in csv.DictReader((run/'safety/ugv_rgbd_hazard_detections.csv').open(encoding='utf-8-sig')):
        if not r.get('world_xyz'):continue
        row={k:(json.loads(v) if k in ['world_xyz','bbox_xyxy'] else v) for k,v in r.items()}
        for k in ['forward_m','lateral_m','confidence','world_speed_mps','lateral_speed_mps','track_age_s','lateral_displacement_m','signed_lateral_velocity_mps','timestamp']:
            row[k]=float(row[k])
        row['track_confirmed']=row['track_confirmed']=='True'
        frames[int(r['frame'])].append(row)
    detector=RGBDHazardPerception(HazardDetectorConfig(Path('unused')),model=object())
    old_counts=collections.Counter();new_counts=collections.Counter();stationary=[]
    for frame,batch in sorted(frames.items()):
        old=classify_safety_hazards(batch)
        detector._update_tracks(batch,batch[0]['timestamp'],{'yaw':float(poses[frame]['yaw'])})
        new=classify_safety_hazards(batch)
        if batch[0]['timestamp']>=100:
            old_counts[old.get('forced_stop_reason','')]+=1
            new_counts[new.get('forced_stop_reason','')]+=1
            stationary.extend(r for r in batch if r.get('lead_motion_speed_mps') is not None)
    speeds=np.array([r['lead_motion_speed_mps'] for r in stationary])
    result=dict(run_dir=str(run),scope='Same logged RGB-D positions only; no resimulation and no new closed-loop success claim',
        source_sha256=digest(run/'safety/ugv_rgbd_hazard_detections.csv'),
        old_lead_decision_counts=dict(old_counts),replayed_lead_decision_counts=dict(new_counts),
        stable_speed_p95_mps=float(np.percentile(speeds,95)),stable_speed_max_mps=float(max(speeds)),
        frozen_revision='validation2',samples=len(stationary))
    save(OUT/'diagnostics/gray_truck_motion_replay.json',result);print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()

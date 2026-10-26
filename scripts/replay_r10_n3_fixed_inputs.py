"""Supplemental fixed-input safety replay, separate from failed live pairing."""
from pathlib import Path
import sys,json,dataclasses
import cv2,numpy as np,yaml,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,rows,save,digest
from safety.rgbd_hazard_perception import RGBDHazardPerception,HazardDetectorConfig
from run_stage1_dynamic import camera_mount_matrix,classify_safety_hazards
from sensors.depth import carla_depth_to_metres


def main():
    torch.set_num_threads(2)
    pair=json.loads((OUT/'analysis/N3_A_pair_audit.json').read_text(encoding='utf-8'))['runs']
    run=Path(pair['legacy']);cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
    hazard=cfg['dynamic']['ugv_safety']['hazard_detector'];spec=cfg['sensors']['streams']['ugv_depth']
    mount=camera_mount_matrix(spec['location_xyz_m'],spec['rotation_pyr_degrees'])
    config=HazardDetectorConfig(model_path=ROOT/hazard['model_path'],horizontal_fov_degrees=cfg['sensors']['fov_degrees'],
        confidence=hazard['confidence'],image_size=hazard['image_size'],device='cpu')
    # Two independent detector and track states receive exactly the same saved
    # RGB-D and own odometry. No simulator or target actor input is available.
    detectors=[RGBDHazardPerception(config),RGBDHazardPerception(config)]
    own={a['frame']:a for a in rows(run/'actors/actor_states.jsonl') if a['role']=='ugv'}
    frames=[a for a in rows(run/'sensors/ugv/metadata/frames.jsonl') if a.get('sensor_files_saved')][:33]
    records=[]
    for a in frames:
        rgb=cv2.cvtColor(cv2.imread(a['rgb_path']),cv2.COLOR_BGR2RGB)
        raw=cv2.imread(a['depth_raw_path'],cv2.IMREAD_UNCHANGED)
        if raw.shape[2]==3:raw=cv2.cvtColor(raw,cv2.COLOR_BGR2BGRA)
        depth=carla_depth_to_metres(raw.tobytes(),raw.shape[1],raw.shape[0]);outputs=[];decisions=[]
        for detector in detectors:
            out=detector.infer(rgb,depth,mount,own[a['frame']],a['frame'],a['timestamp'])
            outputs.append(out);decisions.append(classify_safety_hazards(out,pedestrian_stop_forward_m=hazard.get('pedestrian_stop_forward_m',0)))
        identical=json.dumps(outputs[0],sort_keys=True)==json.dumps(outputs[1],sort_keys=True)
        same_decision=json.dumps(decisions[0],sort_keys=True)==json.dumps(decisions[1],sort_keys=True)
        records.append(dict(frame=a['frame'],relative_s=a['timestamp']-frames[0]['timestamp'],detections=len(outputs[0]),
            identical_detections=identical,identical_decision=same_decision,forced_stop_reason=decisions[0].get('forced_stop_reason'),
            rgb_sha256=digest(a['rgb_path']),depth_sha256=digest(a['depth_raw_path'])))
    source_manifests={v:json.loads((Path(p)/'config_snapshot/runtime_source_manifest.json').read_text()) for v,p in pair.items()}
    common=['src/safety/rgbd_hazard_perception.py','src/safety/rgbd_obstacle_guard.py','scripts/run_stage1_dynamic.py']
    equal=all(source_manifests['legacy'][p]==source_manifests['upgraded'][p] for p in common)
    result=dict(passed=equal and all(r['identical_detections'] and r['identical_decision'] for r in records),
        shared_acquisition_safety_source_hashes_equal=equal,records=records,device='cpu',model_sha256=digest(config.model_path),
        live_pair_audit_still_failed=True,
        scope='33 saved frames at4Hz, two independent detector/track states, identical RGB-D/own pose. Component replay only; does not reproduce all20Hz live frames or pass the failed closed-loop trajectory audit.')
    save(OUT/'analysis/N3_A_fixed_input_replay.json',result)
    print(json.dumps(dict(passed=result['passed'],frames=len(records),shared_safety_sources_equal=equal)))


if __name__=='__main__':main()

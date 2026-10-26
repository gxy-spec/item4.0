"""Diagnostic replay of saved own-sensor inputs, never simulator control."""
from pathlib import Path
import sys,json,argparse,collections
import cv2,numpy as np,yaml,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,rows,save
from safety.rgbd_hazard_perception import RGBDHazardPerception,HazardDetectorConfig
from run_stage1_dynamic import camera_mount_matrix,classify_safety_hazards
from sensors.depth import carla_depth_to_metres


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);p.add_argument('--last',type=int,default=20);args=p.parse_args()
    run=args.run_dir;torch.set_num_threads(2);cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
    h=cfg['dynamic']['ugv_safety']['hazard_detector'];spec=cfg['sensors']['streams']['ugv_depth']
    mount=camera_mount_matrix(spec['location_xyz_m'],spec['rotation_pyr_degrees'])
    detector=RGBDHazardPerception(HazardDetectorConfig(model_path=ROOT/h['model_path'],horizontal_fov_degrees=cfg['sensors']['fov_degrees'],
        confidence=h['confidence'],image_size=h['image_size'],device='cpu'))
    own={a['frame']:a for a in rows(run/'actors/actor_states.jsonl') if a['role']=='ugv'}
    frames=[a for a in rows(run/'sensors/ugv/metadata/frames.jsonl') if a.get('sensor_files_saved')][-args.last:];result=[]
    for a in frames:
        rgb=cv2.cvtColor(cv2.imread(a['rgb_path']),cv2.COLOR_BGR2RGB);raw=cv2.imread(a['depth_raw_path'],cv2.IMREAD_UNCHANGED)
        if raw.shape[2]==3:raw=cv2.cvtColor(raw,cv2.COLOR_BGR2BGRA)
        depth=carla_depth_to_metres(raw.tobytes(),raw.shape[1],raw.shape[0])
        detections=detector.infer(rgb,depth,mount,own[a['frame']],a['frame'],a['timestamp'])
        decision=classify_safety_hazards(detections,pedestrian_stop_forward_m=h.get('pedestrian_stop_forward_m',0))
        result.append(dict(frame=a['frame'],timestamp=a['timestamp'],own_xy=[own[a['frame']]['x'],own[a['frame']]['y']],decision=decision,detections=detections))
    save(run/'r10_saved_sensor_hazard_replay.json',dict(records=result,scope='CPU saved4Hz replay; not an exact reproduction of the20Hz live history; no online command written'))
    print(json.dumps(dict(frames=len(result),decisions=dict(collections.Counter(r['decision'].get('forced_stop_reason') for r in result)),last=result[-1]),ensure_ascii=False))


if __name__=='__main__':main()

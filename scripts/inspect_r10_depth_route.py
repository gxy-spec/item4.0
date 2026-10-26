"""One stored depth/own-pose probe, no simulator commands."""
from pathlib import Path
import sys,json,argparse,dataclasses,math
import numpy as np,cv2,yaml,carla
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,rows,save
from run_stage1_dynamic import camera_mount_matrix
from navigation.budgeted_approach import select_task_route
from navigation.ugv_oracle_planner import plan_route_from_message
from perception.body_clearance import rectangle
from safety.rgbd_obstacle_guard import RGBDObstacleGuard,ObstacleGuardConfig
from safety.route_corridor import local_route
from sensors.depth import carla_depth_to_metres


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',required=True,type=Path);args=p.parse_args();run=args.run_dir
    cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'));c=cfg['dynamic']['ugv_safety'];s=cfg['sensors'];spec=s['streams']['ugv_depth']
    a=[a for a in rows(run/'sensors/ugv/metadata/frames.jsonl') if a.get('sensor_files_saved')][-1]
    own=next(x for x in rows(run/'actors/actor_states.jsonl') if x['frame']==a['frame'] and x['role']=='ugv')
    poly=rectangle(own['bbox_location_xyz'][:2],own['bbox_extent_xyz'][:2],own['bbox_rotation_pyr'][1])
    m=carla.Map('Town10HD',(OUT/'Town10HD.xodr').read_text(encoding='utf-8'))
    target=json.loads((run/'logs/live_progress.json').read_text())['committed_target_reference'];start=carla.Location(own['x'],own['y'],own['z'])
    base=plan_route_from_message(m,start,target,standoff_m=6.)
    chosen,audit=select_task_route(m,start,target,poly,base,1000.,2.,[],a['timestamp'])
    # Probe the observation region too, regardless of the fixed-input budget.
    from navigation.approach_region import plan_observation_region
    regional=plan_observation_region(m,start,target,poly)
    kw={k:c[k] for k in {f.name for f in dataclasses.fields(ObstacleGuardConfig)} if k in c}
    kw.update(horizontal_fov_degrees=s['fov_degrees'],corridor_half_width_m=c.get('sensor_corridor_half_width_m',1.6),stop_distance_m=c.get('sensor_stop_distance_m',4.))
    mount=camera_mount_matrix(spec['location_xyz_m'],spec['rotation_pyr_degrees']);raw=cv2.imread(a['depth_raw_path'],cv2.IMREAD_UNCHANGED)
    if raw.shape[2]==3:raw=cv2.cvtColor(raw,cv2.COLOR_BGR2BGRA)
    depth=carla_depth_to_metres(raw.tobytes(),raw.shape[1],raw.shape[0]);result=[]
    for name,route in [('compatibility',base),('observation_region',regional),('straight',None)]:
        guard=RGBDObstacleGuard(ObstacleGuardConfig(**kw));local=None if route is None else local_route(route,own)
        r=guard.observe(depth,a['frame'],a['timestamp'],a['timestamp'],mount,route_vehicle_xy=local)
        pixels=guard.last_obstacle_pixels_uv.astype(int)
        fx=depth.shape[1]/(2*math.tan(math.radians(s['fov_degrees'])/2))
        support=[]
        segmentation=cv2.imread(a['instance_segmentation_path'],cv2.IMREAD_UNCHANGED)
        for u,v in pixels[:30]:
            d=float(depth[v,u]);point=(mount@np.array([d,(u-(depth.shape[1]-1)/2)*d/fx,-(v-(depth.shape[0]-1)/2)*d/fx,1]))[:3]
            support.append(dict(u=int(u),v=int(v),depth_m=d,vehicle_xyz=point.tolist(),offline_instance_bgr=segmentation[v,u,:3].tolist()))
        result.append(dict(route=name,result=r,support_points=support,local_xy=None if local is None else local.tolist()))
    save(run/'r10_saved_depth_route_probe.json',dict(frame=a['frame'],own_xy=[own['x'],own['y']],records=result,
        scope='Recomputed map routes and one saved depth frame, not the actual full live route/guard state. No outcome changed.'))
    print(json.dumps(dict(frame=a['frame'],own_xy=[own['x'],own['y']],records=result)))


if __name__=='__main__':main()

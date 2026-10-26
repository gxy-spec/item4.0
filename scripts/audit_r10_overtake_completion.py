"""Physical return-to-route audit, independent of the word 'cruise'."""
from pathlib import Path
import json,sys,csv,math,argparse
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows,digest
from run_stage1_dynamic import route_lateral_offset_m,bounded_nearest_route_index


def audit(run):
    report=json.loads((run/'validation_report.json').read_text(encoding='utf-8'))
    route=json.loads((run/'planning/ugv_patrol_route_executed.json').read_text(encoding='utf-8'))['points_xyz']
    safety=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
    passing=[int(r['frame']) for r in safety if r['ugv_safety_mode']=='overtake_lane_change']
    frames={}
    for a in rows(run/'actors/actor_states.jsonl'):frames.setdefault(a['frame'],{})[a['role']]=a
    samples=[];hold=[];accepted=[]
    for f,a in sorted(frames.items()):
        if not passing or f<=max(passing) or 'ugv' not in a or 'safety_test_obstacle' not in a:continue
        own=a['ugv'];obstacle=a['safety_test_obstacle'];xy=np.array([own['x'],own['y']])
        i=min(range(len(route)),key=lambda i:np.linalg.norm(xy-np.asarray(route[i][:2])))
        b=np.asarray(route[min(len(route)-1,i+1)][:2])-np.asarray(route[max(0,i-1)][:2]);b=b/max(1e-9,np.linalg.norm(b))
        yaw=math.degrees(math.atan2(b[1],b[0]));yaw_error=abs((own['yaw']-yaw+180)%360-180)
        pa=own.get('bbox_world_vertices_xyz');pb=obstacle.get('bbox_world_vertices_xyz')
        separation=float(np.min(np.array(pa)[:,:2]@b)-np.max(np.array(pb)[:,:2]@b)) if pa and pb else None
        offset=abs(route_lateral_offset_m(route,[own['x'],own['y']]))
        drivable=own.get('map_drivable_footprint') is True
        valid=bool(drivable and offset<=.75 and yaw_error<=15 and separation is not None and separation>=3 and own['speed_mps']<=.3)
        sample=dict(frame=f,timestamp_s=own['timestamp'],lateral_offset_m=offset,heading_error_deg=yaw_error,
            body_separation_after_pass_m=separation,drivable=drivable,speed_mps=own['speed_mps'],valid=valid)
        samples.append(sample)
        if not valid:hold=[];continue
        if hold and own['timestamp']-hold[-1]['timestamp_s']>.075:hold=[]
        hold.append(sample)
        if hold[-1]['timestamp_s']-hold[0]['timestamp_s']>=2.-1e-6:
            accepted=hold;break
    collisions=report['summary']['ugv_collision_count']+report['summary']['uav_collision_count']
    result=dict(passed=bool(len(passing)>=50 and accepted and not collisions),run_dir=str(run),
        criteria=dict(minimum_overtake_ticks=50,maximum_route_offset_m=.75,maximum_heading_error_deg=15,
            minimum_body_separation_beyond_obstacle_m=3.,maximum_hold_speed_mps=.3,minimum_continuous_hold_s=2.,
            drivable_footprint_required=True,episode_collisions=0),
        overtake_ticks=len(passing),collisions=collisions,hold_samples=accepted,
        source_sha256=digest(run/'actors/actor_states.jsonl'),raw_runner_status=report['status'],
        scope='Offline physical completion; only resolves cruise-label false rejection. It never makes collision/unfinished-passing runs pass.')
    save(run/'r10_overtake_physical_audit.json',result);return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path);args=p.parse_args()
    if args.run_dir:runs=[args.run_dir]
    else:
        runs=[]
        for name in ['safety_development_progress.json','safety_development_corridor2_progress.json','safety_development_validation2_progress.json','safety_development_validation3_progress.json']:
            path=OUT/name
            if path.exists():runs.extend(Path(a['run_dir']) for a in json.loads(path.read_text(encoding='utf-8')).values() if a['case']=='overtake')
    result=[]
    for run in dict.fromkeys(runs):
        r=audit(run);print(json.dumps({k:r[k] for k in ['passed','run_dir','overtake_ticks','collisions','raw_runner_status']},ensure_ascii=False));result.append(r)
    save(OUT/'analysis/overtake_physical_audit.json',result)


if __name__=='__main__':main()

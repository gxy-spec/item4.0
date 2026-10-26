"""A-control paired trajectory audit; tolerances declared before final N3 capture."""
from pathlib import Path
import json,sys,csv,collections
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,rows,save,digest

LIMITS=dict(initial_position_m=.02,initial_yaw_deg=.1,ugv_max_xy_delta_m=.10,
            background_max_xy_delta_m=.50)


def tracks(run):
    result=collections.defaultdict(list)
    for a in rows(run/'actors/actor_states.jsonl'):result[a['role']].append(a)
    for values in result.values():values.sort(key=lambda r:r['frame'])
    return result


def compare(a,b):
    result=[];errors=[]
    for role in sorted(set(a)|set(b)):
        x=a.get(role,[]);y=b.get(role,[])
        if len(x)!=len(y) or not x:
            errors.append(dict(role=role,error='length_or_role_mismatch',counts=[len(x),len(y)]));continue
        gap=np.linalg.norm(np.array([[r['x'],r['y']] for r in x])-np.array([[r['x'],r['y']] for r in y]),axis=1)
        dt=np.abs((np.array([r['timestamp'] for r in x])-x[0]['timestamp'])-(np.array([r['timestamp'] for r in y])-y[0]['timestamp']))
        yaw=abs((x[0]['yaw']-y[0]['yaw']+180)%360-180)
        limit=LIMITS['ugv_max_xy_delta_m'] if role=='ugv' else LIMITS['background_max_xy_delta_m']
        ok=bool(gap[0]<=LIMITS['initial_position_m'] and yaw<=LIMITS['initial_yaw_deg'] and max(gap)<=limit and max(dt)<=1e-5)
        result.append(dict(role=role,samples=len(x),initial_xy_delta_m=float(gap[0]),initial_yaw_delta_deg=yaw,
            maximum_xy_delta_m=float(max(gap)),p95_xy_delta_m=float(np.percentile(gap,95)),mean_xy_delta_m=float(np.mean(gap)),
            relative_tick_time_max_delta_s=float(max(dt)),within_engineering_tolerance=ok))
        if not ok:errors.append(result[-1])
    return result,errors


def main():
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'));rev=scope['active_validation_revision']
    p=OUT/f'diagnostics_development_{rev}_progress.json'
    if not p.exists():raise RuntimeError('Final N3 pair not captured')
    pair={a['variant']:Path(a['run_dir']) for a in json.loads(p.read_text(encoding='utf-8')).values() if a['case']=='N3' and a['group']=='A' and a.get('report_complete')}
    if set(pair)!= {'legacy','upgraded'}:raise RuntimeError('Both final N3-A runs required')
    a,b=pair['legacy'],pair['upgraded'];results,errors=compare(tracks(a),tracks(b));plans={}
    phase={}
    for variant,run in pair.items():
        plans[variant]=len([e for e in rows(run/'planning/target_route_events.jsonl') if 'planned' in e['event']])
        own=tracks(run)['ugv'];first=own[0]['frame']
        hazard=list(csv.DictReader((run/'safety/ugv_rgbd_hazard_detections.csv').open(encoding='utf-8-sig')))
        phase[variant]=sorted(set(int(r['frame'])-first for r in hazard))
    if any(plans.values()):errors.append(dict(error='A-control must not respond with a target route',plan_counts=plans))
    phase_same=phase['legacy']==phase['upgraded']
    if not phase_same:errors.append(dict(error='safety_sampling_phase_mismatch'))
    save(OUT/'analysis/N3_A_pair_audit.json',dict(passed=not errors,revision=rev,tolerances=LIMITS,
        tolerance_scope='Engineering reproducibility check, not a statistical significance standard',
        records=results,errors=errors,runs={k:str(v) for k,v in pair.items()},target_plan_counts=plans,
        exact_relative_hazard_sampling_frames_equal=phase_same,
        source_hashes={k:digest(v/'actors/actor_states.jsonl') for k,v in pair.items()},
        scope='One paired control run only; same seed alone is not proof of identical process'))
    print(json.dumps(dict(passed=not errors,records=results,errors=errors),ensure_ascii=False))


if __name__=='__main__':main()

"""Independent R10 business audit; preserves historical distance metrics.

This file runs after acquisition. Body truth and semantic instance images never
feed back into the robot. A historical re-evaluation is explicitly incomplete
if the original capture omitted true body transforms or legal-stop evidence.
"""
from pathlib import Path
import json,sys,math,argparse
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from mainline_paths import OUT,save,rows
from analyze_recovery_campaign import episode,table
from src.perception.body_clearance import polygon_clearance,points_to_polygon_distance
from src.perception.strict_completion import verify_completion


def polygon(actor):
    v=actor.get('bbox_world_vertices_xyz')
    return None if not v else cv2.convexHull(np.ascontiguousarray(np.array(v,np.float32)[:,:2])).reshape(-1,2).tolist()


def assess(item):
    run=Path(item['run_dir'])
    if not run.resolve().is_relative_to(OUT.resolve()):
        raise RuntimeError('Mainline evaluation is restricted to its configured output root')
    base=episode({**item,'runner_wall_seconds':item.get('wall_seconds')})
    if base.get('semantic_verdict')=='NOT_EVALUABLE':
        save(run/'r10_evaluation.json',base);return base
    audit=rows(run/'task1_candidate_audit.jsonl')
    evidence={(a['device'],a['frame'],a['candidate_id']):a for a in audit}
    actorframes={}
    for a in rows(run/'actors/actor_states.jsonl'):actorframes.setdefault(a['frame'],{})[a['role']]=a
    ownstates=sorted((a['ugv'] for a in actorframes.values() if 'ugv' in a),key=lambda a:a['timestamp'])
    origin=ownstates[0]['timestamp'];targets={f:actors.get('target') for f,actors in actorframes.items()}
    monitor_path=run/'ground_truth/evaluation_only/sensor_business_monitor.json'
    monitor=json.loads(monitor_path.read_text(encoding='utf-8')) if monitor_path.exists() else {}
    stage_results={};claim_counts={};stage_claims=[]
    for device in ['uav','ugv']:
        history=[];claims=[];correct=[]
        for frame in rows(run/f'perception/{device}_target_candidates_synchronized.jsonl'):
            candidates=[c for c in frame.get('candidates',[]) if c.get('target_color_match') and c.get('category_match_score',0)>=.5
                        and c.get('temporal_confirmed') and min(c['bbox_xyxy'][2]-c['bbox_xyxy'][0],c['bbox_xyxy'][3]-c['bbox_xyxy'][1])>=16]
            c=max(candidates,key=lambda c:c.get('candidate_score',0),default=None)
            a=evidence.get((device,frame['frame_id'],c['candidate_id'])) if c else None
            history.append((frame['timestamp_s'],a));history=history[-5:]
            if c:
                claims.append(a)
                matches=[v for t,v in history if v and frame['timestamp_s']-t<=1. and v['semantic_correct']
                         and a and v.get('matched_actor_id')==a.get('matched_actor_id')]
                ok=bool(a and a['semantic_correct'] and len(matches)>=3
                        and a.get('point_outside_actor_footprint_m') is not None and a['point_outside_actor_footprint_m']<=1)
                stage_claims.append(dict(device=device,frame=frame['frame_id'],timestamp_s=frame['timestamp_s']-origin,
                    correct=ok,candidate_id=c['candidate_id']))
                if ok:correct.append(a)
        stage_results[device]=bool(correct)
        claim_counts[device]=dict(claim_frames=len(claims),correct_discovery_frames=len(correct),
                                 first_correct_s=min((a['timestamp']-origin for a in correct),default=None))
    local=[]
    for obs in monitor.get('observations',[]):
        key=('ugv',obs.get('frame_id'),obs['candidate_id']);a=evidence.get(key)
        local.append(dict(**obs,identity_correct=bool(a and a.get('semantic_correct')),
                          matched_actor_id=None if a is None else a.get('matched_actor_id')))
    completions=[]
    for event in monitor.get('events',[]):
        if event.get('event')!='sensor_safe_approach_hold_completed':continue
        start=event['hold_start_s'];end=event['timestamp_s']
        interval=[s for s in ownstates if start-1e-6<=s['timestamp']-origin<=end+1e-6]
        measurements=[];missing=False
        for own in interval:
            target=targets.get(own['frame']);pa=polygon(own);pb=polygon(target) if target else None
            if pa is None or pb is None:missing=True;continue
            measurements.append(dict(frame=own['frame'],timestamp_s=own['timestamp']-origin,
                true_body_clearance_m=polygon_clearance(pa,pb),reference_distance_m=math.hypot(own['x']-target['x'],own['y']-target['y']),
                speed_mps=own['speed_mps'],legal_stop=own.get('map_drivable_footprint',own.get('legal_stop_footprint')),
                stopped_confirmation_in_junction=own.get('stopped_confirmation_in_junction')))
        local_window=[a for a in local if start-1e-6<=a['timestamp_s']<=end+1e-6]
        identity=bool(len(local_window)>=6 and all(a['identity_correct'] for a in local_window))
        timing=bool(len(interval)>=39 and interval[-1]['timestamp']-interval[0]['timestamp']>=1.9
                    and max(np.diff([s['timestamp'] for s in interval]),default=0)<=.075)
        distance_ok=bool(measurements and not missing and all(3<=s['true_body_clearance_m']<=8 for s in measurements))
        speed_ok=bool(measurements and all(s['speed_mps']<=.3 for s in measurements))
        legal_ok=bool(measurements and all(s['legal_stop'] is True for s in measurements))
        missing_legal=bool(not measurements or any(s['legal_stop'] is None for s in measurements))
        expected_id=next(iter(targets.values()))['actor_id'] if targets else None
        strict=verify_completion(event,ownstates,targets,local,origin_s=origin,expected_actor_id=expected_id,
            episode_collisions=base.get('collisions'),s1_pass=any(stage_results.values()),
            s2_pass=any(a['identity_correct'] and a.get('matched_actor_id')==expected_id for a in local),
            expected_present=base.get('expected_target_present'))
        completions.append(dict(event=event,correct_identity=identity,continuous_hold=timing,body_clearance_pass=distance_ok,
            speed_pass=speed_ok,legal_stop_pass=legal_ok,missing_body_geometry=missing,
            missing_legal_stop_evidence=missing_legal,
            drivable_footprint_pass=legal_ok,
            stopped_confirmation_in_junction=any(s.get('stopped_confirmation_in_junction') is True for s in measurements) if measurements and all(s.get('stopped_confirmation_in_junction') is not None for s in measurements) else None,
            passed=strict['passed'],strict_verification=strict,
            measurements=measurements,local_evidence=local_window))
    last=ownstates[-1];target=targets[last['frame']]
    final_gap=polygon_clearance(polygon(last),polygon(target)) if polygon(last) is not None and target and polygon(target) is not None else None
    s3=any(c['passed'] for c in completions)
    s2=any(a['identity_correct'] for a in local)
    found=any(stage_results.values())
    reasons=[]
    if base.get('collisions'):reasons.append('collision')
    if not found:reasons.append('no_verified_reliable_discovery')
    if not s2:reasons.append('no_verified_near_ground_confirmation')
    if not completions:reasons.append('no_online_body_clearance_completion_claim')
    elif not s3:reasons.append('completion_claim_failed_independent_audit')
    r=dict(**base)
    r.update(evaluation_version='L11-strict-independent-completion-v1',S1_correct_discovery=found,S1_by_device=stage_results,S1_claim_counts=claim_counts,
        S2_correct_ground_confirmation=s2,S3_found_and_safely_approached=s3,
        final_true_body_clearance_m=final_gap,completion_audits=completions,local_stage_observations=local,
        historical_reference_3_7_success=base.get('task_success',False),
        task_success=s3,reasons=reasons,
        final_scope='Full old/new same-version comparison; training/audit data remain controlled same-map. Truth used here after acquisition only.')
    save(run/'r10_evaluation.json',r);table(run/'r10_discovery_claims.csv',stage_claims)
    return r


def main():
    p=argparse.ArgumentParser();p.add_argument('--result',type=Path);p.add_argument('--cohort');args=p.parse_args()
    if args.result:
        item=json.loads(args.result.read_text(encoding='utf-8'));result=assess(item)
        print(json.dumps({k:result.get(k) for k in ['case','S1_correct_discovery','S2_correct_ground_confirmation','S3_found_and_safely_approached','final_true_body_clearance_m','collisions','reasons']},ensure_ascii=False))
    else:
        progress=json.loads((OUT/f'{args.cohort}_progress.json').read_text(encoding='utf-8'))
        results=[assess(item) for item in progress.values() if item.get('report_complete')]
        save(OUT/f'analysis/{args.cohort}_results.json',results)


if __name__=='__main__':main()

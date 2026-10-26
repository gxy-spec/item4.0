"""Evaluate all completed task-1 episodes, retaining missing and failed cases."""
from __future__ import annotations
import argparse
from collections import Counter,defaultdict
import csv
import json
import math
from pathlib import Path
import sys
import shutil
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from perception_online_acceptance import OUT,read_jsonl,save_json,ground_truth,metrics
from export_instance_segmentation_dataset import load_actor_states
from src.perception.candidate_pipeline import CandidatePipeline


def quantiles(values):
    values=[float(x) for x in values if x is not None and math.isfinite(float(x))]
    return {'n':len(values),'median':float(np.median(values)) if values else None,
        'p95':float(np.percentile(values,95)) if values else None,'maximum':max(values) if values else None}


def table(path,rows):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)


def csv_rows(path):
    return list(csv.DictReader(Path(path).open(encoding='utf-8-sig',newline=''))) if Path(path).exists() else []


def classify_commit(evidence):
    """Separate a proven wrong object from an unresolved instance association."""
    if not evidence or evidence.get('matched_actor_id') is None:return 'unverified'
    if not evidence.get('semantic_correct'):return 'wrong_target'
    error=evidence.get('localization_error_m')
    if error is None:return 'unverified'
    return 'accepted' if error<=5 else 'position_rejected'


def split_commit_verdicts(result):
    for c in result.get('commits',[]):
        c['verification_status']=classify_commit(c.get('evidence'))
        c['correct']=c['verification_status']=='accepted'
    counts=Counter(c['verification_status'] for c in result.get('commits',[]))
    result['wrong_commits']=counts['wrong_target']
    result['unverified_commits']=counts['unverified']
    result['position_rejected_commits']=counts['position_rejected']
    result['commit_verification_failures']=sum(counts[k] for k in ['wrong_target','unverified','position_rejected'])
    reasons=[s for s in result.get('reasons',[]) if s not in ['wrong_target_commit','unverified_target_commit','committed_position_error']]
    for key,label in [('wrong_target','wrong_target_commit'),('unverified','unverified_target_commit'),('position_rejected','committed_position_error')]:
        if counts[key]:reasons.insert(0,label)
    result['reasons']=reasons;result['evaluation_version']=4
    return result


def episode(item):
    if not item.get('run_dir') or not (Path(item['run_dir'])/'actors/actor_states.jsonl').exists():
        return {**item,'semantic_verdict':'NOT_EVALUABLE','task_success':False,'reason':'No complete runner report'}
    run=Path(item['run_dir']);cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
    report=json.loads((run/'validation_report.json').read_text(encoding='utf-8'))
    spec=cfg['online_acceptance'];category=spec['task_category'];color=spec['task_color'];present=spec['expected_target_present']
    actors=load_actor_states(run/'actors/actor_states.jsonl')
    metadata={device:{int(r['frame']):r for r in read_jsonl(run/f'sensors/{device}/metadata/frames.jsonl')}
        for device in ['uav','ugv']}
    cframes={device:read_jsonl(run/f'perception/{device}_target_candidates_synchronized.jsonl') for device in ['uav','ugv']}
    evidence={};frame_rows=[];local_observations=[];errors=defaultdict(list)
    actual_matching_actors=set();missing_files=0
    for device,frames in cframes.items():
        for row in frames:
            frame=int(row['frame_id']);m=metadata[device].get(frame)
            if not m or not m.get('instance_segmentation_path'):
                missing_files+=1;continue
            gt,order=ground_truth(m['instance_segmentation_path'],actors[frame])
            target_all=[g for g in gt if g['category']==category and g['color']==color and g.get('role') not in {'ugv','uav'}]
            target_visible=[g for g in target_all if g['pixels']>=80]
            small=bool(target_all) and not target_visible
            actual_matching_actors.update(g['actor_id'] for g in target_all)
            options=[]
            for c in row.get('candidates',[]):
                nearest=max(gt,key=lambda g:CandidatePipeline._iou(c['bbox_xyxy'],g['bbox']),default=None)
                overlap=0 if nearest is None else CandidatePipeline._iou(c['bbox_xyxy'],nearest['bbox'])
                semantic_ok=bool(nearest is not None and overlap>=.5 and nearest['category']==category and nearest['color']==color and nearest.get('role') not in {'ugv','uav'})
                xyz=c.get('world_position_xyz')
                loc=math.dist(xyz[:2],nearest['xyz'][:2]) if semantic_ok and xyz is not None else None
                eligible=c.get('category')==category and bool(c.get('target_color_match'))
                info={'device':device,'frame':frame,'timestamp':row['timestamp_s'],'candidate_id':c['candidate_id'],
                    'bbox':c['bbox_xyxy'],'image':m['rgb_path'],'category':c.get('category'),'color':c.get('color'),
                    'semantic_correct':semantic_ok,'iou':overlap,'matched_actor_id':nearest['actor_id'] if overlap>=.5 else None,
                    'matched_category':nearest['category'] if overlap>=.5 else None,
                    'matched_color':nearest['color'] if overlap>=.5 else None,
                    'matched_role':nearest.get('role') if overlap>=.5 else None,
                    'detector_score':c.get('detector_score'),
                    'localization_error_m':loc,'eligible':eligible,'temporal_confirmed':c.get('temporal_confirmed',False),
                    'world_xyz':xyz,'candidate_score':c.get('candidate_score',0),
                    'correct_visible':semantic_ok and nearest['pixels']>=80 if nearest else False}
                evidence[(device,frame,c['candidate_id'])]=info
                if eligible:options.append(info)
                if eligible and semantic_ok and loc is not None:errors[device].append(loc)
                if device=='ugv' and eligible and c.get('temporal_confirmed') and xyz is not None:
                    local_observations.append(info)
            chosen=max(options,key=lambda c:c['candidate_score'],default=None)
            frame_rows.append({'device':device,'frame':frame,'timestamp':row['timestamp_s'],'positive':bool(target_visible),
                'negative':not target_all,'ignored_small':small,'selected':chosen is not None,
                'selected_correct':bool(chosen and chosen['correct_visible']),
                'detector_wall_ms':row.get('detector_wall_ms'),'pipeline_wall_ms':row.get('candidate_pipeline_wall_ms')})
    # Message identity and localization are checked against the exact source
    # frame and candidate, not merely against the closest current target.
    messages=read_jsonl(run/'communication/messages.jsonl')
    message_map={m['message_id']:m for m in messages}
    memory=read_jsonl(run/'communication/target_memory_events.jsonl')
    commits=[m for m in memory if m.get('event') in {'memory_initial_target','memory_confirmed_target_switch'}]
    commit_results=[]
    for event in commits:
        message=message_map.get(event.get('message_id'),{})
        candidate=evidence.get((message.get('source'),int(message.get('source_frame',-1)),message.get('candidate_id')))
        correct=bool(candidate and candidate['semantic_correct'] and candidate['localization_error_m'] is not None and candidate['localization_error_m']<=5)
        commit_results.append({'event':event,'message':message,'evidence':candidate,'correct':correct})
    wrong_commits=sum(not c['correct'] for c in commit_results)
    # Reconstruct local confirmation and arrival exclusively from logged
    # online-available observations/own pose; actor truth is a second audit.
    own_states=[a for values in actors.values() for a in values if a.get('role')=='ugv']
    own_states.sort(key=lambda s:s['frame'])
    byframe={s['frame']:{a['actor_id']:a for a in actors[s['frame']]} for s in own_states}
    local_observations.sort(key=lambda o:o['timestamp'])
    commit_results.sort(key=lambda c:c['event']['timestamp_s'])
    current=None;ci=0;li=0;confirmed=False;confirmed_correct=False;hold_start=None;arrival=None;max_hold=0.
    for own in own_states:
        now=own['timestamp']
        while ci<len(commit_results) and commit_results[ci]['event']['timestamp_s']<=now:
            current=commit_results[ci];confirmed=False;confirmed_correct=False;hold_start=None;ci+=1
        while li<len(local_observations) and local_observations[li]['timestamp']<=now:
            o=local_observations[li];li+=1
            if current and o['timestamp']>=current['event']['timestamp_s'] and math.dist(o['world_xyz'][:2],current['event']['target_xyz'][:2])<=5:
                confirmed=True;confirmed_correct=confirmed_correct or o['semantic_correct']
        if not current:continue
        perceived=math.dist([own['x'],own['y']],current['event']['target_xyz'][:2])
        stopped=float(own.get('speed_mps',float('inf')))<=.3
        reached=3<=perceived<=7 and stopped and confirmed
        hold_start=(now if hold_start is None else hold_start) if reached else None
        hold=0. if hold_start is None else now-hold_start+.05;max_hold=max(max_hold,hold)
        if hold>=2 and arrival is None:
            match_id=None if current.get('evidence') is None else current['evidence']['matched_actor_id']
            truth=byframe[own['frame']].get(match_id)
            actual=None if truth is None else math.dist([own['x'],own['y']],[truth['x'],truth['y']])
            arrival={'timestamp':now,'perceived_distance_m':perceived,'actual_distance_m':actual,
                'committed_correct':current['correct'],'local_confirmation_correct':confirmed_correct,
                'validated':bool(current['correct'] and confirmed_correct and actual is not None and 3<=actual<=7)}
    summary=report['summary'];collisions=summary.get('ugv_collision_count',0)+summary.get('uav_collision_count',0)
    def frame_metrics(rows):
        usable=[r for r in rows if not r['ignored_small']]
        pos=sum(r['positive'] for r in usable);neg=sum(r['negative'] for r in usable)
        sel=sum(r['selected'] for r in usable);correct=sum(r['selected_correct'] for r in usable)
        false=sum(r['negative'] and r['selected'] for r in usable)
        return {'evaluated_task_frames':len(usable),'positive_frames':pos,'negative_frames':neg,
            'selected':sel,'correct':correct,'negative_false_triggers':false,
            'precision':correct/sel if sel else None,'recall':correct/pos if pos else None,
            'negative_fpr':false/neg if neg else None}
    device_metrics={d:frame_metrics([r for r in frame_rows if r['device']==d]) for d in ['uav','ugv']}
    timing={d:{'detector_ms':quantiles([r['detector_wall_ms'] for r in frame_rows if r['device']==d][2:]),
        'pipeline_ms':quantiles([r['pipeline_wall_ms'] for r in frame_rows if r['device']==d][2:])} for d in ['uav','ugv']}
    combined=defaultdict(float)
    for r in frame_rows:combined[r['frame']]+=r['pipeline_wall_ms'] or 0.
    timing['combined_pipeline_ms']=quantiles(list(combined.values())[2:])
    first_sim=min((s['timestamp'] for s in own_states),default=0)
    task_success=bool(present and arrival and arrival['validated'] and wrong_commits==0 and collisions==0)
    negative_pass=bool(not present and len(commits)==0 and collisions==0)
    routes=read_jsonl(run/'planning/target_route_events.jsonl')
    safety=csv_rows(run/'safety/safety_state.csv');last=safety[-1] if safety else {}
    reason=[]
    if wrong_commits:reason.append('wrong_target_commit')
    if collisions:reason.append('collision')
    if present and not commits:reason.append('no_target_commit')
    if present and not confirmed:reason.append('no_local_confirmation')
    if present and not arrival:reason.append('no_valid_arrival_hold')
    if arrival and not arrival['validated']:reason.append('perceived_arrival_failed_truth_audit')
    if not present and commits:reason.append('false_commit_in_target_absent_scene')
    result={**item,'evaluation_version':3,'expected_target_present':present,'task_category':category,'task_color':color,
        'model_calibration_feasible':spec['calibration_feasible'],'runner_status':report['status'],
        'runner_failed_checks':[c for c in report['checks'] if not c['passed']],
        'semantic_verdict':'PASS' if (task_success or negative_pass) else 'FAIL',
        'task_success':task_success,'negative_rejection_pass':negative_pass,'reasons':reason,
        'frame_metrics':frame_metrics(frame_rows),'by_device':device_metrics,'localization_xy_m':{d:quantiles(v) for d,v in errors.items()},
        'online_latency':timing,'combined_pipeline_budget_ms':250,
        'actual_runner_real_time_factor':cfg['simulation']['duration_seconds']/item['runner_wall_seconds'] if item.get('runner_wall_seconds') else None,
        'commits':commit_results,'wrong_commits':wrong_commits,'sent_messages':len(messages),
        'message_count_by_device':dict(Counter(m['source'] for m in messages)),
        'route_plan_count':sum(r.get('event')=='target_route_planned_from_selected_message' for r in routes),
        'local_confirmation_observed':confirmed,'local_confirmation_correct':confirmed_correct,
        'arrival':arrival,'maximum_hold_seconds':max_hold,'collisions':collisions,
        'final_target_distance_m':summary.get('ugv_final_target_distance_m') if present else None,
        'final_control_mode':last.get('ugv_safety_mode'),'final_sensor_state':last.get('ugv_obstacle_sensor_state'),
        'safety_mode_counts':dict(Counter(r.get('ugv_safety_mode') for r in safety)),
        'missing_instance_frames':missing_files,'known_matching_actor_ids':sorted(actual_matching_actors),
        'confirmation_method':'post-run reconstruction using only logged local RGB-D candidates, delivered target memory and own pose; truth is a separate audit',
        'runtime_has_explicit_business_completion_state':False}
    result=split_commit_verdicts(result)
    save_json(run/'task1_evaluation.json',result)
    (run/'task1_candidate_audit.jsonl').write_text(''.join(json.dumps(e,ensure_ascii=False)+'\n' for e in evidence.values()),encoding='utf-8')
    table(run/'task1_frame_metrics.csv',frame_rows)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--smoke',action='store_true');a=p.parse_args()
    idx=OUT/('smoke_index.json' if a.smoke else 'online_index.json')
    data=json.loads(idx.read_text(encoding='utf-8')) if idx.exists() else {}
    results=[]
    for name,item in data.items():
        rp=Path(item.get('run_dir','_missing_'))/'task1_evaluation.json'
        cached=json.loads(rp.read_text(encoding='utf-8')) if rp.exists() else {}
        if cached.get('evaluation_version')==3:
            backup=rp.with_name('task1_evaluation_v3_before_commit_split.json')
            if not backup.exists():shutil.copy2(rp,backup)
            r=split_commit_verdicts(cached);save_json(rp,r)
        elif cached.get('evaluation_version')==4:r=cached
        else:r=episode(item)
        results.append(r);print(name,r.get('semantic_verdict'),r.get('reasons'),flush=True)
    name='smoke' if a.smoke else 'online'
    save_json(OUT/f'analysis/{name}_results.json',results)
    flattened=[]
    for r in results:
        fm=r.get('frame_metrics',{})
        flattened.append({'case':r['case'],'runner_status':r.get('runner_status'),'verdict':r.get('semantic_verdict'),
            'expected_target_present':r.get('expected_target_present'),'task_success':r.get('task_success'),
            'negative_rejection_pass':r.get('negative_rejection_pass'),'wrong_commits':r.get('wrong_commits'),
            'unverified_commits':r.get('unverified_commits'),'position_rejected_commits':r.get('position_rejected_commits'),
            'commit_verification_failures':r.get('commit_verification_failures'),
            'collisions':r.get('collisions'),'final_distance_m':r.get('final_target_distance_m'),
            'precision':fm.get('precision'),'recall':fm.get('recall'),'negative_fpr':fm.get('negative_fpr'),
            'messages':r.get('sent_messages'),'rtf':r.get('actual_runner_real_time_factor'),
            'reasons':';'.join(r.get('reasons',[])),'run_dir':r.get('run_dir')})
    table(OUT/f'analysis/{name}_summary.csv',flattened)
    if results:
        fig,axes=plt.subplots(1,2,figsize=(13,4.8));x=np.arange(len(results))
        axes[0].bar(x,[r.get('final_target_distance_m') if r.get('final_target_distance_m') is not None else np.nan for r in results],color='#307fac')
        for i,r in enumerate(results):
            if not r.get('expected_target_present'):axes[0].text(i,1,'absent',ha='center',va='bottom',rotation=90,fontsize=8)
        axes[0].axhspan(3,7,color='#79bd8f',alpha=.25,label='Arrival band (positive cases)');axes[0].set_ylabel('Final true target distance (m)');axes[0].legend()
        axes[1].bar(x,[r.get('wrong_commits',0) for r in results],color='#ca664e',label='Confirmed wrong object')
        axes[1].bar(x,[r.get('unverified_commits',0) for r in results],bottom=[r.get('wrong_commits',0) for r in results],color='#d9a441',label='Unverified association')
        axes[1].set_ylabel('Initial/switch target commits');axes[1].legend(fontsize=8)
        for ax in axes:ax.set_xticks(x,[r['case'].replace('_4210',' / ') for r in results],rotation=70,ha='right');ax.grid(axis='y',alpha=.2)
        fig.tight_layout();fig.savefig(OUT/f'analysis/{name}_task_results.png',dpi=180);plt.close(fig)
    print('Results retained:',OUT/'analysis',flush=True)

if __name__=='__main__':main()

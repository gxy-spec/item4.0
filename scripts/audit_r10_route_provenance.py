"""Post-run message-plan causality. Does not claim simulator-wide truth proof."""
from pathlib import Path
import sys,json,math,argparse
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows


def audit(run):
    messages={m['message_id']:m for m in rows(run/'communication/messages.jsonl')}
    selected=rows(run/'communication/selected_target_states.jsonl')
    selections={}
    for s in selected:
        if s.get('event')=='selected':selections.setdefault(s['message_id'],[]).append(s)
    candidates={}
    for d in ['uav','ugv']:
        for f in rows(run/f'perception/{d}_target_candidates_synchronized.jsonl'):
            for c in f.get('candidates',[]):candidates[(d,f['frame_id'],c['candidate_id'])]=c
    records=[];errors=[]
    for p in rows(run/'planning/target_route_events.jsonl'):
        if p.get('event') not in ['target_route_planned_from_selected_message','blocked_route_replanned']:continue
        m=messages.get(p.get('message_id'));c=None if not m else candidates.get((m['source'],m['source_frame'],m['candidate_id']))
        s=[a for a in selections.get(p.get('message_id'),[]) if a['timestamp_s']<=p['timestamp_s']+1e-6]
        position_error=None if not m or not c else math.dist(m['world_position_xyz'],c['world_position_xyz'])
        goal_error=None if not m else math.dist(m['world_position_xyz'],p['target_xyz'])
        passed=bool(m and c and s and m['source_timestamp_s']<=m['receive_timestamp_s']<=p['timestamp_s']+1e-6
                    and position_error<=1e-6 and goal_error<=1e-6)
        r=dict(event=p['event'],timestamp_s=p['timestamp_s'],message_id=p.get('message_id'),passed=passed,
            candidate_message_position_error_m=position_error,message_plan_position_error_m=goal_error,
            source_timestamp_s=None if not m else m['source_timestamp_s'],
            receive_timestamp_s=None if not m else m['receive_timestamp_s'])
        records.append(r)
        if not passed:errors.append(r)
    result=dict(passed=not errors,chains=len(records),records=records,errors=errors,
        scope='Logged sensor candidate -> sent message -> selected state -> plan causality only; not global simulator truth independence')
    save(run/'r10_route_provenance.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);a=p.parse_args();r=audit(a.run_dir)
    print(json.dumps(dict(passed=r['passed'],chains=r['chains'],errors=r['errors']),ensure_ascii=False))

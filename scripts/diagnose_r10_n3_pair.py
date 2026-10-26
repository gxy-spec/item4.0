"""Locate first actual paired divergence without changing tolerance or outcomes."""
from pathlib import Path
import sys,json,csv,collections,math
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows
from build_r10_targeted_report import DEST


def main():
    audit=json.loads((OUT/'analysis/N3_A_pair_audit.json').read_text(encoding='utf-8'))
    runs={k:Path(v) for k,v in audit['runs'].items()};tracks={};safety={};hazards={}
    for v,run in runs.items():
        grouped=collections.defaultdict(list)
        for a in rows(run/'actors/actor_states.jsonl'):grouped[a['role']].append(a)
        for items in grouped.values():items.sort(key=lambda a:a['frame'])
        tracks[v]=grouped
        safety[v]=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
        hazards[v]=list(csv.DictReader((run/'safety/ugv_rgbd_hazard_detections.csv').open(encoding='utf-8-sig')))
    first=safety['legacy'][0];origin=float(first['timestamp']);records=[]
    for role in tracks['legacy']:
        a,b=tracks['legacy'][role],tracks['upgraded'][role];thresholds={}
        for limit in [.001,.01,.1,.5,1.,10.]:
            index=next((i for i,(x,y) in enumerate(zip(a,b)) if math.hypot(x['x']-y['x'],x['y']-y['y'])>limit),None)
            thresholds[str(limit)]=None if index is None else dict(relative_s=a[index]['timestamp']-a[0]['timestamp'],sample=index,
                legacy_xy=[a[index]['x'],a[index]['y']],upgraded_xy=[b[index]['x'],b[index]['y']])
        records.append(dict(role=role,first_exceeds=thresholds))
    differences=[]
    for i,(a,b) in enumerate(zip(safety['legacy'],safety['upgraded'])):
        if a['ugv_safety_mode']!=b['ugv_safety_mode']:
            t=float(a['timestamp'])-origin
            nearby={}
            for v in runs:
                start=float(safety[v][0]['timestamp'])
                nearby[v]=[h for h in hazards[v] if t-.6<=float(h['timestamp'])-start<=t+.1]
            differences.append(dict(relative_s=t,sample=i,legacy=a,upgraded=b,nearby_hazards=nearby))
            if len(differences)>=5:break
    counts={v:dict(collections.Counter(r['ugv_safety_mode'] for r in safety[v])) for v in runs}
    result=dict(pair_passed=audit['passed'],first_role_divergences=records,first_safety_mode_differences=differences,
        safety_mode_ticks=counts,paired_audit=str(OUT/'analysis/N3_A_pair_audit.json'),
        scope='Temporal diagnosis; correlation does not by itself prove a simulator or perception root cause. Original tolerances unchanged.')
    save(OUT/'analysis/N3_A_pair_divergence_diagnosis.json',result)
    print(json.dumps(dict(first_role_divergences=records,safety_mode_ticks=counts,
        first_safety_mode_differences=[{k:r[k] for k in ['relative_s','sample','legacy','upgraded']} for r in differences]),ensure_ascii=False))


if __name__=='__main__':main()

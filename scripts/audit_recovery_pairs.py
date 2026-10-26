"""Actual initial poses and A-control trajectories, beyond config equality."""
import sys,json,math,csv
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def states(run,trace=False):
    first={};tracks={k:[] for k in ['ugv','uav','target']};first_frame=None
    with (run/'actors/actor_states.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r=json.loads(line)
            if first_frame is None:first_frame=r['frame']
            if r['frame']==first_frame:first[r['role']]=r
            elif not trace:break
            if trace and r['role'] in tracks:tracks[r['role']].append(r)
    return first,tracks


def main():
    progress=json.loads((OUT/'formal_progress.json').read_text(encoding='utf-8'));pairs={};records=[]
    for key,r in progress.items():
        if r.get('report_complete'):pairs.setdefault((r['case'],r['group']),{})[r['variant']]=r
    for (case,group),p in pairs.items():
        if set(p)!={'legacy','upgraded'}:continue
        paths=[Path(p[v]['run_dir']) for v in ['legacy','upgraded']]
        (a,at),(b,bt)=[states(r,group=='A') for r in paths];record=dict(case=case,group=group,scene=p['legacy']['scene'])
        for role in ['ugv','uav','target']:
            record[role+'_initial_xy_delta_m']=math.hypot(a[role]['x']-b[role]['x'],a[role]['y']-b[role]['y'])
            if group=='A':
                z=list(zip(at[role],bt[role]));record[role+'_A_maximum_aligned_xy_delta_m']=max(math.hypot(x['x']-y['x'],x['y']-y['y']) for x,y in z)
                record[role+'_A_paired_tick_count']=len(z);record[role+'_A_tick_counts_equal']=len(at[role])==len(bt[role])
        background=[r for r in a if r not in ['ugv','uav','target'] and r in b]
        record['background_maximum_initial_xy_delta_m']=max([math.hypot(a[r]['x']-b[r]['x'],a[r]['y']-b[r]['y']) for r in background],default=0.)
        record['initial_roles_equal']=set(a)==set(b);record['legacy_run']=str(paths[0]);record['upgraded_run']=str(paths[1]);records.append(record)
    DELIVERY.mkdir(parents=True,exist_ok=True)
    if records:
        keys=list(dict.fromkeys(k for r in records for k in r))
        with (DELIVERY/'配对初始状态与A组轨迹核查.csv').open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(records)
    save(DELIVERY/'配对实际状态核查.json',dict(completed_pairs=len(records),records=records,
        scope='Read-only actual-state comparison. Same seed/config does not imply exact dynamic reproducibility. A is non-responsive and should remain close; B/C post-message trajectories intentionally may differ. Completed outcomes are never filtered using this audit.'))
    print('PAIRS AUDITED',len(records),flush=True)


if __name__=='__main__':main()

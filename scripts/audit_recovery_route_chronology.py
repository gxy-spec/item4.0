"""Read-only trace from sensor candidates to messages, plans and replay frames."""
import json,csv,sys,math,hashlib
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,DELIVERY

def rows(p):
    if not p.exists():return []
    return [json.loads(s) for s in p.read_text(encoding='utf-8').splitlines() if s.strip()]

def green_count(im):
    # Map only, excluding RGB, legends, and the green pedestrian marker hue.
    a=im[185:895,90:915].astype(np.int16);b,g,r=cv2.split(a)
    mask=(b>=65)&(b<=145)&(r>=65)&(r<=145)&(g>=145)&(g<=215)&(g-b>=30)&(g-r>=30)&(abs(b-r)<=22)
    return int(mask.sum())

def main():
    records=[];failures=[]
    for cohort in ['formal','acceptance_r9']:
        progress=json.loads((OUT/f'{cohort}_progress.json').read_text(encoding='utf-8'))
        for key,p in progress.items():
            run=Path(p['run_dir']);plans=[r for r in rows(run/'planning/target_route_events.jsonl') if r.get('event')=='target_route_planned_from_selected_message']
            messages={m['message_id']:m for m in rows(run/'communication/messages.jsonl')}
            selections=rows(run/'communication/selected_target_states.jsonl')
            with (run/'synchronization/frame_index.csv').open(encoding='utf-8-sig') as f:frames=[r for r in csv.DictReader(f) if int(r.get('sensor_files_saved','1'))==1]
            first=float(frames[0]['timestamp']);times=np.array([float(r['timestamp']) for r in frames]);chains=[]
            needed={d:set() for d in ['uav','ugv']}
            for e in plans:
                m=messages.get(e['message_id'])
                if m:needed[m['source']].add((int(m['source_frame']),m['candidate_id']))
            candidates={}
            for device,need in needed.items():
                if not need:continue
                with (run/f'perception/{device}_target_candidates_synchronized.jsonl').open(encoding='utf-8') as f:
                    for line in f:
                        r=json.loads(line)
                        for c in r.get('candidates',[]):
                            if (int(r['frame_id']),c['candidate_id']) in need:candidates[(device,int(r['frame_id']),c['candidate_id'])]=c
                        if sum(k[0]==device for k in candidates)==len(need):break
            for e in plans:
                m=messages.get(e['message_id']);selected=next((s for s in selections if s.get('message_id')==e['message_id'] and s.get('timestamp_s',float('inf'))<=e['timestamp_s']+1e-6),None)
                c=candidates.get((m['source'],int(m['source_frame']),m['candidate_id'])) if m else None
                ok=bool(m and selected and c and c.get('temporal_confirmed') and m['source_timestamp_s']<=m['sent_timestamp_s']+1e-6 and m['sent_timestamp_s']<=selected['timestamp_s']+1e-6 and selected['timestamp_s']<=e['timestamp_s']+1e-6)
                dm=math.dist(e['target_xyz'],m['world_position_xyz']) if m else None
                dc=math.dist(m['world_position_xyz'],c['world_position_xyz']) if m and c else None
                ok=ok and dm is not None and dm<=1e-8 and dc is not None and dc<=1e-8
                chains.append(dict(message_id=e['message_id'],source=m['source'] if m else None,source_frame=m.get('source_frame') if m else None,candidate_id=m.get('candidate_id') if m else None,
                    source_elapsed_s=m['source_timestamp_s']-first if m else None,selected_elapsed_s=selected['timestamp_s']-first if selected else None,plan_elapsed_s=e['timestamp_s']-first,
                    plan_message_xyz_delta_m=dm,message_candidate_xyz_delta_m=dc,chain_valid=ok,bbox_xyxy=c.get('bbox_xyxy') if c else None,run_dir=str(run)))
                if not ok:failures.append((key,e['message_id']))
            cap=cv2.VideoCapture(str(run/'preview/recovery_five_view.mp4'));sample_indices={0}
            if plans:
                pre=np.flatnonzero(times<plans[0]['timestamp_s']-1e-6)
                if len(pre):sample_indices.update([int(pre[len(pre)//2]),int(pre[-1])])
                after=np.flatnonzero(times>=plans[0]['timestamp_s']-1e-6)
                if len(after):sample_indices.add(int(after[0]))
            samples=[]
            for i in sorted(sample_indices):
                cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,im=cap.read()
                if not ok:raise RuntimeError('Cannot decode '+key)
                n=green_count(im);preplan=not plans or times[i]<plans[0]['timestamp_s']-1e-6
                samples.append(dict(video_frame=i,source_frame=int(frames[i]['frame']),elapsed_s=float(times[i]-first),before_first_plan=bool(preplan),green_route_colored_pixels=n))
            cap.release()
            records.append(dict(cohort=cohort,case=key,run_dir=str(run),plan_count=len(plans),first_plan_elapsed_s=plans[0]['timestamp_s']-first if plans else None,chains=chains,video_samples=samples))
            print('AUDIT',cohort,key,len(plans),'pre-green',[s['green_route_colored_pixels'] for s in samples if s['before_first_plan']],flush=True)
    dest=DELIVERY/'目标消息与绿色规划时序核查';dest.mkdir(exist_ok=True)
    result=dict(runs=len(records),planned_events=sum(r['plan_count'] for r in records),chain_failures=failures,
        preplan_samples_with_green=[dict(case=r['case'],**s) for r in records for s in r['video_samples'] if s['before_first_plan'] and s['green_route_colored_pixels']>8],
        scope='Frozen logged source candidates/messages/selection/plan coordinate equality and sampled actual encoded replay frames. Pixel threshold is a visual diagnostic, not an instrumented simulator-wide truth-access audit.',records=records)
    (dest/'审计明细.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps({k:v for k,v in result.items() if k!='records'},ensure_ascii=False),flush=True)

if __name__=='__main__':main()

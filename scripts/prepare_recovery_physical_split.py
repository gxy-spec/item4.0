"""Group target deployments by actual coordinates; remove repeated control views."""
import sys,json,re,yaml
from pathlib import Path
from collections import Counter,defaultdict
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def main():
    dest=OUT/'attribute_data_v6';dest.mkdir(exist_ok=True)
    if (dest/'manifest.jsonl').exists():raise RuntimeError('Already frozen')
    assignment={'east_1':'test','east_2':'train','east_3':'train','north_1':'validation','north_2':'validation',
        'north_3':'train','west_1':'test','west_2':'test','west_3':'validation','whole_1':'train','whole_2':'train','whole_3':'validation'}
    physical=defaultdict(list);by_scene={}
    for p in (ROOT/'configs/experiments/multiscene_abc_formal_20261010_v2').glob('*_C_formal.yaml'):
        c=yaml.safe_load(p.read_text(encoding='utf-8'));key=tuple(round(float(x),2) for x in c['region']['target_transform']['location_xyz'][:2])
        scene=c['region']['region_id'].replace('town10hd_','');physical[key].append(scene);by_scene[scene]=key
    for key,scenes in physical.items():
        if len({assignment[s] for s in scenes})!=1:raise RuntimeError('Physical deployment leakage '+str(scenes))
    records=[];excluded=[];changed=[]
    for original in rows(OUT/'attribute_data_v5/manifest.jsonl'):
        scene=original['scene'].split('town10hd_')[-1];split=assignment[scene];source=original.get('source','')
        controlled=source.startswith('nearfield_') or source=='balanced_virtual_camera_capture'
        if controlled and scene in ['north_2','west_1']:
            excluded.append(dict(id=original['id'],reason='Repeated controlled physical view; keep canonical N1/W2 counterpart'));continue
        if source.startswith('nearfield_') and split!='train' and source.endswith('_gt'):
            excluded.append(dict(id=original['id'],reason='Evaluation uses one detector crop per nearfield frame'));continue
        r={**original,'split':split,'previous_split':original['split'],'physical_target_xy':by_scene[scene]}
        if r['split']!=r['previous_split']:changed.append(dict(id=r['id'],old_split=r['previous_split'],new_split=split,scene=scene))
        records.append(r)
    split_sets={s:{tuple(r['physical_target_xy']) for r in records if r['split']==s} for s in ['train','validation','test']}
    for a,b in [('train','validation'),('train','test'),('validation','test')]:
        if split_sets[a]&split_sets[b]:raise RuntimeError('Overlap')
    (dest/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    save(dest/'split_adjudication.json',dict(source_manifest_sha256=digest(OUT/'attribute_data_v5/manifest.jsonl'),
        assignments=assignment,physical_groups={str(k):v for k,v in physical.items()},removed=excluded,reassigned=changed,
        scope='Disjoint target-deployment coordinates for the ATTRIBUTE HEAD. Same map, vehicle assets and camera conventions; city-wide camera backgrounds can overlap. The retained YOLOE detector used its original split and can have seen some new head-test regions; this is not an unseen-map/system-wide generalization claim.'))
    save(dest/'summary.json',dict(n=len(records),excluded=len(excluded),reassigned=len(changed),manifest_sha256=digest(dest/'manifest.jsonl'),
        splits={s:dict(n=sum(r['split']==s for r in records),physical_target_xy=sorted(split_sets[s]),
            types=dict(Counter(r['category'] for r in records if r['split']==s)),
            colors=dict(Counter(r['color'] for r in records if r['split']==s and r['color']))) for s in split_sets}))
    print('FROZEN physical split',len(records),'removed',len(excluded),'reassigned',len(changed),flush=True)


if __name__=='__main__':main()

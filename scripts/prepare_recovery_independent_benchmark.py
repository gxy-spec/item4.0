"""Newly rendered calibration/test snapshots; never add these images to training."""
import sys,json,cv2,numpy as np,random,argparse
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from prepare_recovery_nearfield_attributes import verify_paint_response


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--capture',default='independent_attribute_holdout_v1');parser.add_argument('--dest',default='independent_attribute_benchmark_v2');parser.add_argument('--expected',type=int,default=2048);args=parser.parse_args()
    dest=OUT/args.dest;dest.mkdir(exist_ok=True)
    if (dest/'manifest.jsonl').exists():raise RuntimeError('Benchmark already frozen')
    capture=rows(OUT/f'{args.capture}/manifest.jsonl')
    if len(capture)!=args.expected or len({r['id'] for r in capture})!=args.expected:raise RuntimeError('Capture incomplete or duplicate IDs')
    labels=verify_paint_response(capture,dest);records=[];excluded=[];color_unobservable=[];rng=random.Random(20261022)
    for r in capture:
        box=r['bbox'];label=labels[r['id']]
        if not box or r['mask_pixels']<80 or min(box[2]-box[0],box[3]-box[1])<8:
            excluded.append(dict(id=r['id'],reason='Tiny/missing semantic footprint; no detector-dependent filtering'));continue
        if label is None:color_unobservable.append(r['id'])
        image=cv2.imread(r['image']);x1,y1,x2,y2=box;pad=max(2,round(min(x2-x1,y2-y1)*.08))
        crop=image[max(0,y1-pad):min(600,y2+pad),max(0,x1-pad):min(800,x2+pad)]
        p=dest/f'crops/{r["split"]}/{r["id"]}.png';p.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(p),crop)
        records.append({**r,'crop':str(p),'color':label,'source':'new_controlled_primary_vehicle',
            'label_provenance':'Known offline vehicle type and controlled RGB-verified paint identity; fixed white bus/red firetruck visually audited',
            'trained_on_these_images':False})
        if r['requested_color'] not in ['red','white'] or (r['requested_color']=='white' and r['category']!='bus'):continue
        encoded=cv2.imread(r['instance'])
        for attempt in range(50):
            w,h=rng.randint(36,140),rng.randint(28,115);x,y=rng.randint(0,800-w),rng.randint(0,600-h);tags=encoded[y:y+h,x:x+w,2]
            if np.isin(tags,[14,15,16,17,18,19]).any() or np.isin(tags,[1,2,3,4,9,10,11]).mean()<.95:continue
            q=dest/f'crops/{r["split"]}/{r["id"]}_background.png';cv2.imwrite(str(q),image[y:y+h,x:x+w])
            records.append({**r,'id':r['id']+'_background','crop':str(q),'category':'non_vehicle','color':None,
                'bbox':[x,y,x+w,y+h],'source':'new_semantic_nonvehicle_negative','label_provenance':'>=95% certified nonvehicle semantics, zero vehicle/person semantic pixels; no model labels',
                'trained_on_these_images':False});break
    (dest/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    save(dest/'summary.json',dict(captured_images=len(capture),crop_records=len(records),excluded=excluded,color_unsupervised_ids=color_unobservable,
        manifest_sha256=digest(dest/'manifest.jsonl'),capture_manifest_sha256=digest(OUT/f'{args.capture}/manifest.jsonl'),
        sites=json.loads((OUT/f'{args.capture}/predeclared_sites.json').read_text(encoding='utf-8')),
        splits={s:dict(n=sum(r['split']==s for r in records),types=dict(Counter(r['category'] for r in records if r['split']==s)),colors=dict(Counter(r['color'] for r in records if r['split']==s and r['color']))) for s in ['validation','test']},
        scope='Fresh validation/test image sequences from predeclared target deployments, no new benchmark images used for training. Type labels retained when color unobservable; color evaluated only where responding paint is visible. Same map/assets/camera conventions; original city-wide backgrounds and moving-object locations can overlap. Classifier benchmark uses GT object crops; full frontend audited separately, not exhaustive AP.'))
    print('FROZEN independent benchmark',len(records),'excluded',len(excluded),flush=True)


if __name__=='__main__':main()

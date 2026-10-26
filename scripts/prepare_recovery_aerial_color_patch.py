"""Training-only aerial colors, fresh validation, retired diagnostic test."""
import sys,json,cv2,numpy as np
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
from prepare_recovery_nearfield_attributes import verify_paint_response


def main():
    dest=OUT/'attribute_data_v7';dest.mkdir(exist_ok=True)
    if (dest/'manifest.jsonl').exists():raise RuntimeError('Already frozen')
    capture=rows(OUT/'aerial_training_patch_v1/manifest.jsonl')
    if len(capture)!=512 or len({r['id'] for r in capture})!=512:raise RuntimeError('Expected512 aerial training frames')
    labels=verify_paint_response(capture,dest);records=[];excluded=[]
    for r in capture:
        box=r['bbox']
        if not box or r['mask_pixels']<80 or min(box[2]-box[0],box[3]-box[1])<8:
            excluded.append(r['id']);continue
        image=cv2.imread(r['image']);x1,y1,x2,y2=box;pad=max(2,round(min(x2-x1,y2-y1)*.08))
        p=dest/f'crops/{r["id"]}.png';p.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(p),image[max(0,y1-pad):min(600,y2+pad),max(0,x1-pad):min(800,x2+pad)])
        records.append({**r,'crop':str(p),'color':labels[r['id']],'source':'aerial_training_rgb_paint_response','source_image':r['image']})
    train=[r for r in rows(OUT/'attribute_data_v5/manifest.jsonl') if r['split']=='train']+records
    benchmark=rows(OUT/'independent_attribute_benchmark_v2/manifest.jsonl')
    data=train+benchmark
    (dest/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in data),encoding='utf-8')
    save(dest/'summary.json',dict(training_n=len(train),new_aerial_frames=len(records),new_aerial_colors=dict(Counter(r['color'] for r in records if r['color'])),excluded=excluded,
        manifest_sha256=digest(dest/'manifest.jsonl'),
        validation='New V1/V2 only; none of these pixels admitted to training',
        test='T1/T2 are now developmental diagnostics after revealing the aerial-black weakness; no longer advertised as a pristine blind final test. Final independent audit, if collected, must occur after model/online acceptance freeze.',
        repair='Fine-tune color head only. Freeze backbone and type head, including BatchNorm statistics, to preserve demonstrated type improvement.'))
    print('AERIAL COLOR PATCH FROZEN',len(data),'newcolors',Counter(r['color'] for r in records),flush=True)


if __name__=='__main__':main()

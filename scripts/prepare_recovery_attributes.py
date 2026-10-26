"""Create quality-filtered RGB vehicle crops from semantically repaired labels.

HSV is an offline label-quality filter only, not the final attribute predictor.
The filter cannot certify all RGB visibility; visual QA and held-out testing follow.
"""
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json,math
from pathlib import Path
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.attributes import analyze_color
from scripts.export_instance_segmentation_dataset import vehicle_actor_mask

SOURCE=Path('E:/CarlaAirData/CityInspection_GOC/datasets/perception_instance_town10hd_v2_semantic_repair_20261021')

def process(pair):
    index,r=pair;result=[];counts=Counter()
    image=cv2.imread(r['image']);encoded=cv2.imread(r['instance_source'])
    if image is None or encoded is None:raise FileNotFoundError(r['image'])
    b=encoded[:,:,0].astype(np.uint16);g=encoded[:,:,1].astype(np.uint16)
    ids=b+256*g if r['instance_id_byte_order']=='B_plus_256G' else g+256*b
    h,w=image.shape[:2]
    for a in r['annotations']:
        counts['original_annotations']+=1
        if a['color'] not in COLORS or a['category'] not in CLASSES:counts['outside_label_scope']+=1;continue
        if a['role'] in ['ugv','uav','inspection_ugv','inspection_uav']:counts['execution_vehicle_excluded']+=1;continue
        mask=vehicle_actor_mask(ids,encoded,a['actor_id']);ys,xs=np.where(mask)
        if len(xs)<80:counts['too_few_vehicle_pixels']+=1;continue
        x1,y1,x2,y2=int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1
        bw,bh=x2-x1,y2-y1
        if min(bw,bh)<8 or max(bw,bh)>550 or bw*bh>w*h*.6:counts['size_rejected']+=1;continue
        ratio=len(xs)/(bw*bh)
        if ratio<.20:counts['fragmented_mask']+=1;continue
        e=analyze_color(image,[x1,y1,x2,y2],mask);paint=e.score(a['color'])
        threshold=.08 if a['color'] in ['red','blue','yellow'] else .16
        if paint<threshold or e.valid_pixel_count*paint<12:counts['rgb_color_visibility_unconfirmed']+=1;continue
        padding=max(2,int(min(bw,bh)*.08));box=[max(0,x1-padding),max(0,y1-padding),min(w,x2+padding),min(h,y2+padding)]
        crop=image[box[1]:box[3],box[0]:box[2]]
        target=OUT/f'attribute_data/crops/{r["split"]}/{index:05d}_{a["actor_id"]}.jpg'
        target.parent.mkdir(parents=True,exist_ok=True)
        if not cv2.imwrite(str(target),crop,[cv2.IMWRITE_JPEG_QUALITY,95]):raise IOError(str(target))
        result.append({'id':target.stem,'crop':str(target),'category':a['category'],'color':a['color'],
            'split':r['split'],'scene':r['scene'],'run':r['run'],'device':r['device'],'frame':r['frame'],
            'source_image':r['image'],'instance_source':r['instance_source'],'actor_id':a['actor_id'],
            'bbox':box,'mask_pixels':len(xs),'offline_color_evidence':paint,'role':a['role'],'type_id':a['type_id'],
            'quality_scope':'vehicle semantics + RGB color evidence; not independently exhaustive RGB visibility certification'})
        counts['retained_crops']+=1
    return result,counts

def main():
    initialize();dest=OUT/'attribute_data';dest.mkdir(exist_ok=True)
    if (dest/'manifest.jsonl').exists():raise RuntimeError('Frozen crop manifest already exists; use a new data version for changes')
    source=rows(SOURCE/'manifest.jsonl');records=[];counts=Counter();cv2.setNumThreads(1)
    with ThreadPoolExecutor(max_workers=4) as pool:
        for crops,c in pool.map(process,enumerate(source)):records.extend(crops);counts.update(c)
    (dest/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    summary={'filter_counts':dict(counts),'by_split_category':{},'by_split_color':{},'by_device':dict(Counter(r['device'] for r in records)),
        'input_manifest_sha256':digest(SOURCE/'manifest.jsonl'),'crop_manifest_sha256':digest(dest/'manifest.jsonl')}
    for split in ['train','validation','test']:
        subset=[r for r in records if r['split']==split]
        summary['by_split_category'][split]=dict(Counter(r['category'] for r in subset));summary['by_split_color'][split]=dict(Counter(r['color'] for r in subset))
    save(dest/'summary.json',summary)
    for label in CLASSES:
        selected=[]
        for split in ['train','validation','test']:
            for device in ['uav','ugv']:
                candidates=[r for r in records if r['category']==label and r['split']==split and r['device']==device]
                for idx in np.linspace(0,len(candidates)-1,min(4,len(candidates))).astype(int):selected.append(candidates[idx])
        panels=[]
        for r in selected:
            im=cv2.imread(r['crop']);s=min(180/im.shape[1],145/im.shape[0]);im=cv2.resize(im,(max(1,int(im.shape[1]*s)),max(1,int(im.shape[0]*s))))
            panel=np.full((190,210,3),248,np.uint8);panel[25:25+im.shape[0],10:10+im.shape[1]]=im
            cv2.putText(panel,f'{r["id"]} {r["device"]}',(5,16),0,.38,(20,30,40),1)
            cv2.putText(panel,f'{r["split"]} {r["color"]} {label}',(5,185),0,.38,(20,30,40),1);panels.append(panel)
        while len(panels)%4:panels.append(np.full_like(panels[0],255))
        cv2.imwrite(str(dest/f'qa_{label}.jpg'),np.vstack([np.hstack(panels[i:i+4]) for i in range(0,len(panels),4)]))
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__':main()

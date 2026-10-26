"""Attribute data with detector-assisted RGB visibility and honest color supervision.

Type labels come from recorded vehicle identity; color loss is enabled only
when requested paint and clearly observed dominant RGB color agree. This
rejects fixed-material/roof ambiguity instead of training wrong paint labels.
"""
from collections import Counter,defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json,random,sys
import cv2,numpy as np
from recovery_stage import *
sys.path.insert(0,str(ROOT))
from src.perception.attributes import analyze_color
from src.perception.candidate_pipeline import CandidatePipeline
from scripts.export_instance_segmentation_dataset import vehicle_actor_mask

SOURCE=Path('E:/CarlaAirData/CityInspection_GOC/datasets/perception_instance_town10hd_v2_semantic_repair_20261021')
DEST=OUT/'attribute_data_v2'

def color_label(e,requested):
    if e is None:return None
    ratios=e.ratios if hasattr(e,'ratios') else e['ratios']
    if len(ratios)<2:return None
    values=sorted(ratios.items(),key=lambda kv:kv[1],reverse=True)
    dominant,confidence=values[0];second=values[1][1]
    if dominant==requested and confidence>=.30 and confidence-second>=.10:return requested
    return None

def process(pair):
    index,r,boxes=pair;records=[];counter=Counter();image=cv2.imread(r['image']);encoded=cv2.imread(r['instance_source'])
    b=encoded[:,:,0].astype(np.uint16);g=encoded[:,:,1].astype(np.uint16)
    ids=b+256*g if r['instance_id_byte_order']=='B_plus_256G' else g+256*b
    for a in r['annotations']:
        if a['category'] not in CLASSES or a['role'] in ['ugv','uav','inspection_ugv','inspection_uav']:continue
        mask=vehicle_actor_mask(ids,encoded,a['actor_id']);ys,xs=np.where(mask)
        if len(xs)<80:continue
        box=[int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1];bw,bh=box[2]-box[0],box[3]-box[1]
        if min(bw,bh)<8 or bw*bh>800*600*.6:continue
        matched=max(boxes,key=lambda p:CandidatePipeline._iou(p['bbox'],box),default=None)
        if not matched or CandidatePipeline._iou(matched['bbox'],box)<.5:counter['no_rgb_vehicle_box']+=1;continue
        evidence=analyze_color(image,box,mask);label=color_label(evidence,a['color'])
        pad=max(2,round(min(bw,bh)*.08));crop=image[max(0,box[1]-pad):min(600,box[3]+pad),max(0,box[0]-pad):min(800,box[2]+pad)]
        path=DEST/f'crops/{r["split"]}/old_{index:05d}_{a["actor_id"]}.jpg';path.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(path),crop)
        records.append({'id':path.stem,'crop':str(path),'category':a['category'],'color':label,
            'requested_paint':a['color'],'split':r['split'],'scene':r['scene'],'device':r['device'],
            'source_image':r['image'],'bbox':box,'frame':r['frame'],'actor_id':a['actor_id'],
            'source':'existing_scene_grouped_corpus','rgb_vehicle_iou':CandidatePipeline._iou(matched['bbox'],box),
            'color_supervised':label is not None})
        counter['old_type_crops']+=1;counter['old_color_supervised']+=label is not None
    # Deterministic known non-vehicle patches. They contain >=95% road/building/
    # wall/sky pixels and zero vehicle semantic pixels; Other is not trusted.
    if index%4==0:
        rng=random.Random(12000+index)
        for attempt in range(12):
            bw=rng.randint(30,150);bh=rng.randint(25,120);x=rng.randint(0,800-bw);y=rng.randint(0,600-bh)
            tags=encoded[y:y+bh,x:x+bw,2]
            if np.isin(tags,[14,15,16,17,18,19]).any() or np.isin(tags,[1,2,3,4,9,10,11]).mean()<.95:continue
            path=DEST/f'crops/{r["split"]}/background_{index:05d}.jpg';path.parent.mkdir(parents=True,exist_ok=True)
            cv2.imwrite(str(path),image[y:y+bh,x:x+bw]);records.append({'id':path.stem,'crop':str(path),
                'category':'non_vehicle','color':None,'split':r['split'],'scene':r['scene'],'device':r['device'],
                'source_image':r['image'],'bbox':[x,y,x+bw,y+bh],'source':'semantic_nonvehicle_negative','color_supervised':False});counter['background_crops']+=1;break
    return records,counter

def main():
    DEST.mkdir(exist_ok=True)
    if (DEST/'manifest.jsonl').exists():raise RuntimeError('Frozen v2 manifest already exists')
    cached={r['image']:r['boxes'] for r in rows(OUT/'attribute_data/detector_visibility_cache.jsonl')}
    original=rows(SOURCE/'manifest.jsonl')
    if len(cached)!=len(original):raise RuntimeError('All source-frame detections must be cached')
    all_records=[];counts=Counter();cv2.setNumThreads(1)
    with ThreadPoolExecutor(max_workers=4) as pool:
        for records,counter in pool.map(process,[(i,r,cached[r['image']]) for i,r in enumerate(original)]):all_records.extend(records);counts.update(counter)
    captures=rows(OUT/'balanced_attribute_capture/manifest.jsonl');last={r['id']:r for r in captures}
    sequence_ids=defaultdict(set)
    for r in last.values():sequence_ids[r['sequence']].add(r['id'])
    if len(sequence_ids)!=192 or any(len(v)!=8 for v in sequence_ids.values()):raise RuntimeError('All 192 balanced capture sequences must be complete')
    for r in last.values():
        if r['mask_pixels']<100 or not r['bbox']:counts['capture_not_in_view']+=1;continue
        box=r['bbox'];bw,bh=box[2]-box[0],box[3]-box[1]
        if min(bw,bh)<8:continue
        image=cv2.imread(r['image']);pad=max(2,round(min(bw,bh)*.08));crop=image[max(0,box[1]-pad):min(600,box[3]+pad),max(0,box[0]-pad):min(800,box[2]+pad)]
        label=color_label(r['color_evidence'],r['requested_color'])
        # For these controlled snapshots use a clear expected-color observation
        # to establish RGB visibility. Ambiguous samples remain archived, not
        # silently labelled from an actor that may be hidden by map geometry.
        if label is None:counts['capture_color_or_visibility_unconfirmed']+=1;continue
        path=DEST/f'crops/{r["split"]}/balanced_{r["id"]}.jpg';path.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(path),crop)
        all_records.append({'id':path.stem,'crop':str(path),'category':r['category'],'color':label,
            'requested_paint':r['requested_color'],'split':r['split'],'scene':'Town10HD:'+r['scene'],'device':r['device'],
            'source_image':r['image'],'bbox':box,'frame':r['frame'],'actor_id':r['actor_id'],
            'source':'balanced_virtual_camera_capture','color_supervised':True})
        counts['balanced_crops']+=1
    manifest=DEST/'manifest.jsonl';manifest.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in all_records),encoding='utf-8')
    summary={'counts':dict(counts),'split_counts':{},'limitations':'detector-assisted conditional attribute corpus and controlled virtual-camera color coverage; not exhaustive detection or closed-loop acceptance',
        'source_manifest_sha256':digest(SOURCE/'manifest.jsonl'),'manifest_sha256':digest(manifest)}
    for split in ['train','validation','test']:
        selected=[r for r in all_records if r['split']==split]
        summary['split_counts'][split]={'n':len(selected),'types':dict(Counter(r['category'] for r in selected)),
            'supervised_colors':dict(Counter(r['color'] for r in selected if r['color'])),
            'scenes':sorted(set(r['scene'] for r in selected))}
        if set(summary['split_counts'][split]['supervised_colors'])!=set(COLORS):raise RuntimeError(f'Missing color coverage: {split}')
    for a,b in [('train','validation'),('train','test'),('validation','test')]:
        if set(summary['split_counts'][a]['scenes']) & set(summary['split_counts'][b]['scenes']):raise RuntimeError('Scene split leakage')
    save(DEST/'summary.json',summary);print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    for cat in CLASSES+['non_vehicle']:
        selected=[]
        for split in ['train','validation','test']:
            pool=[r for r in all_records if r['category']==cat and r['split']==split]
            for i in np.linspace(0,len(pool)-1,min(8,len(pool))).astype(int):selected.append(pool[i])
        panels=[]
        for r in selected:
            image=cv2.imread(r['crop']);s=min(180/image.shape[1],145/image.shape[0]);im=cv2.resize(image,(max(1,int(image.shape[1]*s)),max(1,int(image.shape[0]*s))))
            panel=np.full((190,210,3),248,np.uint8);panel[25:25+im.shape[0],10:10+im.shape[1]]=im
            cv2.putText(panel,r['id'][-24:],(4,16),0,.35,(20,30,40),1)
            cv2.putText(panel,f'{r["split"]} {r["color"] or "COLOR_UNSUP"}',(4,185),0,.36,(20,30,40),1);panels.append(panel)
        while len(panels)%4:panels.append(np.full_like(panels[0],255))
        cv2.imwrite(str(DEST/f'qa_{cat}.jpg'),np.vstack([np.hstack(panels[i:i+4]) for i in range(0,len(panels),4)]))

if __name__=='__main__':main()

"""Add near-field type evidence; preserve old splits and all label provenance."""
import sys,json,cv2,numpy as np
from pathlib import Path
from collections import Counter,defaultdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.candidate_pipeline import CandidatePipeline
from src.perception.attributes import analyze_color
from scripts.export_instance_segmentation_dataset import decode_actor_ids,vehicle_actor_mask
from prepare_recovery_attributes_v3 import color_label


def verify_paint_response(capture,dest):
    """Supervise semantic paint identity only where RGB actually responds.

    Same camera/vehicle geometry, different controlled paints. The responding
    pixel set must change in the red/blue directions AND white/black luminance.
    This supplies color-constancy labels without using HSV as ground truth.
    """
    groups=defaultdict(dict);labels={};audits=[]
    for r in capture:
        key=(r['scene'],r['blueprint'],r['id'].split('_b')[-1])
        groups[key][r['requested_color']]=r
    for key,g in groups.items():
        if len(g)==1:
            r=next(iter(g.values()));labels[r['id']]=r['requested_color']
            audits.append(dict(group=key,coverage=None,provenance='visually audited fixed white primary bus body or fixed red firetruck',labels={r['id']:r['requested_color']}));continue
        if set(g)!=set(COLORS):raise RuntimeError('Incomplete controlled paint group '+str(key))
        images={};masks={}
        for color,r in g.items():
            images[color]=cv2.imread(r['image']).astype(np.int16)
            encoded=cv2.imread(r['instance']);b=encoded[:,:,0].astype(np.uint16);gg=encoded[:,:,1].astype(np.uint16)
            ids=b+256*gg if r['mask_order']=='B_plus_256G' else gg+256*b
            masks[color]=vehicle_actor_mask(ids,encoded,r['actor_id'])
        common=np.logical_and.reduce(list(masks.values()))
        red,blue,white,black,gray=[images[c] for c in ['red','blue','white','black','gray']]
        response=common & (red[:,:,2]-blue[:,:,2]>15) & (blue[:,:,0]-red[:,:,0]>15)
        response &= white.mean(2)-black.mean(2)>20
        response &= gray.mean(2)-black.mean(2)>8
        n=int(response.sum());coverage=n/max(1,int(common.sum()));valid=n>=20 and coverage>=.05
        for color,r in g.items():labels[r['id']]=color if valid else None
        audits.append(dict(group=key,common_mask_pixels=int(common.sum()),paint_response_pixels=n,coverage=coverage,
            primary_paint_visible=valid,mean_rgb={c:images[c][response][:,::-1].mean(0).tolist() if n else None for c in COLORS},
            labels={r['id']:labels[r['id']] for r in g.values()},provenance='controlled RGB paint response, semantic paint identity; no HSV-label agreement filter'))
    save(dest/'paint_response_audit.json',dict(groups=audits,protocol='Fixed view across6 paints; common vehicle-semantic instance mask; red/blue channel changes>15, white-black luminance>20 and gray-black>8; >=20 pixels and>=5% common mask. Colors without sufficient responding visible body remain unsupervised.'))
    return labels


def main():
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(4);cv2.setNumThreads(2)
    dest=OUT/'attribute_data_v5';dest.mkdir(exist_ok=True)
    if (dest/'manifest.jsonl').exists():raise RuntimeError('Frozen manifest exists')
    original=rows(OUT/'attribute_data_v4/manifest.jsonl');capture=rows(OUT/'nearfield_attribute_capture_v1/manifest.jsonl')
    if len(capture)!=3072 or len({r['id'] for r in capture})!=3072:raise RuntimeError('Expected complete 256 sequences x12 views')
    paint_labels=verify_paint_response(capture,dest)
    detector=YOLO(str(ROOT/'models/yolo26x.pt'));records=[];counts=Counter();detections=[]
    for start in range(0,len(capture),8):
        batch=capture[start:start+8];results=detector.predict([r['image'] for r in batch],imgsz=960,conf=.08,classes=[2,3,5,7],device=0,verbose=False)
        for r,res in zip(batch,results):
            boxes=res.boxes.xyxy.cpu().numpy().tolist();best=max(boxes,key=lambda b:CandidatePipeline._iou(b,r['bbox']),default=None) if r['bbox'] else None
            overlap=CandidatePipeline._iou(best,r['bbox']) if best else 0.
            detections.append(dict(id=r['id'],image=r['image'],boxes=boxes,best_target_iou=overlap))
            if not r['bbox'] or r['mask_pixels']<100 or min(r['bbox'][2]-r['bbox'][0],r['bbox'][3]-r['bbox'][1])<8:
                counts['tiny_or_absent']+=1;continue
            # No color-agreement exclusion for the TYPE task. Only require
            # RGB vehicle evidence to avoid labels seen through opaque objects.
            if overlap<.5:counts['rgb_visibility_unconfirmed']+=1;continue
            image=cv2.imread(r['image']);encoded=cv2.imread(r['instance'])
            ids,_=decode_actor_ids(encoded,[r['actor_id']],semantic_tags=[14,15,16]);mask=vehicle_actor_mask(ids,encoded,r['actor_id'])
            label=paint_labels[r['id']]
            crop_specs=[('gt',r['bbox']),('detector',best)] if r['split']=='train' else [('detector',best)]
            for kind,box in crop_specs:
                x1,y1,x2,y2=box;pad=max(2,round(min(x2-x1,y2-y1)*.08))
                crop=image[max(0,int(y1)-pad):min(600,int(np.ceil(y2))+pad),max(0,int(x1)-pad):min(800,int(np.ceil(x2))+pad)]
                p=dest/f'crops/{r["split"]}/near_{r["id"]}_{kind}.jpg';p.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(p),crop)
                records.append(dict(id=p.stem,crop=str(p),category=r['category'],color=label,split=r['split'],
                    device='ugv',scene=r['scene'],source_image=r['image'],bbox=r['bbox'],crop_bbox=box,frame=r['frame'],actor_id=r['actor_id'],
                    source='nearfield_rgb_verified_'+kind,blueprint=r['blueprint'],requested_paint=r['requested_color'],
                    color_supervised=label is not None,color_label_provenance='controlled_paint_response_audit',rgb_vehicle_iou=overlap,surface_gap_m=r['surface_gap_m'],view_azimuth_deg=r['view_azimuth_deg']))
                counts[r['split']+'_crops']+=1
        if start%256==0:print('PREPARED',start,'/',len(capture),flush=True)
    all_records=original+records
    (dest/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in all_records),encoding='utf-8')
    (dest/'nearfield_visibility_cache.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in detections),encoding='utf-8')
    save(dest/'summary.json',dict(counts=dict(counts),original_n=len(original),new_n=len(records),
        splits={s:dict(n=sum(r['split']==s for r in all_records),types=dict(Counter(r['category'] for r in all_records if r['split']==s)),colors=dict(Counter(r['color'] for r in all_records if r['split']==s and r['color']))) for s in ['train','validation','test']},
        manifest_sha256=digest(dest/'manifest.jsonl'),capture_manifest_sha256=digest(OUT/'nearfield_attribute_capture_v1/manifest.jsonl'),
        protocol='Near-field captures: 7 blueprints, 6 mutable paint colors; fixed white bus and red firetruck. Optical-axis pitch -5deg, height2m, approximate surface gaps3.5/5/9m, 12 azimuths. Old region-ID splits retained; physical overlap remains. RGB detector agreement only validates training/conditional crop visibility, not independent full-frame recall. GT/detector crops of one frame always stay in the same split.'))
    print('DATA FROZEN',len(all_records),counts,flush=True)


if __name__=='__main__':main()

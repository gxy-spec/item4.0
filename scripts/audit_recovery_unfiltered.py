"""Known-target recall on ALL held-out virtual-camera frames, without box filtering.

Background annotations are incomplete: this is target-instance recall and
type/color correctness, not exhaustive full-frame precision or AP.
"""
import sys,json,cv2,numpy as np
from pathlib import Path
from collections import defaultdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.candidate_pipeline import CandidatePipeline
from src.perception.vehicle_attributes import VehicleAttributeRecognizer
from src.perception.attributes import analyze_color


def main():
    import torch
    from ultralytics import YOLO
    torch.set_num_threads(4);cv2.setNumThreads(2)
    dataset=[r for r in rows(OUT/'balanced_attribute_capture_v3/manifest.jsonl') if r['split']=='test']
    cal=json.loads((OUT/'calibration/deployment_final.json').read_text(encoding='utf-8'))
    head=VehicleAttributeRecognizer(cal['weights']);records=[];cache={}
    for variant,weight in [('legacy',ROOT/'models/yolo26n.pt'),('current',DETECTOR),('auxiliary',ROOT/'models/yolo26x.pt')]:
        model=YOLO(str(weight));cache[variant]={}
        for start in range(0,len(dataset),8):
            batch=dataset[start:start+8]
            results=model.predict([r['image'] for r in batch],imgsz=960,conf=.08,iou=.7,device=0,verbose=False)
            for r,res in zip(batch,results):
                image=cv2.imread(r['image']);proposals=[]
                for i,(b,s,c) in enumerate(zip(res.boxes.xyxy,res.boxes.conf,res.boxes.cls)):
                    category=res.names[int(c)]
                    if variant!='current' and category not in ['car','motorcycle','bus','truck']:continue
                    box=b.cpu().tolist();mask=CandidatePipeline._instance_mask(res,i,image.shape[:2])
                    proposals.append(dict(bbox=box,score=float(s),category=category,
                        color=analyze_color(image,box,mask).dominant_color))
                cache[variant][r['id']]=proposals
            if start%128==0:print(variant,start,'/',len(dataset),flush=True)
        del model
    for r in dataset:
        image=cv2.imread(r['image'])
        for variant in ['legacy','current','final_rgb_proposals']:
            proposals=(cache['current'][r['id']]+cache['auxiliary'][r['id']] if variant=='final_rgb_proposals' else cache[variant][r['id']])
            p=max(proposals,key=lambda p:CandidatePipeline._iou(p['bbox'],r['bbox']),default=None)
            overlap=CandidatePipeline._iou(p['bbox'],r['bbox']) if p else 0.
            category=p['category'] if p else 'miss';color=p['color'] if p else 'miss'
            if variant=='final_rgb_proposals' and p:
                a=head.predict(image,[p['bbox']])[0]
                category=a['predicted_type'] if a['type_probability']>=cal['type_threshold'] else 'unknown'
                color=a['predicted_color'] if a['color_probability']>=cal['color_threshold'] else 'unknown'
            records.append(dict(id=r['id'],variant=variant,device=r['device'],truth_type=r['category'],
                box_iou=overlap,box_found=overlap>=.5,exact_type_correct=overlap>=.5 and category==r['category'],
                predicted_type=category,predicted_color=color,image=r['image'],bbox=r['bbox'],
                note='Final RGB proposal union upper-bound association; no depth refinement, NMS/containment suppression or online temporal decisions in this offline audit'))
    summaries={}
    for variant in ['legacy','current','final_rgb_proposals']:
        z=[r for r in records if r['variant']==variant]
        def stats(q):return dict(n=len(q),box_iou50=sum(r['box_iou']>=.5 for r in q),box_iou75=sum(r['box_iou']>=.75 for r in q),exact_type_correct=sum(r['exact_type_correct'] for r in q))
        summaries[variant]={**stats(z),'by_type':{c:stats([r for r in z if r['truth_type']==c]) for c in CLASSES},'by_device':{d:stats([r for r in z if r['device']==d]) for d in ['ugv','uav']}}
    dest=OUT/'perception_acceptance_unfiltered';dest.mkdir(exist_ok=True)
    (dest/'predictions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    save(dest/'summary.json',dict(models=summaries,manifest_sha256=digest(OUT/'balanced_attribute_capture_v3/manifest.jsonl'),
        selection='All test frames, no detector-dependent exclusion; region-ID holdout on one map; physical sites can repeat',
        scope='Known target instance recall, not AP or exhaustive precision. Final proposal union is an optimistic RGB-only diagnostic, not the complete runtime pipeline. Colors deliberately not scored against unverified requested paints.'))
    print(json.dumps(summaries),flush=True)


if __name__=='__main__':main()

"""Bounded diagnostic on unmodified RGB from the historical 36-run baseline.

This is a case-control root-cause study, not an AP/generalization benchmark.
Sampling precedes inference and RGB visibility review precedes metric calculation.
"""
from __future__ import annotations
from collections import defaultdict
import argparse
import csv
import json
from pathlib import Path
import time
import cv2
import numpy as np
from ultralytics import YOLO, YOLOE
from perception_online_acceptance import OUT, ROOT, MODELS, read_jsonl, save_json, sha256

DEST=OUT/'reaudit_frozen_baseline'
SCENES=['N1','E1','W2','F1']


def prepare():
    DEST.mkdir(exist_ok=True)
    source=Path('E:/CarlaAirData/CityInspection_GOC/perception_benchmark_v1/all.jsonl')
    groups=defaultdict(list)
    for r in read_jsonl(source):
        if r['scene'] in SCENES and r['policy']=='A':groups[(r['scene'],r['device'])].append(r)
    samples=[]
    for (scene,device),rows in sorted(groups.items()):
        rows.sort(key=lambda x:x['frame']);indices=[0,len(rows)//3,2*len(rows)//3]
        area=lambda r:max(0,r['target_projected_bbox_xyxy'][2]-r['target_projected_bbox_xyxy'][0])*max(0,r['target_projected_bbox_xyxy'][3]-r['target_projected_bbox_xyxy'][1])
        eligible=[i for i,r in enumerate(rows) if r.get('target_projected_in_frame') and r.get('target_visible_depth_proxy')]
        ranked=sorted(eligible,key=lambda i:area(rows[i]),reverse=True)
        indices+=next(([i] for i in ranked if i not in indices),[])
        if len(indices)<4:indices+=next(([i] for i in range(len(rows)-1,-1,-1) if i not in indices),[])
        for i in indices:
            r=dict(rows[i]);r.update(review_id=f'{scene}_{device}_{len([s for s in samples if s["scene"]==scene and s["device"]==device])+1}',
                sampling_reason='fixed time strata + largest depth-proxy-visible target; selected before predictions')
            samples.append(r)
    (DEST/'samples.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in samples),encoding='utf-8')
    save_json(DEST/'sampling_protocol.json',{'source_manifest':str(source),'source_sha256':sha256(source),
        'sample_manifest_sha256':sha256(DEST/'samples.jsonl'),'n':len(samples),'scenes':SCENES,
        'policy':'A only, frozen archived source RGB; no simulator rerun',
        'scope':'diagnostic selection; not unbiased population performance or cross-scene generalization',
        'selection':'first / one-third / two-thirds saved image; add largest projected depth-proxy-visible target not already selected, otherwise last',
        'labels':'RGB visual review by assistant before predictions; unclear cases excluded; not independent double annotation'})
    for scene in SCENES:
        panels=[]
        for r in [s for s in samples if s['scene']==scene]:
            im=cv2.imread(r['image_path']);panel=np.full((255,550,3),250,np.uint8)
            full=im.copy();box=r['target_projected_bbox_xyxy'] or [0,0,0,0];x1,y1,x2,y2=[int(np.clip(v,-10000,10000)) for v in box]
            if r['target_projected_in_frame']:cv2.rectangle(full,(x1,y1),(x2,y2),(220,20,180),2)
            thumb=cv2.resize(full,(300,225));panel[28:253,:300]=thumb
            crop=im[max(0,y1-18):min(600,y2+18),max(0,x1-18):min(800,x2+18)] if r['target_projected_in_frame'] else None
            if crop is not None and crop.size:
                scale=min(235/crop.shape[1],195/crop.shape[0]);res=cv2.resize(crop,(max(1,int(crop.shape[1]*scale)),max(1,int(crop.shape[0]*scale))))
                panel[40:40+res.shape[0],310:310+res.shape[1]]=res
            cv2.putText(panel,f"{r['review_id']} frame={r['frame']}",(8,20),0,.48,(20,30,40),1,cv2.LINE_AA)
            cv2.putText(panel,'GT projected region' if r['target_projected_in_frame'] else 'GT outside view',(310,247),0,.42,(40,40,60),1,cv2.LINE_AA)
            panels.append(panel)
        cv2.imwrite(str(DEST/f'{scene}_RGB_review.jpg'),np.vstack([np.hstack(panels[i:i+2]) for i in range(0,len(panels),2)]))
    print('PREPARED',len(samples),flush=True)


def infer():
    samples=read_jsonl(DEST/'samples.jsonl')
    specs={'historical_yolo26n':ROOT/'models/yolo26n.pt','historical_yolo26x':ROOT/'models/yolo26x.pt',
           'original_yoloe':MODELS['original'],'adapted_yoloe':MODELS['adapted']}
    outputs=[]
    cv2.setNumThreads(2)
    for name,path in specs.items():
        model=YOLOE(str(path)) if 'yoloe' in name else YOLO(str(path))
        if name=='original_yoloe':model.set_classes(['van','car','truck','bus'])
        # The four diagnostic models share input resolution, NMS and floor.
        started=time.perf_counter()
        for offset in range(0,len(samples),8):
            subset=samples[offset:offset+8]
            result=model.predict([r['image_path'] for r in subset],imgsz=960,conf=.03,iou=.7,
                agnostic_nms=False,device=0,verbose=False)
            for sample,pred in zip(subset,result):
                predictions=[]
                for b,c,k in zip(pred.boxes.xyxy.cpu().tolist(),pred.boxes.conf.cpu().tolist(),pred.boxes.cls.cpu().tolist()):
                    category=pred.names[int(k)]
                    if category not in ['van','car','truck','bus','motorcycle']:continue
                    predictions.append({'bbox':b,'confidence':c,'category':category})
                outputs.append({'review_id':sample['review_id'],'model':name,'predictions':predictions})
        print('INFERRED',name,len(samples),f'{time.perf_counter()-started:.2f}s',flush=True)
        del model
    (DEST/'predictions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in outputs),encoding='utf-8')
    save_json(DEST/'model_protocol.json',{'models':{n:{'path':str(p),'sha256':sha256(p)} for n,p in specs.items()},
        'image_size':960,'confidence_floor':.03,'nms_iou':.7,'device':0,
        'metric_scope':'GT-target-associated generic vehicle discovery; COCO models have no van category; NOT four-class AP or complete task success'})


def summarize():
    from src.perception.candidate_pipeline import CandidatePipeline
    samples={r['review_id']:r for r in read_jsonl(DEST/'samples.jsonl')}
    labels=json.loads((DEST/'rgb_visibility_labels.json').read_text(encoding='utf-8'))
    rows=[];groups=defaultdict(list)
    for r in read_jsonl(DEST/'predictions.jsonl'):
        s=samples[r['review_id']];label=labels[r['review_id']]
        target_box=s['target_projected_bbox_xyxy']
        best=max(r['predictions'],key=lambda p:CandidatePipeline._iou(p['bbox'],target_box),default=None) if target_box else None
        iou=CandidatePipeline._iou(best['bbox'],target_box) if best else 0
        row={'sample':r['review_id'],'scene':s['scene'],'device':s['device'],'rgb_visibility':label['status'],
             'review_note':label['note'],'model':r['model'],'matched_box_iou':iou,
             'matched_category':best['category'] if best else None,'matched_confidence':best['confidence'] if best else None,
             'vehicle_target_hit_iou50':label['status']=='visible' and iou>=.5,
             'van_target_hit_iou50':label['status']=='visible' and bool(target_box) and any(
                 p['category']=='van' and CandidatePipeline._iou(p['bbox'],target_box)>=.5 for p in r['predictions'])}
        rows.append(row);groups[r['model']].append(row)
    with (DEST/'paired_diagnostic_details.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary={m:{'rgb_visible_targets':sum(r['rgb_visibility']=='visible' for r in rs),
        'generic_vehicle_target_hits':sum(r['vehicle_target_hit_iou50'] for r in rs),
        'van_category_hits':sum(r['van_target_hit_iou50'] for r in rs) if 'yoloe' in m else None,
        'excluded_unclear':sum(r['rgb_visibility']=='unclear' for r in rs),
        'outside_or_occluded':sum(r['rgb_visibility'] in ['outside_view','occluded'] for r in rs)} for m,rs in groups.items()}
    save_json(DEST/'paired_diagnostic_summary.json',{'sample_count':len(samples),'results':summary,
        'scope':'small selected historical-baseline diagnostic, NOT AP, precision, production acceptance or unseen-scene generalization',
        'review_limit':'assistant visual review; single reviewer; projected 3D box may differ from visible box under partial occlusion'})
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['prepare','infer','summarize']);args=parser.parse_args()
    {'prepare':prepare,'infer':infer,'summarize':summarize}[args.action]()

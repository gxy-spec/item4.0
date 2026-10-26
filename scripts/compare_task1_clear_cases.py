"""Qualitative two-case CPU replay; no population-performance claim."""
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import torch
from perception_online_acceptance import OUT,MODELS,save_json,sha256,read_jsonl,ground_truth
from export_instance_segmentation_dataset import load_actor_states
from src.perception.candidate_pipeline import CandidatePipeline,PipelineConfig
from src.perception.task_spec import TaskSpec

torch.set_num_threads(2);cv2.setNumThreads(2)
index=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'))
run=Path(index['blue_car_42101']['run_dir'])
evaluation=json.loads((run/'task1_evaluation.json').read_text(encoding='utf-8'))
appearance=json.loads((run/'task1_appearance_audit.json').read_text(encoding='utf-8'))
ground=next(v for v in appearance['views'] if v['device']=='ugv')
wrong=evaluation['commits'][0]['evidence']
meta=next(r for r in read_jsonl(run/'sensors/uav/metadata/frames.jsonl') if r['frame']==wrong['frame'])
actors=load_actor_states(run/'actors/actor_states.jsonl')[wrong['frame']]
gt,_=ground_truth(meta['instance_segmentation_path'],actors,require_vehicle_semantics=True)
van=next(g for g in gt if g['actor_id']==wrong['matched_actor_id'])
cases=[{'case':'visible_true_blue_car_ground','image':ground['image'],'gt_bbox':ground['bbox'],'gt_category':'car','gt_color':'blue'},
    {'case':'committed_blue_van_aerial','image':wrong['image'],'gt_bbox':van['bbox'],'gt_category':'van','gt_color':'blue'}]
results=[];panels={c['case']:[] for c in cases}
for name in ['original','adapted']:
    pipeline=CandidatePipeline(PipelineConfig(model_path=MODELS[name],detector_backend='custom' if name=='adapted' else 'open_vocabulary',
        detector_image_size=960,detector_confidence=.03,enable_attribute_proposals=False,device='cpu',
        task_spec=TaskSpec.legacy_red_van(),detector_prompts=('van','car','truck','bus')))
    for case in cases:
        im=cv2.imread(case['image']);result=pipeline.infer_images([im])[0];predictions=[]
        for box,score,cls in zip(result.boxes.xyxy,result.boxes.conf,result.boxes.cls):
            bbox=box.cpu().tolist();predictions.append({'bbox':bbox,'category':result.names[int(cls)],'score':float(score),
                'iou':CandidatePipeline._iou(case['gt_bbox'],bbox)})
        best=max(predictions,key=lambda p:p['iou'],default=None)
        results.append({**case,'model':name,'device':'cpu','model_sha256':sha256(MODELS[name]),'best_overlap_prediction':best,
            'note':'error-selected diagnostic example; not an independent performance estimate'})
        box=case['gt_bbox'];x1,y1,x2,y2=map(int,box)
        annotated=im.copy();cv2.rectangle(annotated,(x1,y1),(x2,y2),(0,200,0),2)
        if best:
            pbox=list(map(int,best['bbox']));cv2.rectangle(annotated,tuple(pbox[:2]),tuple(pbox[2:]),(0,0,220),1)
        crop=annotated[max(0,y1-10):min(600,y2+10),max(0,x1-10):min(800,x2+10)]
        panel=np.full((350,480,3),248,np.uint8)
        cv2.putText(panel,f"{name} | GT: {case['gt_color']} {case['gt_category']}",(8,24),0,.6,(20,20,20),1)
        label='NO PREDICTION' if best is None else f"prediction: {best['category']} {best['score']:.3f} IoU {best['iou']:.2f}"
        cv2.putText(panel,label,(8,50),0,.52,(0,0,180),1)
        scale=min(460/crop.shape[1],270/crop.shape[0]);crop=cv2.resize(crop,(max(1,int(crop.shape[1]*scale)),max(1,int(crop.shape[0]*scale))))
        panel[70:70+crop.shape[0],10:10+crop.shape[1]]=crop;panels[case['case']].append(panel)
save_json(OUT/'analysis/clear_case_model_comparison.json',results)
cv2.imwrite(str(OUT/'analysis/clear_case_model_comparison.jpg'),np.vstack([np.hstack(panels[c['case']]) for c in cases]))
print(json.dumps(results,ensure_ascii=False,indent=2),flush=True)

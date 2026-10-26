"""Offline-only static-map annotation audit; never imports detector predictions."""
from collections import Counter
import json
from pathlib import Path
import sys
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from perception_online_acceptance import OUT,read_jsonl,save_json
from src.perception.candidate_pipeline import project_world_point,CandidatePipeline

inventory=json.loads((OUT/'analysis/static_vehicle_inventory.json').read_text(encoding='utf-8'))
rows=read_jsonl(OUT/'benchmark/manifest.jsonl')
best={};augment={};unmatched=Counter()
for i,r in enumerate(rows):
    ins=cv2.imread(r['instance']);semantic=ins[:,:,2]
    # The original manifest already records the byte order selected using
    # actual actor IDs. Keep it fixed for unknown map components too.
    a=ins[:,:,0].astype(np.uint16);b=ins[:,:,1].astype(np.uint16)
    ids=a+256*b if r['instance_order']=='B_plus_256G' else b+256*a
    vehicle=np.isin(semantic,[14,15,16]) & ~np.isin(ids,[g['actor_id']%65536 for g in r['ground_truth']])
    n,labels,stats,centres=cv2.connectedComponentsWithStats(vehicle.astype(np.uint8))
    projected=[]
    for obj in inventory:
        pts=[project_world_point(tuple(v),r['sensor_transform'],800,600,100) for v in obj['vertices']]
        valid=[p for p in pts if p[2]>.5]
        if len(valid)!=8:continue
        box=[max(0,min(p[0] for p in valid)),max(0,min(p[1] for p in valid)),
             min(800,max(p[0] for p in valid)),min(600,max(p[1] for p in valid))]
        if box[2]<=box[0] or box[3]<=box[1]:continue
        projected.append((obj,box))
    found=[]
    for component in range(1,n):
        x,y,w,h,pixels=map(int,stats[component])
        if pixels<80:continue
        box=[x,y,x+w,y+h]
        closest=max(projected,key=lambda ob:CandidatePipeline._iou(box,ob[1]),default=None)
        overlap=0 if closest is None else CandidatePipeline._iou(box,closest[1])
        if overlap<.5:unmatched[r['split']]+=1;continue
        obj=closest[0]
        found.append({'environment_id':obj['id'],'name':obj['name'],'pixels':pixels,'bbox':box,
            'xyz':obj['bbox_center'],'projected_iou':overlap})
        score=pixels*(2. if r['device']=='ugv' else .5)
        if obj['id'] not in best or score>best[obj['id']]['score']:
            best[obj['id']]={'score':score,'image':r['image'],'sample_id':r['sample_id'],**found[-1]}
    augment[r['sample_id']]=found
    if i%400==0:print(i,'/',len(rows),'static objects seen',len(best),flush=True)
save_json(OUT/'analysis/static_instance_matches.json',augment)
save_json(OUT/'analysis/static_label_audit.json',{'best_crops':list(best.values()),'unmatched_components':dict(unmatched),
    'matching_rule':'unknown vehicle semantic connected component, static projected 3D box IoU >= .5; no predictions used'})
panels=[]
for index,obj in enumerate(inventory):
    b=best.get(obj['id']);panel=np.full((240,320,3),245,np.uint8)
    cv2.putText(panel,f"#{index} {obj['name'][:33]}",(8,18),0,.39,(0,0,0),1)
    if b:
        im=cv2.imread(b['image']);x1,y1,x2,y2=b['bbox'];crop=im[max(0,y1-8):min(600,y2+8),max(0,x1-8):min(800,x2+8)]
        scale=min(304/crop.shape[1],180/crop.shape[0]);crop=cv2.resize(crop,(int(crop.shape[1]*scale),int(crop.shape[0]*scale)))
        panel[30:30+crop.shape[0],8:8+crop.shape[1]]=crop
        cv2.putText(panel,f"{b['pixels']}px IoU {b['projected_iou']:.2f}",(8,229),0,.42,(0,0,0),1)
    else:cv2.putText(panel,'NOT VISIBLE IN BENCHMARK',(8,110),0,.5,(0,0,180),1)
    panels.append(panel)
while len(panels)%4:panels.append(np.full_like(panels[0],255))
cv2.imwrite(str(OUT/'analysis/static_vehicle_annotation_gallery.jpg'),np.vstack([np.hstack(panels[i:i+4]) for i in range(0,len(panels),4)]))
print('Gallery saved; seen',len(best),'of',len(inventory),flush=True)

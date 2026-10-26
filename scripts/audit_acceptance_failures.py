"""Generate validation-only visual audits before interpreting calibration."""
import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from perception_online_acceptance import OUT,read_jsonl

rows=[r for r in read_jsonl(OUT/'benchmark/adapted_predictions.jsonl') if r['split']=='validation']
examples=[]
for r in rows:
    category,color,_=r['queries'][0]
    ps=[p for p in r['predictions'] if p['category']==category and p['confidence']>=.5 and p['color_ratios'].get(color,0)>=.2]
    if not ps:continue
    p=max(ps,key=lambda p:p['confidence']*(.5+.5*p['color_ratios'][color]))
    right=any(g['actor_id']==p['matched_actor'] and g['category']==category and g['color']==color for g in r['ground_truth'])
    if right:continue
    if any(e['run']==r['run'] and e['device']==r['device'] for e in examples):continue
    examples.append(r)
    if len(examples)>=8:break
panels=[]
for r in examples:
    im=cv2.imread(r['image'])
    for g in r['ground_truth']:
        x1,y1,x2,y2=g['bbox'];cv2.rectangle(im,(x1,y1),(x2,y2),(0,220,0),2)
        cv2.putText(im,f"GT {g['actor_id']} {g['color']} {g['category']}",(x1,max(12,y1-6)),0,.42,(0,200,0),1)
    for p in r['predictions']:
        if p['confidence']<.5:continue
        x1,y1,x2,y2=map(int,p['bbox']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,0,255),1)
        cv2.putText(im,f"P {p['dominant_color']} {p['category']} {p['confidence']:.2f} IoU{p['matched_iou']:.2f}",(x1,min(590,y2+12)),0,.4,(0,0,255),1)
    cv2.putText(im,r['sample_id'],(8,22),0,.5,(0,200,255),1)
    cv2.putText(im,'TASK '+str(r['queries'][0][:2]),(8,44),0,.5,(0,200,255),1)
    panels.append(cv2.resize(im,(640,480)))
while len(panels)%2:panels.append(np.zeros_like(panels[0]))
out=OUT/'analysis/validation_failure_audit.jpg';out.parent.mkdir(parents=True,exist_ok=True)
cv2.imwrite(str(out),np.vstack([np.hstack(panels[i:i+2]) for i in range(0,len(panels),2)]))
print(out)

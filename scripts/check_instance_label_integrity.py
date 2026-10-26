"""Check whether recorded actor-ID masks contain non-vehicle semantics."""
from collections import Counter
import cv2
import numpy as np
from perception_online_acceptance import OUT,read_jsonl,save_json
from analyze_online_acceptance import table

rows=read_jsonl(OUT/'benchmark/manifest.jsonl');details=[];summary=Counter()
for r in rows:
    ins=cv2.imread(r['instance']);b=ins[:,:,0].astype(np.uint16);g=ins[:,:,1].astype(np.uint16)
    ids=b+256*g if r['instance_order']=='B_plus_256G' else g+256*b
    for actor in r['ground_truth']:
        mask=ids==actor['actor_id']%65536
        all_pixels=int(mask.sum());vehicle_pixels=int((mask&np.isin(ins[:,:,2],[14,15,16])).sum())
        nonvehicle=all_pixels-vehicle_pixels
        summary['actor_masks']+=1;summary['masks_with_nonvehicle_pixels']+=nonvehicle>0
        summary['masks_majority_nonvehicle']+=nonvehicle>vehicle_pixels
        summary['total_mask_pixels']+=all_pixels;summary['nonvehicle_mask_pixels']+=nonvehicle
        if nonvehicle:
            details.append({'sample_id':r['sample_id'],'split':r['split'],'device':r['device'],
                'actor_id':actor['actor_id'],'role':actor['role'],'category':actor['category'],
                'all_pixels':all_pixels,'vehicle_pixels':vehicle_pixels,'nonvehicle_pixels':nonvehicle,
                'semantic_tag_counts':str(dict(Counter(ins[:,:,2][mask].tolist())))})
save_json(OUT/'analysis/instance_label_integrity.json',dict(summary))
table(OUT/'analysis/instance_label_nonvehicle_masks.csv',details)
print(dict(summary),flush=True)

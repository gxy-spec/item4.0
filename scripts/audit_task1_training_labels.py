"""Audit the actual 9163-image train/val/test dataset, without rewriting labels."""
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from perception_online_acceptance import OUT,read_jsonl,save_json


def inspect(row):
    image=cv2.imread(row['instance_source'],cv2.IMREAD_COLOR)
    if image is None:raise FileNotFoundError(row['instance_source'])
    b=image[:,:,0].astype(np.uint16);g=image[:,:,1].astype(np.uint16)
    ids=b+256*g if row['instance_id_byte_order']=='B_plus_256G' else g+256*b
    vehicle=np.isin(image[:,:,2],[14,15,16]);counts=Counter(images=1);details=[]
    for actor in row['annotations']:
        mask=ids==actor['actor_id']%65536;total=int(mask.sum());valid=int((mask&vehicle).sum());bad=total-valid
        counts['annotations']+=1;counts['masks_with_nonvehicle_pixels']+=bad>0
        counts['masks_without_vehicle_pixels']+=valid==0
        counts['nonvehicle_pixels']+=bad;counts['all_mask_pixels']+=total
        if bad:details.append({'split':row['split'],'scene':row['scene'],'device':row['device'],'frame':row['frame'],
            'image':row['image'],'instance_source':row['instance_source'],'actor_id':actor['actor_id'],'category':actor['category'],
            'mask_pixels':total,'vehicle_pixels':valid,'nonvehicle_pixels':bad,
            'semantic_tags':json.dumps(dict(Counter(image[:,:,2][mask].tolist())))})
    counts['images_with_nonvehicle_annotations']+=bool(details)
    return row['split'],counts,details


if __name__=='__main__':
    dest=OUT/'analysis/followup_questions';dest.mkdir(exist_ok=True)
    source=Path('E:/CarlaAirData/CityInspection_GOC/datasets/perception_instance_town10hd_v2/manifest.jsonl')
    rows=read_jsonl(source);summaries=defaultdict(Counter);details=[];cv2.setNumThreads(1)
    with ThreadPoolExecutor(max_workers=4) as pool:
        for split,counts,items in pool.map(inspect,rows):summaries[split].update(counts);details.extend(items)
    save_json(dest/'training_dataset_semantic_audit.json',{'source':str(source),'images':len(rows),'splits':dict(summaries),
        'scope':'actual exported train/val/test actor masks vs vehicle semantic tags; no RGB visibility verification; labels unchanged'})
    if details:
        with (dest/'training_dataset_nonvehicle_labels.csv').open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(details[0]));w.writeheader();w.writerows(details)
    print(json.dumps(dict(summaries),indent=2),flush=True)

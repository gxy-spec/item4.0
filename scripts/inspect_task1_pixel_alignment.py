"""Offline pixel/geometry consistency check for the foreground actor."""
from pathlib import Path
import json
import cv2
import numpy as np
from perception_online_acceptance import OUT,read_jsonl,save_json
from export_instance_segmentation_dataset import load_actor_states,decode_actor_ids
from build_open_vocab_benchmark import projected_actor_box

index=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'))
results=[]
for name in ['red_van_42101','blue_car_42101','white_truck_42101','yellow_bus_42101']:
    if name not in index:continue
    run=Path(index[name]['run_dir']);audit=json.loads((run/'task1_appearance_audit.json').read_text(encoding='utf-8'))
    view=next(v for v in audit['views'] if v['device']=='uav')
    frame=view['frame'];actors=load_actor_states(run/'actors/actor_states.jsonl')[frame]
    target=next(a for a in actors if a['role']=='target')
    m=next(m for m in read_jsonl(run/'sensors/uav/metadata/frames.jsonl') if m['frame']==frame)
    ins=cv2.imread(m['instance_segmentation_path']);im=cv2.imread(m['rgb_path'])
    ids,order=decode_actor_ids(ins,[a['actor_id'] for a in actors]);mask=ids==target['actor_id']%65536
    box,center,depth=projected_actor_box(target,m['sensor_transform'])
    x,y=map(lambda x:int(round(x)),center)
    item={'case':name,'frame':frame,'camera':m['sensor_transform'],'target':target,'byte_order':order,
        'actor_id_mask_pixels':int(mask.sum()),'semantic_values':dict(zip(*[a.tolist() for a in np.unique(ins[:,:,2][mask],return_counts=True)])),
        'projected_bbox':box,'projected_center':center,'mask_bbox':view['bbox']}
    if 0<=x<800 and 0<=y<600:
        item['encoded_bgr_at_projected_center']=ins[y,x].tolist();item['decoded_id_at_projected_center']=int(ids[y,x])
    if box:
        cv2.rectangle(im,tuple(map(int,box[:2])),tuple(map(int,box[2:])),(0,255,255),2)
    x1,y1,x2,y2=view['bbox'];cv2.rectangle(im,(x1,y1),(x2,y2),(0,0,255),2)
    cv2.putText(im,'Yellow: projected GT; red: ID mask bbox',(10,22),0,.5,(0,200,255),1)
    cv2.imwrite(str(OUT/f'analysis/alignment_{name}.jpg'),im)
    results.append(item)
save_json(OUT/'analysis/foreground_pixel_geometry_alignment.json',results)
print(json.dumps(results,ensure_ascii=False,indent=2)[:12000],flush=True)

"""Check rendered foreground appearance against configured task attributes."""
from pathlib import Path
import json
import sys
import cv2
import numpy as np
import yaml
from perception_online_acceptance import OUT,read_jsonl,save_json
from src.perception.attributes import analyze_color
from src.perception.candidate_pipeline import CandidatePipeline
from export_instance_segmentation_dataset import decode_actor_ids


def audit_run(item):
    run=Path(item['run_dir']);out=run/'task1_appearance_audit.json'
    if out.exists():
        cached=json.loads(out.read_text(encoding='utf-8'))
        if cached.get('audit_version')==2:return cached
    cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
    initial=json.loads((run/'actors/initial_actor_states.json').read_text(encoding='utf-8'))
    target=next(a for a in initial if a['role']=='target');actor_id=target['actor_id']%65536
    results=[]
    for device in ['uav','ugv']:
        best=None;best_mask=None;best_image=None;count=0;visible=0
        candidates={r['frame_id']:r['candidates'] for r in read_jsonl(run/f'perception/{device}_target_candidates_synchronized.jsonl')}
        for m in read_jsonl(run/f'sensors/{device}/metadata/frames.jsonl'):
            if not m.get('instance_segmentation_path'):continue
            count+=1;ins=cv2.imread(m['instance_segmentation_path'])
            ids,order=decode_actor_ids(ins,[a['actor_id'] for a in initial])
            mask=(ids==actor_id)&np.isin(ins[:,:,2],[14,15,16])
            pixels=int(mask.sum());visible+=pixels>=80
            if pixels<9 or (best and pixels<=best['pixels']):continue
            ys,xs=np.where(mask);box=[int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1]
            best={'device':device,'frame':m['frame'],'timestamp':m['timestamp'],'image':m['rgb_path'],'pixels':pixels,'bbox':box}
            best_mask=mask;best_image=cv2.imread(m['rgb_path'])
        if best:
            e=analyze_color(best_image,best['bbox'],best_mask);expected=cfg['online_acceptance']['task_color']
            ps=candidates.get(best['frame'],[])
            c=max(ps,key=lambda c:CandidatePipeline._iou(best['bbox'],c['bbox_xyxy']),default=None)
            best.update(configured_task_color=expected,gt_mask_dominant_color=e.dominant_color,
                gt_mask_expected_color_ratio=e.score(expected),visible_frame_count=visible,saved_frame_count=count,
                matching_detection=None if c is None else {'category':c['category'],'color':c['color'],
                    'confidence':c['detector_score'],'iou':CandidatePipeline._iou(best['bbox'],c['bbox_xyxy'])})
            x1,y1,x2,y2=best['bbox'];crop=best_image[max(0,y1-10):min(600,y2+10),max(0,x1-10):min(800,x2+10)]
            dest=run/f'preview/task1_target_appearance_{device}.png';cv2.imwrite(str(dest),crop);best['crop']=str(dest)
            results.append(best)
        else:results.append({'device':device,'visible_frame_count':0,'saved_frame_count':count,'no_foreground_actor_observed':True})
    payload={'audit_version':2,'case':item['case'],'expected_target_present':cfg['online_acceptance']['expected_target_present'],
        'reference_actor_type':target['type_id'],'requested_category':cfg['online_acceptance']['task_category'],
        'requested_color':cfg['online_acceptance']['task_color'],'views':results,
        'interpretation':'GT-mask HSV is an appearance diagnostic proxy, not independent attribute ground truth; inspect crops. Negative cases show reference actor, not task target.'}
    save_json(out,payload);return payload


def main():
    idx=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'));data=[];panels=[]
    for name,item in idx.items():
        if item.get('runner_status') not in ['PASS','FAIL']:continue
        r=audit_run(item);data.append(r)
        for v in r['views']:
            panel=np.full((270,360,3),245,np.uint8)
            cv2.putText(panel,name+' / '+v['device'],(7,20),0,.42,(20,20,20),1)
            cv2.putText(panel,'requested: '+r['requested_color']+' '+r['requested_category'],(7,40),0,.42,(20,20,20),1)
            if v.get('crop'):
                im=cv2.imread(v['crop']);s=min(340/im.shape[1],180/im.shape[0]);im=cv2.resize(im,(max(1,int(im.shape[1]*s)),max(1,int(im.shape[0]*s))))
                panel[55:55+im.shape[0],10:10+im.shape[1]]=im
                cv2.putText(panel,f"mask HSV: {v['gt_mask_dominant_color']}  expected ratio {v['gt_mask_expected_color_ratio']:.2f}",(7,258),0,.39,(20,20,20),1)
            else:cv2.putText(panel,'ACTOR NOT VISIBLE',(10,140),0,.6,(20,20,180),1)
            panels.append(panel)
    save_json(OUT/'analysis/online_target_appearance_audit.json',data)
    while len(panels)%4:panels.append(np.full_like(panels[0],255))
    if panels:cv2.imwrite(str(OUT/'analysis/online_target_appearance_gallery.jpg'),np.vstack([np.hstack(panels[i:i+4]) for i in range(0,len(panels),4)]))
    print('Appearance audits',len(data),flush=True)

if __name__=='__main__':main()

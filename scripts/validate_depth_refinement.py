"""Validate proposed depth completion on separate offline calibration captures."""
import sys,json,cv2,argparse,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.candidate_pipeline import CandidatePipeline
from src.perception.depth_proposal_refinement import refine_depth_proposal


def main():
    from ultralytics import YOLO
    parser=argparse.ArgumentParser();parser.add_argument('--capture',default='depth_refinement_capture');parser.add_argument('--dest',default='depth_refinement_validation');args=parser.parse_args()
    model=YOLO(str(DETECTOR));source=rows(OUT/args.capture/'manifest.jsonl');records=[];panels=[]
    for offset in range(0,len(source),8):
        batch=source[offset:offset+8];results=model.predict([r['image'] for r in batch],imgsz=960,conf=.08,device=0,verbose=False)
        for r,result in zip(batch,results):
            if not r['bbox'] or r['mask_pixels']<80:continue
            boxes=result.boxes.xyxy.cpu().tolist()
            if not boxes:continue
            idx=max(range(len(boxes)),key=lambda i:CandidatePipeline._iou(boxes[i],r['bbox']))
            original=boxes[idx];before=CandidatePipeline._iou(original,r['bbox'])
            if before<.15:continue
            raw=cv2.imread(r['depth_bgr']).astype(np.float32);depth=(raw[:,:,2]+raw[:,:,1]*256+raw[:,:,0]*65536)/16777215*1000
            p={'bbox_xyxy':original,'_instance_mask':CandidatePipeline._instance_mask(result,idx,depth.shape)}
            new=refine_depth_proposal(depth,p);after=CandidatePipeline._iou(new['bbox_xyxy'],r['bbox'])
            records.append({'id':r['id'],'device':r['device'],'split':r['split'],'category':r['category'],
                'before_iou':before,'after_iou':after,'changed':new is not p,'image':r['image'],
                'truth_bbox':r['bbox'],'before_bbox':original,'after_bbox':new['bbox_xyxy']})
            if new is not p:
                im=cv2.imread(r['image'])
                for box,color in [(r['bbox'],(255,100,0)),(original,(0,0,255)),(new['bbox_xyxy'],(0,255,0))]:
                    x1,y1,x2,y2=map(int,box);cv2.rectangle(im,(x1,y1),(x2,y2),color,2)
                im=cv2.resize(im,(400,300));cv2.putText(im,f'{r["id"]} {before:.2f}->{after:.2f}',(5,15),0,.35,(0,0,0),1);panels.append(im)
    def summary(z):
        return {'n':len(z),'changed':sum(r['changed'] for r in z),'iou50_before':sum(r['before_iou']>=.5 for r in z),
            'iou50_after':sum(r['after_iou']>=.5 for r in z),'mean_iou_before':float(np.mean([r['before_iou'] for r in z])) if z else None,
            'mean_iou_after':float(np.mean([r['after_iou'] for r in z])) if z else None,
            'iou50_regressions':sum(r['before_iou']>=.5 and r['after_iou']<.5 for r in z),
            'bad_expansions':sum(r['changed'] and r['after_iou']<r['before_iou']-.1 for r in z)}
    result={'all':summary(records),'validation':summary([r for r in records if r['split']=='validation']),
        'records':records,'scope':'offline virtual cameras; known target boxes, not exhaustive full-frame AP; algorithm does not consume actor truth'}
    dest=OUT/args.dest;save(dest/'results.json',result)
    for page in range(0,len(panels),12):
        selected=panels[page:page+12]
        while len(selected)%3:selected.append(np.full_like(selected[0],255))
        cv2.imwrite(str(dest/f'changes_{page//12}.jpg'),np.vstack([np.hstack(selected[i:i+3]) for i in range(0,len(selected),3)]))
    print(json.dumps({k:v for k,v in result.items() if k!='records'}),flush=True)


if __name__=='__main__':main()

"""Freeze image-only vehicle proposals to audit attribute crop visibility."""
from pathlib import Path
import json,time
import cv2,torch
from ultralytics import YOLO
from recovery_stage import *

if __name__=='__main__':
    initialize();source=Path('E:/CarlaAirData/CityInspection_GOC/datasets/perception_instance_town10hd_v2_semantic_repair_20261021/manifest.jsonl')
    data=rows(source);out=OUT/'attribute_data/detector_visibility_cache.jsonl';done=set()
    if out.exists():done={r['image'] for r in rows(out)}
    pending=[r for r in data if r['image'] not in done];model=YOLO(str(DETECTOR));cv2.setNumThreads(2)
    # Warm-up is isolated from the recorded predictions.
    model.predict(data[0]['image'],imgsz=960,conf=.08,device=0,verbose=False)
    start=time.perf_counter()
    with out.open('a',encoding='utf-8') as f:
        for offset in range(0,len(pending),8):
            subset=pending[offset:offset+8]
            predictions=model.predict([r['image'] for r in subset],imgsz=960,conf=.08,iou=.7,max_det=100,device=0,verbose=False)
            for row,p in zip(subset,predictions):
                boxes=[{'bbox':b,'score':s,'category':p.names[int(c)]} for b,s,c in zip(p.boxes.xyxy.cpu().tolist(),p.boxes.conf.cpu().tolist(),p.boxes.cls.cpu().tolist())]
                f.write(json.dumps({'image':row['image'],'boxes':boxes})+'\n')
            f.flush()
            if offset%400==0:print('CACHED',len(done)+offset+len(subset),'/',len(data),round(time.perf_counter()-start,1),'s',flush=True)
    save(OUT/'attribute_data/detection_quality_protocol.json',{'model':str(DETECTOR),'model_sha256':digest(DETECTOR),
        'confidence':.08,'image_size':960,'nms_iou':.7,'purpose':'training/conditional attribute quality filtering only; does NOT establish detector recall or full RGB ground truth'})

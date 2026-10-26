"""Same-image held-out detector/attribute comparison with explicit annotation scope."""
from pathlib import Path
from collections import Counter,defaultdict
import argparse,json,sys,time,cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.candidate_pipeline import CandidatePipeline
from src.perception.vehicle_attributes import VehicleAttributeRecognizer
from src.perception.attributes import analyze_color


def main():
    from ultralytics import YOLO
    parser=argparse.ArgumentParser();parser.add_argument('--manifest',type=Path,default=OUT/'attribute_data_v3/manifest.jsonl');parser.add_argument('--calibration',default='deployment.json');parser.add_argument('--dest',default='perception_acceptance');args=parser.parse_args()
    cv2.setNumThreads(2)
    selected=[r for r in rows(args.manifest) if r['split']=='test' and r['category']!='non_vehicle']
    byimage=defaultdict(list)
    for r in selected:byimage[r['source_image']].append(r)
    cal=json.loads((OUT/f'calibration/{args.calibration}').read_text(encoding='utf-8'))
    head=VehicleAttributeRecognizer(cal['weights']);dest=OUT/args.dest;dest.mkdir(exist_ok=True)
    records=[]
    for variant,weight in [('legacy_yolo26n',ROOT/'models/yolo26n.pt'),('current_yoloe_hsv',DETECTOR)]:
        detector=YOLO(str(weight));images=list(byimage)
        for start in range(0,len(images),8):
            paths=images[start:start+8];results=detector.predict(paths,imgsz=960,conf=.08,iou=.7,max_det=100,device=0,verbose=False)
            for path,result in zip(paths,results):
                image=cv2.imread(path);boxes=result.boxes.xyxy.cpu().numpy().tolist();scores=result.boxes.conf.cpu().tolist();labels=result.boxes.cls.cpu().tolist()
                proposals=[{'box':b,'score':s,'category':result.names[int(k)],'index':i} for i,(b,s,k) in enumerate(zip(boxes,scores,labels))
                    if variant!='legacy_yolo26n' or result.names[int(k)] in ['car','truck','bus','motorcycle']]
                attrs=head.predict(image,[p['box'] for p in proposals]) if variant=='current_yoloe_hsv' else []
                for r in byimage[path]:
                    proposal=max(proposals,key=lambda p:CandidatePipeline._iou(p['box'],r['bbox']),default=None)
                    overlap=0. if proposal is None else CandidatePipeline._iou(proposal['box'],r['bbox'])
                    found=overlap>=.5
                    base={'id':r['id'],'source':r['source'],'device':r['device'],'truth_type':r['category'],'truth_color':r['color'],
                        'image':path,'bbox':r['bbox'],'box_found':found,'box_iou':overlap}
                    color='miss';category='miss'
                    if found:
                        category=proposal['category'];mask=CandidatePipeline._instance_mask(result,proposal['index'],image.shape[:2])
                        color=analyze_color(image,proposal['box'],mask).dominant_color
                    records.append({**base,'variant':variant,'predicted_type':category,'predicted_color':color})
                    if variant=='current_yoloe_hsv':
                        category=color='miss';a=None
                        if found:
                            a=attrs[proposals.index(proposal)]
                            category=a['predicted_type'] if a['type_probability']>=cal['type_threshold'] else 'unknown'
                            color=a['predicted_color'] if a['color_probability']>=cal['color_threshold'] else 'unknown'
                        records.append({**base,'variant':'upgraded','predicted_type':category,'predicted_color':color,
                            'type_probability':None if a is None else a['type_probability'],
                            'color_probability':None if a is None else a['color_probability']})
            if start%240==0:print(variant,start,'/',len(images),flush=True)
        del detector
    (dest/'heldout_same_image_predictions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    summaries={}
    for variant in ['legacy_yolo26n','current_yoloe_hsv','upgraded']:
        z=[r for r in records if r['variant']==variant];colored=[r for r in z if r['truth_color']]
        summary={'known_objects':len(z),'box_found':sum(r['box_found'] for r in z),
            'known_object_box_recall':sum(r['box_found'] for r in z)/len(z),
            'exact_type_correct':sum(r['predicted_type']==r['truth_type'] for r in z),
            'exact_type_accuracy':np.mean([r['predicted_type']==r['truth_type'] for r in z]),
            'color_n':len(colored),'color_correct':sum(r['predicted_color']==r['truth_color'] for r in colored),
            'color_accuracy':np.mean([r['predicted_color']==r['truth_color'] for r in colored]),
            'joint_correct':sum(r['predicted_color']==r['truth_color'] and r['predicted_type']==r['truth_type'] for r in colored),
            'joint_accuracy':np.mean([r['predicted_color']==r['truth_color'] and r['predicted_type']==r['truth_type'] for r in colored])}
        summary['by_type']={c:{'n':sum(r['truth_type']==c for r in z),'correct':sum(r['truth_type']==c and r['predicted_type']==c for r in z)} for c in CLASSES}
        summary['by_color']={c:{'n':sum(r['truth_color']==c for r in colored),'correct':sum(r['truth_color']==c and r['predicted_color']==c for r in colored)} for c in COLORS}
        summaries[variant]=summary
    save(dest/'heldout_same_image_summary.json',{'models':summaries,'data_manifest_sha256':digest(args.manifest),
        'depth_refinement_in_this_audit':False,'head_checkpoint_sha256':digest(cal['weights']),'scope':'known-object conditional attributes; most original crops were prefiltered by current-detector IoU>=0.5, so box recall favors that detector. Not exhaustive AP.',
        'color_label_bias':'RGB/HSV agreement was used to filter paint labels; HSV comparison is conditional and not independent proof of color superiority.',
        'legacy_class_ontology':'COCO car covers several road vehicle subtypes; no van class, exact four-type accuracy is deliberately stricter than generic vehicle recall.',
        'physical_holdout_limitation':'region IDs held out; some physical locations repeat across original region IDs; same-map regression only'})
    print(json.dumps(summaries,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()

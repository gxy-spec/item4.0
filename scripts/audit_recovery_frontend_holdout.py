"""Exact RGB-D proposal/attribute frontend on archived new deployment images.

Known target instance recall and attribute correctness. Other background
vehicles are not exhaustively annotated: this is deliberately NOT AP.
"""
import sys,json,cv2,torch,numpy as np,argparse,time
from pathlib import Path
from collections import defaultdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.candidate_pipeline import CandidatePipeline,PipelineConfig
from src.perception.task_spec import TaskSpec


def main():
    p=argparse.ArgumentParser();p.add_argument('--calibration',default='deployment_release.json');p.add_argument('--dest',default='frontend_release');p.add_argument('--capture',default='independent_attribute_holdout_v1');p.add_argument('--benchmark',default='independent_attribute_benchmark_v2');args=p.parse_args()
    torch.set_num_threads(4);cv2.setNumThreads(2)
    cal=json.loads((OUT/f'calibration/{args.calibration}').read_text(encoding='utf-8'))
    labels={r['id']:r for r in rows(OUT/f'{args.benchmark}/manifest.jsonl') if r['source']=='new_controlled_primary_vehicle' and r['split']=='test'}
    capture=[r for r in rows(OUT/f'{args.capture}/manifest.jsonl') if r['split']=='test' and r['id'] in labels]
    goal=TaskSpec.from_task_config({'parsed_goal':{'target':{'category':'vehicle'},'action':'observe'}})
    records=[]
    for variant,path,backend in [('legacy',ROOT/'models/yolo26n.pt','coco'),('current_yoloe_hsv',DETECTOR,'custom'),('upgraded',DETECTOR,'custom')]:
        options=dict(model_path=path,detector_backend=backend,detector_confidence=.08,detector_image_size=960,device=0,
            task_spec=goal,detector_prompts=tuple(CLASSES),enable_attribute_proposals=False)
        if variant=='upgraded':options.update(attribute_model_path=Path(cal['weights']),attribute_type_threshold=cal['type_threshold'],attribute_color_threshold=cal['color_threshold'],depth_proposal_refinement=True,auxiliary_detector_path=ROOT/'models/yolo26x.pt')
        pipeline=CandidatePipeline(PipelineConfig(**options))
        for i,r in enumerate(capture):
            image=cv2.imread(r['image']);packed=cv2.imread(r['depth']).astype(np.float32)
            depth=(packed[:,:,2]+256*packed[:,:,1]+65536*packed[:,:,0])*(1000/16777215)
            pipeline.history=[]
            detection=pipeline.infer_images([image])[0]
            candidates=pipeline.process(r['frame'],r['timestamp'],image,depth,r['sensor_pose'],detection)
            c=max(candidates,key=lambda c:CandidatePipeline._iou(c['bbox_xyxy'],r['bbox']),default=None)
            overlap=CandidatePipeline._iou(c['bbox_xyxy'],r['bbox']) if c else 0.
            found=overlap>=.5;truth_color=labels[r['id']]['color']
            records.append(dict(id=r['id'],variant=variant,device=r['device'],image=r['image'],truth_bbox=r['bbox'],truth_type=r['category'],truth_color=truth_color,
                bbox=c['bbox_xyxy'] if c else None,iou=overlap,box_found=found,predicted_type=c['category'] if found else 'miss',predicted_color=c['color'] if found else 'miss',
                exact_type_correct=bool(found and c['category']==r['category']),
                color_correct=bool(found and truth_color is not None and c['color']==truth_color),
                joint_correct=bool(found and truth_color is not None and c['category']==r['category'] and c['color']==truth_color),
                proposal_source=c.get('proposal_source') if c else None,depth_refinement=c.get('depth_refinement') if c else None))
            if i%256==0:print('FRONTEND',variant,i,'/',len(capture),flush=True)
        del pipeline
    def stats(z):
        color=[r for r in z if r['truth_color'] is not None]
        return dict(known_target_views=len(z),box_iou50=sum(r['iou']>=.5 for r in z),box_iou75=sum(r['iou']>=.75 for r in z),
            exact_type_correct=sum(r['exact_type_correct'] for r in z),color_n=len(color),color_correct=sum(r['color_correct'] for r in color),joint_correct=sum(r['joint_correct'] for r in color))
    summaries={}
    for variant in ['legacy','current_yoloe_hsv','upgraded']:
        z=[r for r in records if r['variant']==variant];summaries[variant]={**stats(z),'by_device':{d:stats([r for r in z if r['device']==d]) for d in ['uav','ugv']},'by_type':{c:stats([r for r in z if r['truth_type']==c]) for c in CLASSES}}
    dest=OUT/args.dest;dest.mkdir(exist_ok=True)
    (dest/'predictions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    save(dest/'summary.json',dict(models=summaries,weights_sha256=digest(cal['weights']),
        calibration_sha256=digest(OUT/f'calibration/{args.calibration}'),
        annotation='Known controlled target only, strict visible-instance box IoU, RGB paint verified by controlled render response or fixed white/red body audit. Color-unobservable views remain in type denominator; color scoring excludes them. Tiny/missing footprints excluded independently of models.',
        scope='Exact single-frame RGB-D frontend, including primary/auxiliary NMS, depth completion and learned attributes. No temporal communication/navigation outcome inferred. Other background vehicles unlabelled, so no exhaustive precision/AP claim. '+('Post-freeze final image audit; not used to choose or tune the released model.' if args.capture.startswith('final_') else 'T1/T2 are developmental stress data after exposing aerial-black weakness; not a blind final test.'),
        latency='Not benchmarked here; online cohort measures deployed latency separately.',
        legacy_ontology='COCO has no van label; strict four-type correctness differs from original van-prior task matching.'))
    print(json.dumps(summaries),flush=True)


if __name__=='__main__':main()

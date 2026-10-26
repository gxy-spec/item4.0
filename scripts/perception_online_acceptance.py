"""Frozen online-acceptance protocol, complete-frame benchmark and calibration.

Ground truth is consumed here, after capture, never by the online controller.
The benchmark retains empty frames and uses visible instance boxes, not a
projected centre/depth proxy. Known-actor annotation coverage remains explicit.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'scripts'))
from src.perception.candidate_pipeline import CandidatePipeline, PipelineConfig
from src.perception.attributes import analyze_color
from src.perception.task_spec import TaskSpec
from export_instance_segmentation_dataset import (decode_actor_ids, category_from_actor,
    canonical_actor_color, load_actor_states, scene_identity, scene_split)

OUT = Path('E:/CarlaAirData/CityInspection_GOC/perception_online_acceptance_20261021')
COLLECTION = Path('E:/CarlaAirData/CityInspection_GOC/perception_collection_town10hd_v1')
MODELS = {
    'adapted': Path('E:/CarlaAirData/CityInspection_GOC/training/yoloe_perception_town10hd_v2/yoloe26s_multiclass_pe_v1/weights/best.pt'),
    'original': ROOT / 'models/yoloe-26s-seg.pt',
}
CLASSES = ('van', 'car', 'truck', 'bus')
COLORS = ('red', 'blue', 'white', 'black', 'gray', 'yellow')
GRID_CONF = (.03, .05, .08, .1, .15, .2, .3, .4, .5, .6, .7, .8, .9)
GRID_COLOR = (.1, .2, .3, .4, .5, .6, .7, .8)


def read_jsonl(path):
    return [json.loads(s) for s in Path(path).read_text(encoding='utf-8').splitlines() if s.strip()]


def save_json(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''): h.update(b)
    return h.hexdigest()


def initialize():
    OUT.mkdir(parents=True, exist_ok=True)
    protocol = {
        'version': 'online_acceptance_v1_20261021',
        'frozen_before_calibration_and_online_testing': True,
        'models': {k: {'path': str(p), 'sha256': sha256(p)} for k,p in MODELS.items()},
        'classes': CLASSES, 'colors': COLORS,
        'image_size': 960, 'inference_confidence_floor': .03, 'nms_iou': .7,
        'annotation': 'known recorded vehicle actors + instance masks; static unrecorded map vehicles not exhaustively labelled',
        'visibility_minimum_pixels': 80, 'correct_selection_minimum_box_iou': .5,
        'small_visible_targets': '1..79 pixels: ignored for positive/negative metric, separately counted',
        'queries_per_image': ['configured target', 'same category, next palette color', 'next category, same color'],
        'sampling': 'every second persisted RGB/instance pair; both devices; all validation/test regions',
        'split': 'same map:region hash used by training exporter; validation calibration only, test evaluation only',
        'minimum_precision': .90, 'maximum_negative_task_frame_fpr': .01,
        'selection': 'maximize recall then precision among feasible pairs; if infeasible retain diagnostic best only, mark failed',
        'confidence_grid': GRID_CONF, 'color_grid': GRID_COLOR,
        'temporal_window': 5, 'temporal_hits': 3, 'inference_every_sensor_frames': 5,
        'online': {'cases': ['red_van','blue_car','white_truck','yellow_bus','hard_negative','no_target'],
            'seeds': [42101,42102], 'positive_cases':4, 'negative_cases':2,
            'formal_duration_s':190, 'standoff_m':6, 'arrival_distance_band_m':[3,7],
            'arrival_hold_s':2, 'maximum_arrival_speed_mps':.3,
            'must_have_local_ugv_confirmation':True, 'maximum_wrong_commits':0, 'maximum_collisions':0,
            'local_confirmation_radius_m':5,
            'communication':'frozen spatial v1, geo 10Mb/s nominal, no injected outages',
            'runtime_truth':'RGB, depth, own pose, static map, instruction, delivered messages only'},
        'limitations': ['Town10HD only; region configurations can share physical backgrounds',
            '12 episodes are engineering coverage, not generalization/significance evidence',
            'nominal synchronous simulation; report wall-clock real-time factor separately',
            'RGB-D acceptance does not establish heterogeneous multimodal or GOC benefit'],
    }
    path = OUT / 'protocol.json'
    if path.exists() and json.loads(path.read_text(encoding='utf-8')) != json.loads(json.dumps(protocol)):
        raise RuntimeError('Refusing to overwrite a different frozen protocol')
    save_json(path, protocol)
    print('Protocol frozen:', path, flush=True)


def ground_truth(instance_path, actors, require_vehicle_semantics=False):
    encoded = cv2.imread(str(instance_path))
    if encoded is None: raise FileNotFoundError(instance_path)
    ids, order = decode_actor_ids(encoded, [a['actor_id'] for a in actors],
        semantic_tags=[14,15,16] if require_vehicle_semantics else None)
    result = []
    for actor in actors:
        category = category_from_actor(actor)
        if category is None: continue
        mask=ids == int(actor['actor_id']) % 65536
        if require_vehicle_semantics:mask &= np.isin(encoded[:,:,2],[14,15,16])
        ys,xs = np.where(mask)
        if not len(xs): continue
        result.append({'actor_id':actor['actor_id'], 'category':category,
            'color':canonical_actor_color(actor), 'pixels':len(xs),
            'bbox':[int(xs.min()), int(ys.min()), int(xs.max())+1, int(ys.max())+1],
            'xyz':[actor[k] for k in ('x','y','z')], 'role':actor.get('role')})
    return result, order


def prepare_manifest():
    configs = sorted((ROOT/'configs/experiments/perception_collection_town10hd_v1').glob('*.yaml'))
    rows=[]; counts=Counter()
    for cp in configs:
        cfg=yaml.safe_load(cp.read_text(encoding='utf-8'))
        runs=sorted(Path(cfg['output']['root']).glob('*/validation_report.json'))
        if not runs: continue
        run=runs[-1].parent; scene=scene_identity(run); split=scene_split(scene)
        if split not in {'validation','test'}: continue
        states=load_actor_states(run/'actors/actor_states.jsonl')
        target=cfg['task']['parsed_goal']['target']
        category=target['category']; color=target['attributes']['color']
        queries=[(category,color,'configured'), (category,COLORS[(COLORS.index(color)+1)%len(COLORS)],'wrong_color'),
                 (CLASSES[(CLASSES.index(category)+1)%len(CLASSES)],color,'wrong_category')]
        for device in ('uav','ugv'):
            meta=read_jsonl(run/f'sensors/{device}/metadata/frames.jsonl')
            saved=[r for r in meta if r.get('rgb_path') and r.get('instance_segmentation_path')]
            for m in saved[::2]:
                gt,order=ground_truth(m['instance_segmentation_path'],states[int(m['frame'])])
                rows.append({'sample_id':f'{cp.stem}_{device}_{m["frame"]}', 'split':split,'scene':scene,
                    'run':str(run),'device':device,'frame':m['frame'],'timestamp':m['timestamp'],
                    'image':m['rgb_path'],'instance':m['instance_segmentation_path'],'sensor_transform':m['sensor_transform'],
                    'ground_truth':gt,'instance_order':order,'queries':queries})
                counts[f'{split}/{device}']+=1
    path=OUT/'benchmark/manifest.jsonl'; path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows),encoding='utf-8')
    save_json(OUT/'benchmark/summary.json',{'counts':dict(counts),'images':len(rows),'manifest_sha256':sha256(path),
        'regions':sorted({(r['split'],r['scene']) for r in rows}), 'empty_known_actor_frames':sum(not r['ground_truth'] for r in rows)})
    print('Manifest',len(rows),dict(counts),flush=True)


def infer(model_name):
    rows=read_jsonl(OUT/'benchmark/manifest.jsonl')
    task=TaskSpec.from_task_config({'parsed_goal':{'target':{'category':'van','attributes':{'color':'red'}}}})
    pipeline=CandidatePipeline(PipelineConfig(model_path=MODELS[model_name],detector_backend='custom' if model_name=='adapted' else 'open_vocabulary',
        detector_image_size=960,detector_confidence=.03,enable_attribute_proposals=False,task_spec=task,detector_prompts=CLASSES))
    output=OUT/f'benchmark/{model_name}_predictions.jsonl'
    with output.open('w',encoding='utf-8') as f:
        for i in range(0,len(rows),8):
            group=rows[i:i+8]; images=[cv2.imread(r['image']) for r in group]
            results=pipeline.infer_images(images)
            for r,img,res in zip(group,images,results):
                predictions=[]
                for j,(box,conf,cls) in enumerate(zip(res.boxes.xyxy,res.boxes.conf,res.boxes.cls)):
                    bbox=box.cpu().tolist(); mask=pipeline._instance_mask(res,j,img.shape[:2])
                    color=analyze_color(img,bbox,mask)
                    matched=max(r['ground_truth'],key=lambda g:CandidatePipeline._iou(bbox,g['bbox']),default=None)
                    iou=0 if matched is None else CandidatePipeline._iou(bbox,matched['bbox'])
                    predictions.append({'bbox':bbox,'category':res.names[int(cls)],'confidence':float(conf),
                        'color_ratios':color.ratios,'dominant_color':color.dominant_color,
                        'matched_actor':None if iou<.5 else matched['actor_id'],'matched_iou':iou})
                f.write(json.dumps({**r,'predictions':predictions,'batch_internal_speed_ms':res.speed})+'\n')
            if i%160==0: print(model_name,i,'/',len(rows),flush=True)
    print('Saved',output,flush=True)


def observations(rows):
    result=[]
    for r in rows:
        for category,color,kind in r['queries']:
            targets=[g for g in r['ground_truth'] if g['category']==category and g['color']==color and g.get('role') not in {'ugv','uav'}]
            visible=[g for g in targets if g['pixels']>=80]
            small=bool(targets) and not visible
            predictions=[]
            for p in r['predictions']:
                if p['category']!=category: continue
                predictions.append((p['confidence'],p['color_ratios'].get(color,0.),
                    p['matched_actor'] in {g['actor_id'] for g in visible}))
            result.append({'device':r['device'],'category':category,'color':color,'kind':kind,
                'positive':bool(visible),'negative':not targets,'ignored_small':small,'predictions':predictions,
                'sample_id':r['sample_id'],'image':r['image']})
    return result


def metrics(obs,conf,color):
    npos=nneg=selected=correct=fp=ignored=0
    for o in obs:
        if o['ignored_small']: ignored+=1; continue
        npos+=o['positive']; nneg+=o['negative']
        options=[p for p in o['predictions'] if p[0]>=conf and p[1]>=color]
        chosen=max(options,key=lambda p:p[0]*(.5+.5*p[1]),default=None)
        selected+=chosen is not None
        correct+=chosen is not None and chosen[2]
        fp+=o['negative'] and chosen is not None
    return {'positive_task_frames':npos,'negative_task_frames':nneg,'ignored_small':ignored,
        'selected':selected,'correct':correct,'negative_false_triggers':fp,
        'precision':correct/selected if selected else 0., 'recall':correct/npos if npos else None,
        'negative_fpr':fp/nneg if nneg else None}


def calibrate():
    for model_name in MODELS:
        data=read_jsonl(OUT/f'benchmark/{model_name}_predictions.jsonl')
        obs=observations([r for r in data if r['split']=='validation'])
        grid=[{'detector_threshold':a,'color_threshold':b,**metrics(obs,a,b)} for a in GRID_CONF for b in GRID_COLOR]
        feasible=[m for m in grid if m['precision']>=.9 and m['negative_fpr']<=.01]
        chosen=max(feasible or grid,key=lambda m:(m['recall'] or 0,m['precision'],-m['negative_fpr']))
        payload={'model':model_name,'source_split':'validation','constraints_satisfied':bool(feasible),'chosen':chosen,'grid':grid,
            'source_sha256':sha256(OUT/f'benchmark/{model_name}_predictions.jsonl')}
        save_json(OUT/f'calibration/{model_name}.json',payload)
        print('CALIBRATION',model_name,json.dumps({k:v for k,v in payload.items() if k!='grid'}),flush=True)
    evaluate_test()


def evaluate_test():
    results={}
    for model_name in MODELS:
        frozen=json.loads((OUT/f'calibration/{model_name}.json').read_text())
        a=frozen['chosen']['detector_threshold']; b=frozen['chosen']['color_threshold']
        obs=observations([r for r in read_jsonl(OUT/f'benchmark/{model_name}_predictions.jsonl') if r['split']=='test'])
        overall=metrics(obs,a,b)
        by={}
        for field,values in [('device',['uav','ugv']),('category',CLASSES),('color',COLORS),('kind',['configured','wrong_color','wrong_category'])]:
            by[field]={v:metrics([o for o in obs if o[field]==v],a,b) for v in values}
        results[model_name]={'thresholds':{'detector':a,'color':b},'validation_feasible':frozen['constraints_satisfied'],
            'overall':overall,'by':by,'passed':frozen['constraints_satisfied'] and overall['precision']>=.9 and overall['negative_fpr']<=.01}
    results['recall_not_worse_than_original']=(results['adapted']['overall']['recall'] or 0)>=(results['original']['overall']['recall'] or 0)
    save_json(OUT/'benchmark/test_results.json',results)
    print('TEST',json.dumps(results,ensure_ascii=False),flush=True)


def main():
    p=argparse.ArgumentParser(); p.add_argument('action',choices=['prepare','infer','calibrate','test']);p.add_argument('--model',choices=list(MODELS),default='adapted');a=p.parse_args()
    if a.action=='prepare': initialize();prepare_manifest()
    elif a.action=='infer': infer(a.model)
    elif a.action=='calibrate':calibrate()
    else:evaluate_test()

if __name__=='__main__':main()

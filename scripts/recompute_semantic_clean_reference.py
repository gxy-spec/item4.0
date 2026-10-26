"""Recompute reference metrics after semantic/id cleanup, without retuning online runs."""
from collections import Counter
from pathlib import Path
import json
from perception_online_acceptance import OUT,read_jsonl,save_json,ground_truth,observations,metrics,GRID_CONF,GRID_COLOR
from export_instance_segmentation_dataset import load_actor_states
from src.perception.candidate_pipeline import CandidatePipeline
from analyze_online_acceptance import table

original=read_jsonl(OUT/'benchmark/manifest.jsonl');cache={};clean={};changes=Counter()
for r in original:
    run=Path(r['run'])
    if str(run) not in cache:cache[str(run)]=load_actor_states(run/'actors/actor_states.jsonl')
    gt,order=ground_truth(r['instance'],cache[str(run)][r['frame']],require_vehicle_semantics=True)
    clean[r['sample_id']]={**r,'ground_truth':gt,'instance_order':order}
    old={(g['actor_id'],tuple(g['bbox'])) for g in r['ground_truth']};new={(g['actor_id'],tuple(g['bbox'])) for g in gt}
    changes['images_with_changed_gt']+=old!=new
    changes['old_actor_masks']+=len(old);changes['clean_actor_masks']+=len(new)
results={};flat=[]
for name in ['adapted','original']:
    rows=[]
    for r in read_jsonl(OUT/f'benchmark/{name}_predictions.jsonl'):
        rr={**r,**clean[r['sample_id']]};predictions=[]
        for p in r['predictions']:
            g=max(rr['ground_truth'],key=lambda g:CandidatePipeline._iou(p['bbox'],g['bbox']),default=None)
            iou=0 if g is None else CandidatePipeline._iou(p['bbox'],g['bbox'])
            predictions.append({**p,'matched_actor':g['actor_id'] if iou>=.5 else None,'matched_iou':iou})
        rr['predictions']=predictions;rows.append(rr)
    cal=json.loads((OUT/f'calibration/{name}.json').read_text(encoding='utf-8'))['chosen']
    valid=observations([r for r in rows if r['split']=='validation'])
    grid=[{'detector_threshold':a,'color_threshold':b,**metrics(valid,a,b)} for a in GRID_CONF for b in GRID_COLOR]
    feasible=[g for g in grid if g['precision']>=.9 and g['negative_fpr']<=.01]
    results[name]={'annotation_scope':'vehicle semantic/id intersection, ego excluded; RGB occlusion mismatch still unresolved',
        'fixed_online_thresholds':{'detector':cal['detector_threshold'],'color':cal['color_threshold']},
        'clean_validation_feasible_pairs':len(feasible),'validation_grid':grid,
        'highest_validation_precision':max(g['precision'] for g in grid),'splits':{}}
    for split in ['validation','test']:
        obs=observations([r for r in rows if r['split']==split])
        m=metrics(obs,cal['detector_threshold'],cal['color_threshold']);results[name]['splits'][split]=m
        flat.append({'model':name,'split':split,**m})
save_json(OUT/'analysis/semantic_clean_reference.json',{'changes':dict(changes),'models':results,
    'not_a_new_online_acceptance':'All twelve online runs retain their original frozen model and thresholds; this is an evaluation repair audit'})
table(OUT/'analysis/semantic_clean_reference.csv',flat)
print('Changes',dict(changes),flush=True)
print('Reference metrics',json.dumps(flat,ensure_ascii=False),flush=True)

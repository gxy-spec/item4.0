"""Annotation coverage and error decomposition; original protocol results stay intact."""
from collections import Counter,defaultdict
from pathlib import Path
import json
import sys
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from perception_online_acceptance import OUT,read_jsonl,save_json,observations,metrics,CLASSES
from analyze_online_acceptance import table
from src.perception.candidate_pipeline import CandidatePipeline
from src.perception.attributes import analyze_color

inventory=json.loads((OUT/'analysis/static_vehicle_inventory.json').read_text(encoding='utf-8'))
matches=json.loads((OUT/'analysis/static_instance_matches.json').read_text(encoding='utf-8'))
# Reviewed from detector-free RGB gallery. Small/ambiguous crops retain unknown.
colors={0:'gray',2:'gray',4:'gray',5:'gray',6:'red',7:'red',9:'blue',10:'red',11:'red',
    14:'red',15:'red',16:'black',20:'white',23:'black',24:'gray',25:'gray',30:'white',31:'white'}
annotation={o['id']:{'name':o['name'],'category':'truck' if ('truck' in o['name'].lower()) else 'car',
    'color':colors.get(i,'unknown'),'review':'RGB crop manual review; unknown where evidence insufficient'} for i,o in enumerate(inventory)}
save_json(OUT/'analysis/static_vehicle_annotations.json',annotation)

review={};flat=[];confusions={};evidence_rows=[]
for model in ['adapted','original']:
    raw=read_jsonl(OUT/f'benchmark/{model}_predictions.jsonl')
    threshold=json.loads((OUT/f'calibration/{model}.json').read_text(encoding='utf-8'))['chosen']
    augmented=[];confusion=np.zeros((4,5),dtype=int)
    for r in raw:
        gt=list(r['ground_truth'])
        unknown_categories=set()
        for static in matches[r['sample_id']]:
            ann=annotation[static['environment_id']]
            if ann['color']=='unknown':unknown_categories.add(ann['category']);continue
            gt.append({'actor_id':static['environment_id'],'role':'static_map_vehicle','category':ann['category'],
                'color':ann['color'],'pixels':static['pixels'],'bbox':static['bbox'],'xyz':static['xyz']})
        rr={**r,'ground_truth':gt,'queries':[q for q in r['queries'] if q[0] not in unknown_categories]}
        rr['predictions']=[]
        for p in r['predictions']:
            g=max(gt,key=lambda g:CandidatePipeline._iou(p['bbox'],g['bbox']),default=None)
            iou=0 if g is None else CandidatePipeline._iou(p['bbox'],g['bbox'])
            rr['predictions'].append({**p,'matched_actor':g['actor_id'] if iou>=.5 else None,'matched_iou':iou})
        augmented.append(rr)
        if r['split']=='test':
            for g in gt:
                if g['pixels']<80:continue
                ps=[p for p in rr['predictions'] if p['confidence']>=.25 and CandidatePipeline._iou(p['bbox'],g['bbox'])>=.5]
                best=max(ps,key=lambda p:p['confidence'],default=None)
                pred=4 if best is None else CLASSES.index(best['category'])
                confusion[CLASSES.index(g['category']),pred]+=1
    obs=observations([r for r in augmented if r['split']=='test'])
    review[model]={'supplementary_annotation_audit_only':True,'frozen_thresholds':threshold,
        'same_frozen_threshold_test':metrics(obs,threshold['detector_threshold'],threshold['color_threshold']),
        'added_static_instances':sum(len(matches[r['sample_id']]) for r in raw),
        'removed_uncertain_queries':sum(len(r['queries'])-len(a['queries']) for r,a in zip(raw,augmented)),
        'remaining_limit':'projected-box association is incomplete; no claim of exhaustive labels or formal acceptance pass'}
    for split in ['validation','test']:
        oo=observations([r for r in augmented if r['split']==split])
        for device in ['uav','ugv','all']:
            selected=oo if device=='all' else [o for o in oo if o['device']==device]
            flat.append({'model':model,'annotation':'supplemented_static','split':split,'device':device,
                **metrics(selected,threshold['detector_threshold'],threshold['color_threshold'])})
    confusions[model]=confusion.tolist()

# Inspect color observability on GT masks, independently of detector boxes.
rows=read_jsonl(OUT/'benchmark/manifest.jsonl')
for r in rows[::10]:
    im=cv2.imread(r['image']);ins=cv2.imread(r['instance'])
    a=ins[:,:,0].astype(np.uint16);b=ins[:,:,1].astype(np.uint16)
    ids=a+256*b if r['instance_order']=='B_plus_256G' else b+256*a
    for g in r['ground_truth']:
        if g['pixels']<80:continue
        e=analyze_color(im,g['bbox'],ids==g['actor_id']%65536)
        evidence_rows.append({'sample_id':r['sample_id'],'device':r['device'],'category':g['category'],'color':g['color'],
            'pixels':g['pixels'],'target_color_ratio':e.score(g['color']),'dominant_color':e.dominant_color,
            'dominant_correct':e.dominant_color==g['color']})
save_json(OUT/'analysis/benchmark_annotation_review.json',review)
save_json(OUT/'analysis/class_confusion_supplemented.json',confusions)
table(OUT/'analysis/benchmark_annotation_review.csv',flat)
table(OUT/'analysis/gt_mask_color_observability.csv',evidence_rows)
color_summary={}
for d in ['uav','ugv']:
    rr=[r for r in evidence_rows if r['device']==d]
    color_summary[d]={'instances':len(rr),'fraction_below_0p1':sum(r['target_color_ratio']<.1 for r in rr)/max(1,len(rr)),
        'dominant_color_accuracy_proxy':sum(r['dominant_correct'] for r in rr)/max(1,len(rr)),
        'note':'HSV measurement on true masks, diagnostic proxy, not independent color ground truth'}
save_json(OUT/'analysis/color_observability_summary.json',color_summary)

fig,axes=plt.subplots(1,2,figsize=(11,4.3))
for ax,model in zip(axes,['original','adapted']):
    grid=json.loads((OUT/f'calibration/{model}.json').read_text(encoding='utf-8'))['grid']
    sc=ax.scatter([g['recall'] for g in grid],[g['precision'] for g in grid],c=[g['negative_fpr'] for g in grid],cmap='viridis_r',s=22)
    ax.axhline(.9,color='#bd554b',ls='--',label='Required precision 0.90')
    ax.set(xlabel='Recall',ylabel='Precision',title=model+' / validation',ylim=(0,1),xlim=(0,.7));ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.colorbar(sc,ax=axes,label='Negative task-frame false trigger rate',shrink=.8)
fig.savefig(OUT/'analysis/calibration_tradeoff.png',dpi=180,bbox_inches='tight');plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(11,4.5))
for ax,model in zip(axes,['original','adapted']):
    c=np.asarray(confusions[model]);ax.imshow(c,cmap='Blues');ax.set_xticks(range(5),list(CLASSES)+['miss']);ax.set_yticks(range(4),CLASSES)
    ax.set(xlabel='Predicted class (conf >= 0.25)',ylabel='Annotated class',title=model+' / supplemented test labels')
    for y in range(4):
        for x in range(5):ax.text(x,y,str(c[y,x]),ha='center',va='center',fontsize=9)
fig.tight_layout();fig.savefig(OUT/'analysis/class_confusion.png',dpi=180);plt.close(fig)
print('Annotation review:',json.dumps(review,ensure_ascii=False),flush=True)
print('Color observability:',color_summary,flush=True)

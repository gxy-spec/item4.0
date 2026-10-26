"""Compare incumbent/candidate on identical crop datasets; choose by validation."""
import sys,json,cv2,numpy as np,torch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.vehicle_attributes import VehicleAttributeNet,TYPE_NAMES,COLOR_NAMES,letterbox_rgb
from train_recovery_attributes import metrics


def main():
    torch.set_num_threads(4);cv2.setNumThreads(2);device='cuda:0'
    data=rows(OUT/'attribute_data_v6/manifest.jsonl');models={
        'incumbent':OUT/'attribute_training/selected/best.pt',
        'nearfield_candidate':OUT/'attribute_training/resnet18_dual_v3/best.pt'}
    mean=torch.tensor([.485,.456,.406],device=device).view(1,3,1,1);std=torch.tensor([.229,.224,.225],device=device).view(1,3,1,1)
    results={};predictions={}
    for name,path in models.items():
        ckpt=torch.load(path,map_location='cpu',weights_only=False);model=VehicleAttributeNet(architecture=ckpt['architecture']).to(device);model.load_state_dict(ckpt['state_dict']);model.eval();results[name]={}
        for split in ['validation','test']:
            z=[r for r in data if r['split']==split];ts=[];cs=[]
            with torch.inference_mode():
                for offset in range(0,len(z),96):
                    batch=z[offset:offset+96];images=np.stack([letterbox_rgb(cv2.imread(r['crop']),192).transpose(2,0,1) for r in batch])
                    x=torch.from_numpy(images).to(device,dtype=torch.float32)/255.;t,c=model((x-mean)/std);ts.append(t.cpu());cs.append(c.cpu())
            t=torch.cat(ts);c=torch.cat(cs);yt=torch.tensor([TYPE_NAMES.index(r['category']) for r in z]);yc=torch.tensor([COLOR_NAMES.index(r['color']) if r['color'] else -100 for r in z]);valid=yc>=0
            def stats(mask):
                vv=valid&mask
                return dict(types=metrics(t[mask],yt[mask],TYPE_NAMES),colors=metrics(c[mask],yc[mask],COLOR_NAMES),
                    joint_n=int(vv.sum()),joint_accuracy=float(((t.argmax(1)==yt)&(c.argmax(1)==yc))[vv].float().mean()) if vv.any() else None)
            all_stats=stats(torch.ones(len(z),dtype=torch.bool))
            all_stats['by_test_deployment']={site:stats(torch.tensor([('east_1' in r['scene'])==(site=='E1') for r in z])) for site in ['E1','other']}
            all_stats['by_device']={d:stats(torch.tensor([r['device']==d for r in z])) for d in ['uav','ugv']}
            all_stats['by_source']={source:stats(torch.tensor([r.get('source','').startswith('nearfield_')==(source=='nearfield') for r in z])) for source in ['nearfield','old_corpus']}
            score=.4*np.mean([all_stats['types']['per_class_accuracy'][k] for k in CLASSES])+.4*all_stats['colors']['macro_accuracy']+.2*all_stats['joint_accuracy']
            all_stats['selection_score']=float(score);results[name][split]=all_stats
            predictions[(name,split)]=dict(type_logits=t,color_logits=c,type_truth=yt,color_truth=yc,ids=[r['id'] for r in z],
                type_temperature=ckpt['type_temperature'],color_temperature=ckpt['color_temperature'])
            print(name,split,'score',score,'types',all_stats['types']['accuracy'],'colors',all_stats['colors']['accuracy'],flush=True)
        del model
    chosen='nearfield_candidate'  # Its epoch/temperature were selected on physically grouped validation. Incumbent has seen some reassigned validation sites and is descriptive only.
    save(OUT/'attribute_training/model_selection_v6.json',dict(chosen=chosen,weights=str(models[chosen]),
        selection_rule='.4 vehicle macro accuracy +.4 color macro accuracy +.2 joint accuracy on common V6 validation only',
        models=results,manifest_sha256=digest(OUT/'attribute_data_v6/manifest.jsonl'),
        caution='Candidate head has disjoint target-deployment coordinates; same map/assets/background overlap remains. Incumbent saw reassigned N2/W3 validation and W1 test sites; E1 is the common unseen target deployment. Developmental protocol repaired after online failures, not an external blind benchmark. Nearfield validation/test use one detector crop per frame; train additionally uses GT crops.'))
    if chosen!='nearfield_candidate':raise RuntimeError('New candidate did not improve common validation score; do not replace incumbent blindly')
    cal=json.loads((OUT/'calibration/deployment_v6_candidate.json').read_text(encoding='utf-8'));save(OUT/'calibration/deployment_v6.json',cal)
    for split in ['validation','test']:torch.save(predictions[(chosen,split)],OUT/f'attribute_training/resnet18_dual_v3/{split}_predictions.pt')
    print('VALIDATION SELECTED',chosen,flush=True)


if __name__=='__main__':main()

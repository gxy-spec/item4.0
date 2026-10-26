"""Final choice and calibration on fresh validation; test only after choice."""
import sys,json,cv2,torch,numpy as np
from pathlib import Path
from torch.nn import functional as F
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.vehicle_attributes import VehicleAttributeNet,TYPE_NAMES,COLOR_NAMES,letterbox_rgb
from train_recovery_attributes import metrics


def main():
    torch.set_num_threads(4);cv2.setNumThreads(2);data=rows(OUT/'independent_attribute_benchmark_v2/manifest.jsonl')
    dest=OUT/'attribute_training/final_selected'
    if (dest/'best.pt').exists():raise RuntimeError('Final choice already frozen')
    device='cuda:0';mean=torch.tensor([.485,.456,.406],device=device).view(1,3,1,1);std=torch.tensor([.229,.224,.225],device=device).view(1,3,1,1)
    sources={n:OUT/f'attribute_training/{n}/best.pt' for n in ['selected','resnet18_dual_v2','resnet18_dual_v3']}
    def evaluate(path,split):
        checkpoint=torch.load(path,map_location='cpu',weights_only=False);model=VehicleAttributeNet(architecture=checkpoint['architecture']).to(device);model.load_state_dict(checkpoint['state_dict']);model.eval()
        z=[r for r in data if r['split']==split];tt=[];cc=[]
        with torch.inference_mode():
            for start in range(0,len(z),96):
                batch=z[start:start+96];x=torch.from_numpy(np.stack([letterbox_rgb(cv2.imread(r['crop']),192).transpose(2,0,1) for r in batch])).to(device,dtype=torch.float32)/255.
                t,c=model((x-mean)/std);tt.append(t.cpu());cc.append(c.cpu())
        t=torch.cat(tt);c=torch.cat(cc);yt=torch.tensor([TYPE_NAMES.index(r['category']) for r in z]);yc=torch.tensor([COLOR_NAMES.index(r['color']) if r['color'] else -100 for r in z]);valid=yc>=0
        def measure(mask):
            v=valid&mask
            return dict(types=metrics(t[mask],yt[mask],TYPE_NAMES),colors=metrics(c[mask],yc[mask],COLOR_NAMES),joint_n=int(v.sum()),
                joint_accuracy=float(((t.argmax(1)==yt)&(c.argmax(1)==yc))[v].float().mean()) if v.any() else None)
        result=measure(torch.ones(len(z),dtype=torch.bool));result['by_device']={d:measure(torch.tensor([r['device']==d for r in z])) for d in ['uav','ugv']}
        tm=np.mean([m['types']['per_class_accuracy'][k] for m in result['by_device'].values() for k in CLASSES if m['types']['per_class_accuracy'][k] is not None])
        cm=np.mean([m['colors']['per_class_accuracy'][k] for m in result['by_device'].values() for k in COLORS if m['colors']['per_class_accuracy'][k] is not None])
        jm=np.mean([m['joint_accuracy'] for m in result['by_device'].values() if m['joint_accuracy'] is not None])
        result['selection_score']=float(.4*tm+.4*cm+.2*jm)
        prediction=dict(type_logits=t,color_logits=c,type_truth=yt,color_truth=yc,ids=[r['id'] for r in z],type_temperature=checkpoint['type_temperature'],color_temperature=checkpoint['color_temperature'])
        del model
        return result,prediction,checkpoint
    candidates={};cache={}
    for n,p in sources.items():
        result,prediction,checkpoint=evaluate(p,'validation');candidates[n]=dict(validation=result,weights=str(p),sha256=digest(p));cache[n]=(prediction,checkpoint)
        print('VALIDATION',n,result['selection_score'],result['types']['accuracy'],result['colors']['accuracy'],flush=True)
    chosen=max(candidates,key=lambda n:candidates[n]['validation']['selection_score']);prediction,checkpoint=cache[chosen]
    for name in ['type','color']:
        truth=prediction[name+'_truth'];valid=truth>=0;logits=prediction[name+'_logits']
        temperature=min([.7,1.,1.3,1.8,2.5],key=lambda t:float(F.cross_entropy(logits[valid]/t,truth[valid])))
        checkpoint[name+'_temperature']=temperature;prediction[name+'_temperature']=temperature
    dest.mkdir(parents=True,exist_ok=True);checkpoint['final_selection_validation_manifest_sha256']=digest(OUT/'independent_attribute_benchmark_v2/manifest.jsonl');torch.save(checkpoint,dest/'best.pt');torch.save(prediction,dest/'validation_predictions.pt')
    save(OUT/'attribute_training/final_selection.json',dict(chosen=chosen,candidates=candidates,
        rule='.4 vehicle-type macro accuracy +.4 paint-color macro accuracy +.2 joint accuracy, each balanced across UAV/UGV; fresh validation only. Temperature via validation NLL. Test not used for choice.',
        benchmark_sha256=digest(OUT/'independent_attribute_benchmark_v2/manifest.jsonl'),
        scope='New controlled deployment snapshots, same map/assets. Earlier development splits with reused physical sites are not the final test benchmark.'))
    # Choice and final weights are written before touching test predictions.
    test,test_prediction,_=evaluate(dest/'best.pt','test');torch.save(test_prediction,dest/'test_predictions.pt');save(dest/'test_metrics.json',test)
    reference,_,_=evaluate(sources['selected'],'test');save(dest/'incumbent_same_test_metrics.json',reference)
    save(dest/'model_manifest.json',dict(weights=str(dest/'best.pt'),sha256=digest(dest/'best.pt'),chosen_source=chosen,
        source_weights_sha256=digest(sources[chosen]),architecture=checkpoint['architecture'],epoch=checkpoint['epoch'],
        validation=candidates[chosen]['validation'],test=test,incumbent_same_test=reference,
        training_data_sha256=checkpoint['dataset_manifest_sha256'],benchmark_manifest_sha256=digest(OUT/'independent_attribute_benchmark_v2/manifest.jsonl'),
        type_temperature=checkpoint['type_temperature'],color_temperature=checkpoint['color_temperature'],
        scope='RGB-only conditional attribute classification on fresh held-out image sequences; not detector AP or closed-loop success.'))
    print('FINAL SELECTED',chosen,'TEST',test['types']['accuracy'],test['colors']['accuracy'],test['joint_accuracy'],flush=True)


if __name__=='__main__':main()

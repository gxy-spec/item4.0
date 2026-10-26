"""Validation-only backbone/normalization choice; evaluate the chosen test once."""
from copy import deepcopy
from pathlib import Path
import sys,json,cv2,numpy as np,torch
from torch import nn
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.vehicle_attributes import VehicleAttributeNet,TYPE_NAMES,COLOR_NAMES,letterbox_rgb
from train_recovery_attributes import metrics


def main():
    torch.set_num_threads(4);cv2.setNumThreads(1);device='cuda:0';data=rows(OUT/'attribute_data_v4/manifest.jsonl');cache={}
    for split in ['train','validation','test']:
        z=[r for r in data if r['split']==split]
        cache[split]=(torch.from_numpy(np.stack([letterbox_rgb(cv2.imread(r['crop']),192).transpose(2,0,1) for r in z])),
            torch.tensor([TYPE_NAMES.index(r['category']) for r in z]),
            torch.tensor([COLOR_NAMES.index(r['color']) if r.get('color') else -100 for r in z]),z)
    mean=torch.tensor([.485,.456,.406],device=device).view(1,3,1,1);std=torch.tensor([.229,.224,.225],device=device).view(1,3,1,1)
    def images(split,indices):return (cache[split][0][indices].to(device,dtype=torch.float32)/255.-mean)/std
    def evaluate(model,split):
        t=[];c=[];model.eval()
        with torch.inference_mode():
            for start in range(0,len(cache[split][0]),96):
                a,b=model(images(split,slice(start,start+96)));t.append(a.cpu());c.append(b.cpu())
        t=torch.cat(t);c=torch.cat(c);yt,yc=cache[split][1:3];valid=yc>=0
        m={'types':metrics(t,yt,TYPE_NAMES),'colors':metrics(c,yc,COLOR_NAMES),
            'joint_accuracy':float(((t.argmax(1)==yt)&(c.argmax(1)==yc))[valid].float().mean()),'joint_n':int(valid.sum())}
        score=.4*np.mean([m['types']['per_class_accuracy'][k] for k in CLASSES])+.4*m['colors']['macro_accuracy']+.2*m['joint_accuracy']
        return m,float(score),t,c
    candidates=[]
    for name in ['mobilenet_dual_v2','resnet18_dual_v1']:
        source=OUT/f'attribute_training/{name}/best.pt';checkpoint=torch.load(source,map_location='cpu',weights_only=False)
        architecture=checkpoint.get('architecture','mobilenet_v3_large');model=VehicleAttributeNet(architecture=architecture).to(device);model.load_state_dict(checkpoint['state_dict'])
        m,score,_,_=evaluate(model,'validation');candidates.append({'name':name,'weights':str(source),'validation':m,'score':score,'normalization':'as trained'})
        # Remove augmentation-dependent BatchNorm running-statistic shift using
        # only clean training inputs. Dropout remains disabled; no gradients.
        model.eval()
        for module in model.modules():
            if isinstance(module,nn.BatchNorm2d):module.reset_running_stats();module.momentum=None;module.train()
        with torch.no_grad():
            for start in range(0,len(cache['train'][0]),96):model(images('train',slice(start,start+96)))
        m,score,_,_=evaluate(model,'validation');path=OUT/f'attribute_training/{name}_bn_clean/best.pt';path.parent.mkdir(parents=True,exist_ok=True)
        modified={**checkpoint,'state_dict':{k:v.cpu() for k,v in model.state_dict().items()},'validation':m,
            'architecture':architecture,'normalization_recalibration':'all clean training inputs, no labels or validation/test inputs used for BN statistics'}
        torch.save(modified,path);candidates.append({'name':name+'_bn_clean','weights':str(path),'validation':m,'score':score,'normalization':'clean training BN recalibration'})
        print('CANDIDATES',[(r['name'],round(r['score'],4),round(r['validation']['types']['per_class_accuracy']['van'],4)) for r in candidates[-2:]],flush=True)
        del model
    chosen=max(candidates,key=lambda r:r['score']);dest=OUT/'attribute_training/selected';dest.mkdir(exist_ok=True)
    checkpoint=torch.load(chosen['weights'],map_location='cpu',weights_only=False)
    model=VehicleAttributeNet(architecture=checkpoint.get('architecture','mobilenet_v3_large')).to(device);model.load_state_dict(checkpoint['state_dict'])
    val,_,tt,cc=evaluate(model,'validation')
    from torch.nn import functional as F
    for name,logits,truth in [('type',tt,cache['validation'][1]),('color',cc,cache['validation'][2])]:
        valid=truth>=0;checkpoint[name+'_temperature']=min([.7,1.,1.3,1.8,2.5],key=lambda t:float(F.cross_entropy(logits[valid]/t,truth[valid])))
    torch.save(checkpoint,dest/'best.pt')
    torch.save({'type_logits':tt,'color_logits':cc,'type_truth':cache['validation'][1],'color_truth':cache['validation'][2],
        'ids':[r['id'] for r in cache['validation'][3]],'type_temperature':checkpoint['type_temperature'],'color_temperature':checkpoint['color_temperature']},dest/'validation_predictions.pt')
    test,_,tt,cc=evaluate(model,'test');save(dest/'test_metrics.json',test)
    save(dest/'model_manifest.json',{'weights':str(dest/'best.pt'),'sha256':digest(dest/'best.pt'),'selected':chosen['name'],
        'validation':val,'test':test,'architecture':checkpoint.get('architecture','mobilenet_v3_large'),
        'selection_basis':'validation score only; test evaluated descriptively after selection',
        'dataset_manifest_sha256':digest(OUT/'attribute_data_v4/manifest.jsonl')})
    save(OUT/'attribute_training/model_selection.json',{'candidates':candidates,'chosen':chosen['name'],
        'selection_rule':'.4 vehicle macro accuracy + .4 color macro accuracy + .2 joint accuracy on validation only'})
    print('SELECTED',chosen['name'],chosen['score'],'TEST',test,flush=True)


if __name__=='__main__':main()

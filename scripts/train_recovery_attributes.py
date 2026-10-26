"""Fine-tune a mature pretrained RGB backbone with explicit type/color heads."""
from pathlib import Path
import argparse,json,random,sys,time
import numpy as np,cv2,torch
from torch import nn
from torch.nn import functional as F
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.vehicle_attributes import VehicleAttributeNet,TYPE_NAMES,COLOR_NAMES,letterbox_rgb


def metrics(logits,targets,names):
    prediction=logits.argmax(1);matrix=torch.zeros((len(names),len(names)),dtype=torch.int64)
    valid=targets>=0
    for y,p in zip(targets[valid].tolist(),prediction[valid].tolist()):matrix[y,p]+=1
    rates={name:float(matrix[i,i]/matrix[i].sum()) if matrix[i].sum() else None for i,name in enumerate(names)}
    return {'n':int(valid.sum()),'accuracy':float((prediction[valid]==targets[valid]).float().mean()) if valid.any() else None,
        'macro_accuracy':float(np.mean([v for v in rates.values() if v is not None])),
        'per_class_accuracy':rates,'confusion_matrix':matrix.tolist()}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--epochs',type=int,default=20);parser.add_argument('--batch-size',type=int,default=96)
    parser.add_argument('--architecture',default='mobilenet_v3_large',choices=['mobilenet_v3_large','resnet18']);parser.add_argument('--name',default='mobilenet_dual_v1');parser.add_argument('--manifest',type=Path,default=OUT/'attribute_data_v3/manifest.jsonl')
    parser.add_argument('--nearfield-weight',type=float,default=1.);parser.add_argument('--aerial-weight',type=float,default=1.)
    parser.add_argument('--initial-weights',type=Path);parser.add_argument('--balanced-devices',action='store_true');parser.add_argument('--color-only',action='store_true');args=parser.parse_args()
    dest=OUT/f'attribute_training/{args.name}';dest.mkdir(parents=True,exist_ok=False)
    save(dest/'training_protocol.json',{'architecture':args.architecture,'epochs':args.epochs,'batch_size':args.batch_size,
        'manifest':str(args.manifest),'manifest_sha256':digest(args.manifest),'nearfield_sampling_multiplier':args.nearfield_weight,
        'aerial_sampling_multiplier':args.aerial_weight,'initial_weights':str(args.initial_weights) if args.initial_weights else None,
        'initial_weights_sha256':digest(args.initial_weights) if args.initial_weights else None,'device_balanced_validation':args.balanced_devices,
        'color_only_finetuning':args.color_only,
        'random_seed':20261022,'training_script_sha256':digest(__file__),
        'selection':'epoch and temperature chosen using validation only; test descriptive after selection',
        'optimizer':'AdamW backbone_lr0.00012 head_lr0.0012 weight_decay0.0002; cosine to0.00001'})
    torch.set_num_threads(4);cv2.setNumThreads(1);torch.manual_seed(20261022);np.random.seed(20261022);random.seed(20261022)
    data=rows(args.manifest);cache={}
    for split in ['train','validation','test']:
        selected=[r for r in data if r['split']==split];images=[];types=[];colors=[];is_uav=[]
        if args.color_only and split=='train':selected=[r for r in selected if r.get('color')]
        for r in selected:
            images.append(letterbox_rgb(cv2.imread(r['crop']),192).transpose(2,0,1))
            types.append(TYPE_NAMES.index(r['category']));colors.append(COLOR_NAMES.index(r['color']) if r.get('color') else -100);is_uav.append(r['device']=='uav')
        cache[split]={'images':torch.from_numpy(np.stack(images)), 'types':torch.tensor(types), 'colors':torch.tensor(colors),
            'uav':torch.tensor(is_uav),'rows':selected}
        print('CACHED',split,len(selected),flush=True)
    device=torch.device('cuda:0');mean=torch.tensor([.485,.456,.406],device=device).view(1,3,1,1);std=torch.tensor([.229,.224,.225],device=device).view(1,3,1,1)
    model=VehicleAttributeNet(pretrained=True,architecture=args.architecture).to(device)
    if args.initial_weights:
        model.load_state_dict(torch.load(args.initial_weights,map_location='cpu',weights_only=False)['state_dict'])
    if args.color_only:
        if not args.initial_weights:raise RuntimeError('Color-only requires a frozen pretrained type model')
        for parameter in list(model.backbone.parameters())+list(model.type_head.parameters()):parameter.requires_grad_(False)
    optimizer=torch.optim.AdamW([{'params':model.backbone.parameters(),'lr':.00012},
        {'params':list(model.type_head.parameters())+list(model.color_head.parameters()),'lr':.0012}],weight_decay=.0002)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,args.epochs,eta_min=.00001)
    scaler=torch.amp.GradScaler('cuda');counts=torch.bincount(cache['train']['types'],minlength=len(TYPE_NAMES)).clamp_min(1)
    sample_weights=counts[cache['train']['types']].float().pow(-.65)
    sample_weights*=torch.tensor([args.nearfield_weight if r.get('source','').startswith('nearfield_') else 1. for r in cache['train']['rows']])
    sample_weights*=torch.tensor([args.aerial_weight if r.get('source','').startswith('aerial_') else 1. for r in cache['train']['rows']])
    if args.color_only:
        color_counts=torch.bincount(cache['train']['colors'],minlength=len(COLOR_NAMES)).clamp_min(1)
        sample_weights*=color_counts[cache['train']['colors']].float().pow(-.5)

    def evaluate(split):
        model.eval();tt=[];cc=[];entry=cache[split]
        with torch.inference_mode():
            for offset in range(0,len(entry['rows']),args.batch_size):
                x=entry['images'][offset:offset+args.batch_size].to(device,dtype=torch.float32)/255.
                types,colors=model((x-mean)/std);tt.append(types.cpu());cc.append(colors.cpu())
        types=torch.cat(tt);colors=torch.cat(cc);result={'types':metrics(types,entry['types'],TYPE_NAMES),'colors':metrics(colors,entry['colors'],COLOR_NAMES)}
        valid=entry['colors']>=0;result['joint_n']=int(valid.sum());result['joint_accuracy']=float(((types.argmax(1)==entry['types'])&(colors.argmax(1)==entry['colors']))[valid].float().mean())
        result['by_device']={}
        for name,mask in [('uav',entry['uav']),('ugv',~entry['uav'])]:
            result['by_device'][name]={'types':metrics(types[mask],entry['types'][mask],TYPE_NAMES),'colors':metrics(colors[mask],entry['colors'][mask],COLOR_NAMES)}
            dv=mask&valid
            result['by_device'][name]['joint_accuracy']=float(((types.argmax(1)==entry['types'])&(colors.argmax(1)==entry['colors']))[dv].float().mean()) if dv.any() else None
        return result,types,colors

    best=-1.;history=[]
    for epoch in range(args.epochs):
        started=time.perf_counter();model.train();losses=[]
        if args.color_only:model.backbone.eval();model.type_head.eval()
        order=torch.multinomial(sample_weights,len(sample_weights),replacement=True)
        entry=cache['train']
        for offset in range(0,len(order),args.batch_size):
            idx=order[offset:offset+args.batch_size];x=entry['images'][idx].to(device,dtype=torch.float32)/255.;y=entry['types'][idx].to(device);c=entry['colors'][idx].to(device)
            # Observed camera role is used only for physically sensible training
            # rotations; the deployed model consumes RGB alone.
            uav=entry['uav'][idx].to(device);n=len(idx);angle=(torch.rand(n,device=device)*2-1)*torch.where(uav,torch.tensor(np.pi,device=device),torch.tensor(.10,device=device))
            scale=.92+.16*torch.rand(n,device=device);flip=torch.where(torch.rand(n,device=device)<.5,-1.,1.)
            theta=torch.zeros((n,2,3),device=device);theta[:,0,0]=angle.cos()*scale*flip;theta[:,0,1]=-angle.sin()*scale
            theta[:,1,0]=angle.sin()*scale*flip;theta[:,1,1]=angle.cos()*scale;theta[:,:,2]=(torch.rand((n,2),device=device)-.5)*.04
            grid=F.affine_grid(theta,x.size(),align_corners=False);x=F.grid_sample(x-.5,grid,align_corners=False,padding_mode='zeros')+.5
            x=(x*(.93+.14*torch.rand(n,1,1,1,device=device))).clamp(0,1)
            optimizer.zero_grad(set_to_none=True)
            with torch.autocast('cuda',dtype=torch.float16):
                tlogits,clogits=model((x-mean)/std);loss=clogits.sum()*0 if args.color_only else F.cross_entropy(tlogits,y,label_smoothing=.03)
                if (c>=0).any():loss=loss+.8*F.cross_entropy(clogits[c>=0],c[c>=0],label_smoothing=.02)
            scaler.scale(loss).backward();scaler.step(optimizer);scaler.update();losses.append(float(loss.detach()))
        scheduler.step();validation,tlogits,clogits=evaluate('validation')
        vtype=np.mean([validation['types']['per_class_accuracy'][c] for c in CLASSES]);score=.4*vtype+.4*validation['colors']['macro_accuracy']+.2*validation['joint_accuracy']
        if args.balanced_devices:
            dt=np.mean([m['types']['per_class_accuracy'][c] for m in validation['by_device'].values() for c in CLASSES if m['types']['per_class_accuracy'][c] is not None])
            dc=np.mean([m['colors']['per_class_accuracy'][c] for m in validation['by_device'].values() for c in COLORS if m['colors']['per_class_accuracy'][c] is not None])
            dj=np.mean([m['joint_accuracy'] for m in validation['by_device'].values() if m['joint_accuracy'] is not None]);score=.4*dt+.4*dc+.2*dj
        row={'epoch':epoch+1,'train_loss':float(np.mean(losses)),'validation_score':float(score),'validation':validation,'wall_seconds':time.perf_counter()-started}
        history.append(row);save(dest/'history.json',history)
        if score>best:
            best=score;torch.save({'state_dict':model.state_dict(),'type_names':TYPE_NAMES,'color_names':COLOR_NAMES,'image_size':192,
                'epoch':epoch+1,'architecture':args.architecture,'backbone_pretraining':'torchvision '+args.architecture+' ImageNet weights','validation':validation,
                'dataset_manifest_sha256':digest(args.manifest),'type_temperature':1.,'color_temperature':1.,
                'scope':'RGB-only type/color classifier; quality-filtered manifest-region holdout; physical background overlap exists; same-map regression'},dest/'best.pt')
        print(f'EPOCH {epoch+1}/{args.epochs} loss={np.mean(losses):.4f} vtype={vtype:.4f} vcolor={validation["colors"]["macro_accuracy"]:.4f} joint={validation["joint_accuracy"]:.4f} score={score:.4f} {row["wall_seconds"]:.1f}s',flush=True)
    checkpoint=torch.load(dest/'best.pt',map_location=device,weights_only=False);model.load_state_dict(checkpoint['state_dict'])
    validation,tlogits,clogits=evaluate('validation')
    for name,logits,truth in [('type',tlogits,cache['validation']['types']),('color',clogits,cache['validation']['colors'])]:
        valid=truth>=0;grid=[(.7,0),(1.,0),(1.3,0),(1.8,0),(2.5,0)]
        choice=min([float(t) for t,_ in grid],key=lambda t:float(F.cross_entropy(logits[valid]/t,truth[valid])))
        checkpoint[f'{name}_temperature']=choice
    checkpoint['state_dict']={k:v.cpu() for k,v in checkpoint['state_dict'].items()};torch.save(checkpoint,dest/'best.pt')
    # Test is evaluated once after all epoch/temperature decisions use validation only.
    test,tt,cc=evaluate('test');save(dest/'test_metrics.json',test)
    for split in ['validation','test']:
        result,t,c=evaluate(split);entry=cache[split]
        torch.save({'type_logits':t,'color_logits':c,'type_truth':entry['types'],'color_truth':entry['colors'],
            'ids':[r['id'] for r in entry['rows']],'type_temperature':checkpoint['type_temperature'],'color_temperature':checkpoint['color_temperature']},dest/f'{split}_predictions.pt')
    save(dest/'model_manifest.json',{'weights':str(dest/'best.pt'),'sha256':digest(dest/'best.pt'),'validation':validation,'test':test,
        'type_temperature':checkpoint['type_temperature'],'color_temperature':checkpoint['color_temperature'],
        'test_scope':'conditional attribute crops, not full-frame AP or online task success'})
    print('FINAL',json.dumps({'best_epoch':checkpoint['epoch'],'validation':validation,'test':test}),flush=True)

if __name__=='__main__':main()

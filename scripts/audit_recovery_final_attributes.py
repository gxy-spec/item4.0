"""Post-freeze GT-crop audit; never tunes weights, temperatures or thresholds."""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np,torch
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
sys.path.insert(0,str(ROOT))
from src.perception.vehicle_attributes import VehicleAttributeNet,TYPE_NAMES,COLOR_NAMES,letterbox_rgb


def main():
    p=argparse.ArgumentParser();p.add_argument('--device',default='cpu');a=p.parse_args()
    torch.set_num_threads(2);cv2.setNumThreads(1)
    data=rows(OUT/'final_attribute_audit_v1/manifest.jsonl');assert all(r['split']=='test' for r in data)
    cal=json.loads((OUT/'calibration/deployment_release.json').read_text(encoding='utf-8'));weight=Path(cal['weights']);initial_sha=digest(weight)
    checkpoint=torch.load(weight,map_location='cpu',weights_only=False)
    net=VehicleAttributeNet(architecture=checkpoint['architecture']).to(a.device);net.load_state_dict(checkpoint['state_dict']);net.eval()
    mean=torch.tensor([.485,.456,.406],device=a.device).view(1,3,1,1);std=torch.tensor([.229,.224,.225],device=a.device).view(1,3,1,1)
    predictions=[]
    with torch.inference_mode():
        for start in range(0,len(data),32):
            batch=data[start:start+32]
            x=torch.from_numpy(np.stack([letterbox_rgb(cv2.imread(r['crop']),192).transpose(2,0,1) for r in batch])).to(a.device,dtype=torch.float32)/255
            t,c=net((x-mean)/std);t=(t/checkpoint['type_temperature']).softmax(1).cpu().numpy();c=(c/checkpoint['color_temperature']).softmax(1).cpu().numpy()
            for r,tp,cp in zip(batch,t,c):
                raw_type=TYPE_NAMES[int(tp.argmax())];raw_color=COLOR_NAMES[int(cp.argmax())]
                predictions.append(dict(id=r['id'],device=r['device'],truth_type=r['category'],truth_color=r['color'],
                    raw_type=raw_type,raw_color=raw_color,type_probability=float(tp.max()),color_probability=float(cp.max()),
                    deployed_type=raw_type if tp.max()>=cal['type_threshold'] else 'unknown',
                    deployed_color=raw_color if cp.max()>=cal['color_threshold'] else 'unknown',image=r['image'],crop=r['crop']))
    def stats(z):
        q=[r for r in z if r['truth_color'] is not None]
        return dict(n=len(z),color_n=len(q),raw_type_correct=sum(r['raw_type']==r['truth_type'] for r in z),
            deployed_type_correct=sum(r['deployed_type']==r['truth_type'] for r in z),
            raw_color_correct=sum(r['raw_color']==r['truth_color'] for r in q),
            deployed_color_correct=sum(r['deployed_color']==r['truth_color'] for r in q),
            joint_correct=sum(r['deployed_type']==r['truth_type'] and r['deployed_color']==r['truth_color'] for r in q))
    summary=dict(**stats(predictions),by_device={d:stats([r for r in predictions if r['device']==d]) for d in ['uav','ugv']},
        by_type={t:stats([r for r in predictions if r['truth_type']==t]) for t in TYPE_NAMES},
        by_color={c:stats([r for r in predictions if r['truth_color']==c]) for c in COLOR_NAMES},
        weights_sha256=initial_sha,benchmark_manifest_sha256=digest(OUT/'final_attribute_audit_v1/manifest.jsonl'),
        scope='Post-freeze conditional GT-crop attribute accuracy, including separately counted certified nonvehicle crops. One new controlled target location on the same map/assets; correlated views, not cross-map generalization or detector AP. No tuning/selection from these images.',
        thresholds=dict(type=cal['type_threshold'],color=cal['color_threshold']),model_unchanged=digest(weight)==initial_sha)
    dest=OUT/'final_attribute_audit_results';dest.mkdir(exist_ok=True)
    (dest/'predictions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in predictions),encoding='utf-8');save(dest/'summary.json',summary)
    print(json.dumps({k:summary[k] for k in ['n','color_n','deployed_type_correct','deployed_color_correct','joint_correct','model_unchanged']}),flush=True)


if __name__=='__main__':main()

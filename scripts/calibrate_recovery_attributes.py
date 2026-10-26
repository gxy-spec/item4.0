"""Validation-only abstention thresholds; test remains descriptive, not tuned."""
from pathlib import Path
import argparse,json,sys,torch,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.vehicle_attributes import TYPE_NAMES,COLOR_NAMES


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--model-name',default='mobilenet_dual_v1');parser.add_argument('--deployment',default='deployment.json');args=parser.parse_args()
    model=OUT/f'attribute_training/{args.model_name}'
    manifest=json.loads((model/'model_manifest.json').read_text(encoding='utf-8'))
    predictions=torch.load(model/'validation_predictions.pt',weights_only=False)
    types=(predictions['type_logits']/predictions['type_temperature']).softmax(1)
    colors=(predictions['color_logits']/predictions['color_temperature']).softmax(1)
    tp,ti=types.max(1);cp,ci=colors.max(1);yt=predictions['type_truth'];yc=predictions['color_truth']
    usable=(yc>=0)|(yt==TYPE_NAMES.index('non_vehicle'));positive=yc>=0
    grid=[]
    for t in np.arange(.4,.96,.05):
        for c in np.arange(.4,.96,.05):
            accepted=(tp>=t)&(cp>=c)&(ti!=TYPE_NAMES.index('non_vehicle'))&usable
            correct=accepted&(ti==yt)&(ci==yc)&positive
            precision=int(correct.sum())/max(1,int(accepted.sum()));recall=int(correct.sum())/max(1,int(positive.sum()))
            f1=2*precision*recall/max(1e-9,precision+recall)
            grid.append({'type_threshold':round(float(t),2),'color_threshold':round(float(c),2),
                'accepted':int(accepted.sum()),'correct':int(correct.sum()),'positive':int(positive.sum()),
                'precision':precision,'recall':recall,'f1':f1})
    # Select joint F1 under a declared precision preference. This preference is
    # a validation operating-point choice, not a universal domain standard.
    feasible=[r for r in grid if r['precision']>=.95]
    selected=max(feasible or grid,key=lambda r:(r['f1'],r['precision'],-r['type_threshold']-r['color_threshold']))
    save(OUT/f'calibration/{args.model_name}_validation_grid.json',{'scope':'conditional known-color crops + explicit background negatives',
        'selection_rule':'maximum joint F1 among validation precision>=0.95 points, otherwise maximum F1',
        'feasible':bool(feasible),'selected':selected,'rows':grid})
    save(OUT/f'calibration/{args.deployment}',{'weights':manifest['weights'],'weights_sha256':manifest['sha256'],
        **selected,'validation_precision_preference_feasible':bool(feasible),
        'source':'validation split only','temperatures':{'type':predictions['type_temperature'],'color':predictions['color_temperature']},
        'threshold_basis':'deployment operating point; no claim of a universal minimum acceptable accuracy'})
    print(json.dumps(selected),flush=True)


if __name__=='__main__':main()

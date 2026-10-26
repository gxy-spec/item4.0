from pathlib import Path
import sys,json,math
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from src.perception.body_clearance import observed_clearance
from src.perception.candidate_pipeline import transform_matrix
from recovery_stage import OUT,save


def main():
    source=OUT/'clearance_calibration';dest=OUT/'clearance_calibration_v2'
    records=json.loads((source/'records.json').read_text(encoding='utf-8'))
    mount=transform_matrix(dict(location=[1.5,0,2],rotation=[-5,0,0]))
    for r in records:
        bgr=cv2.imread(str(source/'frames'/f"{r['id']}_depth.png")).astype(float)
        depth=(bgr[:,:,2]+bgr[:,:,1]*256+bgr[:,:,0]*65536)/16777215*1000
        c=r['candidate']
        e=observed_clearance(depth,c['bbox_xyxy'],c['depth_m'],mount,r['own_body_polygon'],100.) if c and r['box_iou']>=.5 else dict(valid=False,reason='no_target')
        r['previous_estimate']=r['estimate'];r['estimate']=e
        r['signed_error_m']=e['estimated_body_clearance_m']-r['true_gap_m'] if e['valid'] else None
        print(r['id'],r['signed_error_m'],flush=True)
    valid=[r for r in records if r['signed_error_m'] is not None];errors=np.array([r['signed_error_m'] for r in valid])
    summary=dict(total=len(records),valid=len(valid),coverage=len(valid)/len(records),mae_m=float(np.mean(abs(errors))),
        p95_absolute_error_m=float(np.percentile(abs(errors),95)),maximum_absolute_error_m=float(max(abs(errors))),
        maximum_dangerous_overestimate_m=float(max(errors)),passed=bool(len(valid)>=24 and max(abs(errors))<=1.),
        scope='Same controlled images after estimator repair; development calibration, not independent test')
    save(dest/'records.json',records);save(dest/'summary.json',summary);print(json.dumps(summary))


if __name__=='__main__':main()

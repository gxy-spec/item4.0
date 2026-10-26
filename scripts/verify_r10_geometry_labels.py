"""Exact virtual own-body transform check, preserving original capture labels."""
from pathlib import Path
import json,sys
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest
from src.perception.candidate_pipeline import transform_matrix
from src.perception.body_clearance import observed_clearance,polygon_clearance


def main():
    mount=transform_matrix(dict(location=[1.5,0,2],rotation=[-5,0,0]))
    for cohort in ['clearance_calibration','clearance_validation']:
        source=OUT/cohort;records=json.loads((source/'records.json').read_text(encoding='utf-8'));result=[]
        for r in records:
            own=np.asarray(r['own_body_polygon']);pose=transform_matrix(r['own_virtual_pose'])
            world=(pose@np.column_stack([own,np.zeros(len(own)),np.ones(len(own))]).T).T[:,:2]
            truth=polygon_clearance(world,r['target_polygon'])
            bgr=cv2.imread(str(source/'frames'/f"{r['id']}_depth.png")).astype(float)
            depth=(bgr[:,:,2]+bgr[:,:,1]*256+bgr[:,:,0]*65536)*1000/16777215
            c=r['candidate'];e=observed_clearance(depth,c['bbox_xyxy'],c['depth_m'],mount,r['own_body_polygon'],100.) if c and r['box_iou']>=.5 else dict(valid=False,reason='target_not_detected')
            result.append(dict(id=r['id'],true_gap_m=truth,original_label_m=r['true_gap_m'],estimate=e,
                signed_error_m=e['estimated_body_clearance_m']-truth if e['valid'] else None))
        valid=[r['signed_error_m'] for r in result if r['signed_error_m'] is not None];errors=np.asarray(valid)
        summary=dict(cohort=cohort,total=len(result),valid=len(valid),invalid=len(result)-len(valid),
            coverage=len(valid)/len(result),mae_m=float(np.mean(abs(errors))),p95_m=float(np.percentile(abs(errors),95)),
            maximum_absolute_error_m=float(max(abs(errors))),maximum_dangerous_overestimate_m=float(max(errors)),
            passed=bool(len(valid)>=24 and max(abs(errors))<=1.),
            estimator_sha256=digest(Path(__file__).resolve().parents[1]/'src/perception/body_clearance.py'),
            label_rule='Exact own-local body transformed by virtual odometry; target polygon used offline only')
        dest=OUT/'geometry_verified'/cohort;save(dest/'records.json',result);save(dest/'summary.json',summary)
        print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':main()

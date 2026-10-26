"""All five distance definitions at exact source frames; all hold-frame safety evidence."""
from pathlib import Path
import sys,json,csv
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save
from build_r10_targeted_report import DEST


def write(path,records):
    if not records:return
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)


def main():
    rev=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))['active_validation_revision']
    observations=[];frames=[];summary=[]
    for cohort in ['acceptance','diagnostics']:
        path=OUT/f'{cohort}_development_{rev}_progress.json'
        if not path.exists():continue
        for item in json.loads(path.read_text(encoding='utf-8')).values():
            run=Path(item['run_dir']);ep=run/'r10_evaluation.json'
            if not ep.exists():continue
            e=json.loads(ep.read_text(encoding='utf-8'));errors=[]
            for n,c in enumerate(e.get('completion_audits',[])):
                if not c['passed']:continue
                actual={m['frame']:m for m in c['measurements']}
                for m in c['measurements']:frames.append(dict(case=item['case'],variant=item.get('variant'),hold=n,**m))
                for o in c['local_evidence']:
                    m=actual.get(o['frame_id'])
                    if m is None:raise RuntimeError('Hold local source frame missing true geometry')
                    b=o['body_clearance'];error=o['estimated_body_clearance_m']-m['true_body_clearance_m'];errors.append(error)
                    observations.append(dict(case=item['case'],variant=item.get('variant'),hold=n,source_frame=o['frame_id'],
                        timestamp_s=o['timestamp_s'],camera_optical_depth_m=o['depth_m'],
                        camera_visible_surface_horizontal_m=b['surface_horizontal_distance_m'],
                        online_estimated_body_gap_m=o['estimated_body_clearance_m'],offline_true_body_gap_m=m['true_body_clearance_m'],
                        actor_reference_distance_m=m['reference_distance_m'],signed_gap_error_m=error,
                        observed_type=o.get('observed_category'),observed_color=o.get('observed_color'),
                        confirmed_track_type=o['category'],confirmed_track_color=o['color'],tracking_provenance=o['tracking_provenance'],
                        correct_identity=o['identity_correct'],speed_mps=m['speed_mps'],drivable=m['legal_stop'],
                        junction=m['stopped_confirmation_in_junction']))
            if errors:
                v=np.asarray(errors);summary.append(dict(case=item['case'],variant=item.get('variant'),
                    valid_hold_observations=len(v),mae_m=float(np.mean(abs(v))),maximum_absolute_error_m=float(max(abs(v))),
                    maximum_dangerous_overestimate_m=max(0.,float(max(v)))))
    DEST.mkdir(exist_ok=True,parents=True)
    write(DEST/'停车保持段_逐观测五类距离.csv',observations);write(DEST/'停车保持段_逐仿真帧安全证据.csv',frames)
    save(OUT/'analysis/closed_loop_hold_distance_errors.json',dict(records=summary,
        scope='Successful hold intervals only; exact source-frame pairing, not all approach frames or independent calibration coverage. Unknown observed attributes remain unknown.'))
    print(json.dumps(dict(passed_holds=len(summary),hold_observations=len(observations),hold_physics_frames=len(frames),
        maximum_absolute_error_m=max((r['maximum_absolute_error_m'] for r in summary),default=None)),ensure_ascii=False))


if __name__=='__main__':main()

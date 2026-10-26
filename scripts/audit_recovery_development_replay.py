"""Classification-only replay of an archived failure, never a closed-loop rerun."""
import sys,json,cv2,torch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.perception.vehicle_attributes import VehicleAttributeRecognizer


def main():
    torch.set_num_threads(4);cv2.setNumThreads(2)
    progress=json.loads((OUT/'acceptance_v4_progress.json').read_text(encoding='utf-8'));run=Path(progress['yellow_car']['run_dir'])
    cal=json.loads((OUT/'calibration/deployment_v5.json').read_text(encoding='utf-8'));head=VehicleAttributeRecognizer(cal['weights'])
    candidates={c['candidate_id']:c for row in rows(run/'perception/ugv_target_candidates_synchronized.jsonl') for c in row['candidates']}
    records=[]
    for r in rows(run/'task1_candidate_audit.jsonl'):
        if r['device']!='ugv' or not r['semantic_correct']:continue
        a=head.predict(cv2.imread(r['image']),[r['bbox']])[0]
        t=a['predicted_type'] if a['type_probability']>=cal['type_threshold'] else 'unknown'
        c=a['predicted_color'] if a['color_probability']>=cal['color_threshold'] else 'unknown'
        records.append(dict(frame=r['frame'],candidate_id=r['candidate_id'],image=r['image'],bbox=r['bbox'],
            depth_m=candidates[r['candidate_id']]['depth_m'],old_type=r['category'],old_color=r['color'],new_type=t,new_color=c,
            truth_type=r['matched_category'],truth_color=r['matched_color'],**a))
    summary={}
    for subset,z in [('all',records),('near_surface_le8m',[r for r in records if r['depth_m']<=8])]:
        summary[subset]={'n':len(z)}
        for version in ['old','new']:
            summary[subset][version]=dict(type_correct=sum(r[version+'_type']==r['truth_type'] for r in z),color_correct=sum(r[version+'_color']==r['truth_color'] for r in z),joint_correct=sum(r[version+'_type']==r['truth_type'] and r[version+'_color']==r['truth_color'] for r in z))
    dest=OUT/'failure_diagnosis/nearfield_replay_v5';dest.mkdir(exist_ok=True)
    (dest/'predictions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records),encoding='utf-8')
    save(dest/'summary.json',dict(summary=summary,model_sha256=digest(cal['weights']),
        scope='Archived developmental yellow-car observations, same exact boxes/images; only classifier replaced. Does not rerun detection, navigation, communication or safety; not an independent online acceptance result.'))
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()

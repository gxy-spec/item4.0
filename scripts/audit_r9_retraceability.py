"""Old72 evidence availability only. Never invent new-protocol task passes."""
from pathlib import Path
import json,sys,csv,collections
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest


def main():
    old=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')
    index=old/'formal_progress.json';progress=json.loads(index.read_text(encoding='utf-8'));records=[]
    for key,item in progress.items():
        run=Path(item['run_dir']);first={};path=run/'actors/actor_states.jsonl'
        if path.exists():
            with path.open(encoding='utf-8') as f:
                for n,line in enumerate(f):
                    r=json.loads(line);first.setdefault(r['role'],r)
                    if ('ugv' in first and 'target' in first) or n>=30:break
        own=first.get('ugv',{});target=first.get('target',{})
        missing=[]
        if not own.get('bbox_world_vertices_xyz'):missing.append('exact_own_body_vertices')
        if not target.get('bbox_world_vertices_xyz'):missing.append('exact_target_body_vertices')
        if 'map_drivable_footprint' not in own:missing.append('drivable_footprint_hold_evidence')
        monitorpath=run/'ground_truth/evaluation_only/sensor_business_monitor.json'
        monitor=json.loads(monitorpath.read_text(encoding='utf-8')) if monitorpath.exists() else {}
        if monitor.get('protocol')!='R10-body-clearance-v1':missing.append('new_protocol_online_completion_state')
        records.append(dict(key=key,variant=item['variant'],case=item['case'],group=item['group'],run_dir=str(run),
            historical_final_reference_distance_m=item.get('summary',{}).get('ugv_final_target_distance_m'),
            exact_new_protocol_retraceability=not missing,missing=';'.join(missing)))
    destination=OUT/'analysis/R9_72_retraceability.csv'
    with destination.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    counts=collections.Counter(x for r in records for x in r['missing'].split(';') if x)
    summary=dict(total=len(records),fully_retraceable=sum(r['exact_new_protocol_retraceability'] for r in records),
        missing_counts=dict(counts),source_sha256=digest(index),records=str(destination),
        scope='Checks original first-frame schema and monitor availability; missing exact geometry/hold evidence prevents certified retrospective S3. No R9 result is overwritten or promoted.')
    save(OUT/'analysis/R9_72_retraceability_summary.json',summary);print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()

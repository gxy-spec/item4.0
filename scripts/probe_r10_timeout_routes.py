"""Read-only map query from recorded message/own pose; never contacts live server."""
from pathlib import Path
import sys,json,math,csv,time,statistics
import carla
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows
from src.navigation.approach_region import plan_observation_region
from src.perception.body_clearance import rectangle


def main():
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'));rev=scope['active_validation_revision']
    progress=json.loads((OUT/f'diagnostics_development_{rev}_progress.json').read_text(encoding='utf-8'))
    world_map=carla.Map('Town10HD',(OUT/'Town10HD.xodr').read_text(encoding='utf-8'));results=[]
    for item in progress.values():
        if item['case'] not in ['E2','F1','F3']:continue
        run=Path(item['run_dir']);ep=run/'r10_evaluation.json'
        if not ep.exists():continue
        e=json.loads(ep.read_text(encoding='utf-8'))
        if e.get('S3_found_and_safely_approached'):continue
        plan=next(p for p in rows(run/'planning/target_route_events.jsonl') if p['event']=='target_route_planned_from_selected_message')
        own=next(a for a in rows(run/'actors/actor_states.jsonl') if a['role']=='ugv')
        poly=rectangle(own['bbox_location_xyz'][:2],own['bbox_extent_xyz'][:2],own['bbox_rotation_pyr'][1])
        safety=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
        speeds=[float(s['ugv_speed_mps']) for s in safety if s['ugv_safety_mode']=='cruise' and float(s['timestamp'])<plan['timestamp_s']]
        before=time.perf_counter();audit={};error=None;route=[]
        try:route=plan_observation_region(world_map,carla.Location(*plan['start_xyz']),plan['target_xyz'],poly,audit=audit)
        except (RuntimeError,ValueError) as ex:error=str(ex)
        length=sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:])) if route else None
        result=dict(case=item['case'],run_dir=str(run),original_route_m=plan['route_length_m'],candidate_point_region_route_m=length,
            first_plan_elapsed_s=plan['timestamp_s']-own['timestamp'],remaining_time_s=item['horizon_seconds']-(plan['timestamp_s']-own['timestamp']),
            prior_observed_cruise_median_mps=statistics.median(speeds) if speeds else None,
            query_wall_seconds=time.perf_counter()-before,map_audit=audit,error=error,
            original_route=plan['route_xyz'],candidate_route=route,
            scope='Offline map/received-point/known-own-body query, not a target-truth or online collision-clearance certification; not executed by robot')
        results.append(result);print(json.dumps({k:v for k,v in result.items() if k not in ['original_route','candidate_route','map_audit']},ensure_ascii=False),flush=True)
    save(OUT/'analysis/timeout_route_candidate_queries.json',results)


if __name__=='__main__':main()

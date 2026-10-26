"""Offline impact screen of the prepared budget policy on every completed run."""
from pathlib import Path
import sys,json,csv
import carla
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows
from src.navigation.budgeted_approach import select_task_route
from src.perception.body_clearance import rectangle


def main():
    rev=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))['active_validation_revision']
    world_map=carla.Map('Town10HD',(OUT/'Town10HD.xodr').read_text(encoding='utf-8'));results=[]
    for cohort in ['safety','acceptance','diagnostics']:
        p=OUT/f'{cohort}_development_{rev}_progress.json'
        if not p.exists():continue
        for key,item in json.loads(p.read_text(encoding='utf-8')).items():
            if not item.get('report_complete'):continue
            run=Path(item['run_dir']);planpath=run/'planning/target_route_events.jsonl'
            plans=[p for p in rows(planpath) if p['event'] in ['target_route_planned_from_selected_message','blocked_route_replanned']] if planpath.exists() else []
            decisions=[]
            if plans:
                own=[a for a in rows(run/'actors/actor_states.jsonl') if a['role']=='ugv'];first=own[0]
                poly=rectangle(first['bbox_location_xyz'][:2],first['bbox_extent_xyz'][:2],first['bbox_rotation_pyr'][1])
                safety=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
                for plan in plans:
                    before=[r for r in safety if float(r['timestamp'])<=plan['timestamp_s']]
                    pose=max((a for a in own if a['timestamp']<=plan['timestamp_s']+1e-6),key=lambda a:a['timestamp'])
                    start=plan.get('start_xyz',[pose['x'],pose['y'],pose['z']])
                    remaining=item['horizon_seconds']-(plan['timestamp_s']-first['timestamp'])
                    chosen,audit=select_task_route(world_map,carla.Location(*start),plan['target_xyz'],poly,plan['route_xyz'],
                        remaining,2.,before,plan['timestamp_s'])
                    decisions.append(dict(event=plan['event'],timestamp_s=plan['timestamp_s'],message_id=plan['message_id'],
                        **audit,unchanged_route=chosen is plan['route_xyz']))
            results.append(dict(cohort=cohort,key=key,case=item['case'],variant=item.get('variant'),group=item.get('group'),
                run_dir=str(run),changes_control=any(d['accepted'] for d in decisions),decisions=decisions))
    result=dict(source_revision=rev,records=results,changed_runs=[r['key'] for r in results if r['changes_control']],
        scope='Prepared policy replay from logged received points, own pose/speeds and static map. Not an executed route or closed-loop success.')
    save(OUT/'analysis/proposed_v5_budget_policy_impact.json',result)
    print(json.dumps(dict(completed_runs=len(results),would_change=result['changed_runs']),ensure_ascii=False))


if __name__=='__main__':main()

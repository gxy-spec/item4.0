"""Apply exact reversible edits only after the v4 physical queue has ended."""
from pathlib import Path
import sys,json,psutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import ROOT,OUT,save,digest


def main():
    for proc in psutil.process_iter(['pid','name','cmdline']):
        if proc.pid==__import__('os').getpid():continue
        try:
            names=[Path(a).name for a in proc.info['cmdline'] or [] if a.endswith('.py')]
            if any(n in names for n in ['run_stage1_dynamic.py','run_r10_development.py','run_r10_targeted.py']):
                raise RuntimeError('Physical acquisition still active; leave frozen v4 unchanged')
        except (psutil.NoSuchProcess,psutil.AccessDenied):pass
    edits=[]
    def replace(before,after):edits.append(dict(before=before,after=after))
    replace('from navigation.ugv_oracle_planner import plan_route_from_message  # noqa: E402',
            'from navigation.ugv_oracle_planner import plan_route_from_message  # noqa: E402\nfrom navigation.budgeted_approach import select_task_route  # noqa: E402')
    replace('    lookahead_points: int = 6,\n) -> int:',
            '    lookahead_points: int = 6,\n    end_stop_tolerance_m: float = 3.0,\n) -> int:')
    replace('    if remaining < 3.0:', '    if remaining < end_stop_tolerance_m:')
    replace('            lookahead_points=3 if lane_change_active else 6,\n        )\n        if horizontal_distance(s0.xyz(vehicle.get_location()), route[-1]) < 3.0:',
            '            lookahead_points=3 if lane_change_active else 6,\n            end_stop_tolerance_m=float(safety.get("route_end_stop_tolerance_m",3.0)),\n        )\n        if horizontal_distance(s0.xyz(vehicle.get_location()), route[-1]) < float(safety.get("route_end_stop_tolerance_m",3.0)):')
    replace('            "src/safety/goal_approach_guard.py",',
            '            "src/safety/goal_approach_guard.py",\n            "src/navigation/approach_region.py", "src/navigation/budgeted_approach.py",')
    replace('        persistent_route_events: list[dict[str, Any]] = []',
            '        persistent_route_events: list[dict[str, Any]] = []\n        route_budget_audits: list[dict[str, Any]] = []\n        active_target_end_stop_tolerance_m = 3.0')
    replace('                        if len(planned_target_route) >= 2:',
            '''                        planned_target_route,budget_audit=select_task_route(
                            world_map,ugv.get_location(),selected_xyz,own_body_polygon,planned_target_route,
                            float(resolved['simulation']['duration_seconds'])-sim_elapsed,
                            float(dynamic_cfg['ugv_target_speed_mps']),safety_rows,communication_now_s)
                        active_target_end_stop_tolerance_m=budget_audit['chosen_end_stop_tolerance_m']
                        route_budget_audits.append(dict(timestamp_s=communication_now_s,
                            event='initial_target_plan',message_id=str(persistent_selected_state['message_id']),**budget_audit))
                        if budget_audit['accepted']:active_overtake=None
                        if len(planned_target_route) >= 2:''')
    replace('                            "target_stop_distance_m": max(0.0, target_standoff_m - 1.2),',
            '                            "target_stop_distance_m": max(0.0, target_standoff_m - 1.2),\n                            "route_end_stop_tolerance_m": active_target_end_stop_tolerance_m,')
    replace("                        persistent_route_cursor=0\n                        persistent_route_events.append(dict(event='blocked_route_replanned'",
            '''                        active_ugv_route,budget_audit=select_task_route(
                            world_map,ugv.get_location(),business_reference_xyz,own_body_polygon,active_ugv_route,
                            float(resolved['simulation']['duration_seconds'])-sim_elapsed,
                            float(dynamic_cfg['ugv_target_speed_mps']),safety_rows,communication_now_s)
                        active_target_end_stop_tolerance_m=budget_audit['chosen_end_stop_tolerance_m']
                        route_budget_audits.append(dict(timestamp_s=communication_now_s,
                            event='blocked_target_replan',message_id=business_reference_state.get('message_id'),**budget_audit))
                        if budget_audit['accepted']:active_overtake=None
                        persistent_route_cursor=0
                        persistent_route_events.append(dict(event='blocked_route_replanned' ''')
    replace('            write_jsonl(run_dirs["planning"] / "target_route_events.jsonl", persistent_route_events)',
            '            write_jsonl(run_dirs["planning"] / "target_route_events.jsonl", persistent_route_events)\n            write_jsonl(run_dirs["planning"] / "r10_route_budget_decisions.jsonl", route_budget_audits)')
    path=ROOT/'scripts/run_stage1_dynamic.py';original=path.read_text(encoding='utf-8');result=original
    for edit in edits:
        if result.count(edit['before'])!=1:raise RuntimeError('Expected unique edit: '+edit['before'])
        result=result.replace(edit['before'],edit['after'],1)
    compile(result,str(path),'exec')
    original_frozen=(OUT/'frozen_validation_v4/scripts/run_stage1_dynamic.py').read_text(encoding='utf-8')
    if original!=original_frozen:raise RuntimeError('Not exact frozen v4 source')
    proof=OUT/'validation5_exact_main_edits.json'
    if proof.exists():raise RuntimeError('Do not overwrite edit proof')
    save(proof,dict(source='validation4',edits=edits,original_sha256=digest(path)))
    path.write_text(result,encoding='utf-8')
    print('Applied',len(edits),'exact reversible main edits')


if __name__=='__main__':main()

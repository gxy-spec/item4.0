from pathlib import Path
p=Path(__file__).resolve().parents[1]/'scripts/run_stage1_dynamic.py'
s=p.read_text(encoding='utf-8')
changes=[
('def mapped_depth_corridor(vehicle, route):\n    from safety.route_corridor import local_route',
 '''def mapped_depth_corridor(vehicle, route, overtake=None):
    from safety.route_corridor import local_route
    if overtake is not None:
        route=route_with_overtake_offset(route,float(overtake['start_s_m']),float(overtake['end_s_m']),
            float(overtake['side_offset_m']),ramp_m=9.)''',1),
('mapped_depth_corridor(ugv,active_ugv_route if persistent_route_active else route)',
 'mapped_depth_corridor(ugv,active_ugv_route if persistent_route_active else route,active_overtake)',2),
('if active_overtake is not None and not latest_hazard_decision.get("forced_stop_reason"):',
 '''if (active_overtake is not None and not latest_hazard_decision.get("forced_stop_reason")
                                and not any(word in str(ugv_safety.get('ugv_safety_mode','')) for word in ['stop','wait','stale'])):''',1),
("persistent_target_cfg.get('route_standoff_m',6.0)","persistent_target_cfg.get('standoff_m',6.0)",1),
("dict(event='blocked_route_replanned',timestamp_s=sim_elapsed,\n                            target_xyz=business_reference_xyz,route_xyz=active_ugv_route)",
 "dict(event='blocked_route_replanned',timestamp_s=communication_now_s,elapsed_seconds=sim_elapsed,\n                            target_xyz=business_reference_xyz,route_xyz=active_ugv_route,\n                            message_id=business_reference_state.get('message_id'),source=business_reference_state.get('source'),\n                            route_length_m=cumulative_distance(active_ugv_route))",1),
("dict(event='blocked_replan_rejected',timestamp_s=sim_elapsed,reason=str(exc))",
 "dict(event='blocked_replan_rejected',timestamp_s=communication_now_s,elapsed_seconds=sim_elapsed,reason=str(exc))",1)]
for old,new,count in changes:
    assert s.count(old)==count,(s.count(old),old)
    s=s.replace(old,new)
p.write_text(s,encoding='utf-8')
print('FIXED actual overtake depth corridor; blocked modes and event clocks preserved')

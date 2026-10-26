from pathlib import Path
p=Path(__file__).resolve().parents[1]/'scripts/run_stage1_dynamic.py'
s=p.read_text(encoding='utf-8')
old='''def mapped_depth_corridor(vehicle, route, overtake=None):
    from safety.route_corridor import local_route
    if overtake is not None:
        route=route_with_overtake_offset(route,float(overtake['start_s_m']),float(overtake['end_s_m']),
            float(overtake['side_offset_m']),ramp_m=9.)'''
new='''def effective_control_route(route,overtake=None):
    if overtake is None:return route
    return route_with_overtake_offset(route,float(overtake['start_s_m']),float(overtake['end_s_m']),
        float(overtake['side_offset_m']),ramp_m=9.)


def mapped_depth_corridor(vehicle, route, overtake=None):
    from safety.route_corridor import local_route
    route=effective_control_route(route,overtake)'''
assert s.count(old)==1;s=s.replace(old,new)
old='''                            active_ugv_route,
                            float(dynamic_cfg["ugv_target_speed_mps"])'''
assert s.count(old)==4,s.count(old)
# Two safety-controlled branches plus direct-follow compatibility branches.
s=s.replace(old,'''                            effective_control_route(active_ugv_route,active_overtake),
                            float(dynamic_cfg["ugv_target_speed_mps"])''')
old='''                            hazard_decision=latest_hazard_decision,
                            route_cursor_index=persistent_route_cursor,'''
assert s.count(old)==2;s=s.replace(old,'''                            hazard_decision=latest_hazard_decision,
                            overtake_active=active_overtake is not None,
                            lane_change_active=active_overtake is not None,
                            route_cursor_index=persistent_route_cursor,''')
old='''                        candidate = decision.get("slow_vehicle_candidate")'''
assert s.count(old)==1;s=s.replace(old,'''                        hazard_reference_route=active_ugv_route if persistent_route_active else route
                        hazard_route_cursor=persistent_route_cursor if persistent_route_active else ugv_safety_route_cursor
                        candidate = decision.get("slow_vehicle_candidate")''')
old='''                            progress, _ = route_progress_m(
                                route,
                                [ego_state["x"], ego_state["y"], ego_state["z"]],
                                cursor_index=ugv_safety_route_cursor,
                            )'''
assert s.count(old)==1;s=s.replace(old,'''                            progress, _ = route_progress_m(
                                hazard_reference_route,
                                [ego_state["x"], ego_state["y"], ego_state["z"]],
                                cursor_index=hazard_route_cursor,
                            )''')
old='''                                ugv,
                                route,
                                progress,'''
assert s.count(old)==1;s=s.replace(old,'''                                ugv,
                                hazard_reference_route,
                                progress,''')
old='''                        actual_lane_offset = route_lateral_offset_m(
                            route,'''
assert s.count(old)==1;s=s.replace(old,'''                        actual_lane_offset = route_lateral_offset_m(
                            active_ugv_route if persistent_route_active else route,''')
old='''            sensor_observation: dict[str, Any] | None = None'''
assert s.count(old)==1;s=s.replace(old,'''            if active_overtake is not None:
                current_base_route=active_ugv_route if persistent_route_active else route
                current_cursor=persistent_route_cursor if persistent_route_active else ugv_safety_route_cursor
                maneuver_progress,_=route_progress_m(current_base_route,s0.xyz(ugv.get_location()),cursor_index=current_cursor)
                if maneuver_progress>=float(active_overtake['end_s_m']):active_overtake=None
            sensor_observation: dict[str, Any] | None = None''')
p.write_text(s,encoding='utf-8')
print('Aligned actual target/patrol routes, legal passing checks, and depth corridors')

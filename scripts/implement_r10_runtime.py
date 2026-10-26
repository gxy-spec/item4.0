"""Apply reviewed edits only to the isolated R10 checkout; fail on stale anchors."""
from pathlib import Path
import json,hashlib

ROOT=Path(__file__).resolve().parents[1]
assert ROOT.name=='CityInspection_GOC_R10'
OUT=Path('E:/CarlaAirData/CityInspection_GOC/perception_repair_20261023_r10')
OUT.mkdir(parents=True,exist_ok=True)


def change(file,old,new,count=1):
    p=ROOT/file;s=p.read_text(encoding='utf-8')
    if s.count(old)!=count:raise RuntimeError((file,'anchor count',s.count(old),old[:80]))
    p.write_text(s.replace(old,new),encoding='utf-8')


change('scripts/recovery_stage.py',"OUT=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')",f"OUT=Path('{OUT.as_posix()}')")
change('scripts/recovery_stage.py',"感知升级与36组复验_20261021_v2","问题修复与36组复验_R10_20261023")
file='scripts/run_stage1_dynamic.py'
change(file,'"bbox_extent_xyz": [float(extent.x), float(extent.y), float(extent.z)],',
'''"bbox_extent_xyz": [float(extent.x), float(extent.y), float(extent.z)],
        "bbox_location_xyz": s0.xyz(actor.bounding_box.location),
        "bbox_rotation_pyr": [actor.bounding_box.rotation.pitch, actor.bounding_box.rotation.yaw, actor.bounding_box.rotation.roll],
        "bbox_world_vertices_xyz": [s0.xyz(v) for v in actor.bounding_box.get_world_vertices(transform)],''')
change(file,'from perception.target_approach import TargetApproachMonitor','''from perception.body_approach import BodyApproachMonitor as TargetApproachMonitor
        from perception.body_clearance import observed_clearance, rectangle
        from safety.recovery_watchdog import ProgressWatchdog
        repair_watchdog=ProgressWatchdog()
        own_bbox=ugv.bounding_box
        own_body_polygon=rectangle([own_bbox.location.x,own_bbox.location.y],
            [own_bbox.extent.x,own_bbox.extent.y],own_bbox.rotation.yaw)''')
change(file,'                        candidate_pipeline_wall_ms = (time.perf_counter() - inference_started) * 1000.0',
'''                        # R10: same-frame geometry from permitted inputs only.
                        for candidate in candidates:
                            b=candidate['bbox_xyxy']
                            candidate['evidence_stage']='reliable_discovery_claim' if min(b[2]-b[0],b[3]-b[1])>=16 and candidate.get('temporal_confirmed') else 'tentative_candidate'
                            candidate['frame_id']=int(common_frame)
                            if device=='ugv':
                                candidate['body_clearance']=observed_clearance(depth_arrays[device],b,candidate.get('depth_m'),
                                    ugv_camera_to_vehicle,own_body_polygon,float(sensor_config['fov_degrees']))
                        candidate_pipeline_wall_ms = (time.perf_counter() - inference_started) * 1000.0''')
change(file,'                    within_standoff = 3.0 <= message_distance <= 7.0',
'''                    within_standoff = (3.0 <= message_distance <= 7.0) if oracle_mode else bool(
                        sensor_approach_monitor is not None and sensor_approach_monitor.completed)''')
change(file,'            frame = world.tick()\n            snapshot = world.get_snapshot()',
'''            recovery_action=repair_watchdog.update(sim_elapsed,s0.xyz(ugv.get_location())[:2],
                str(ugv_safety.get('ugv_safety_mode','')),bool(sensor_approach_monitor and sensor_approach_monitor.completed))
            if recovery_action=='reassess':
                # Replan from current own location and the existing sensor message.
                # This cannot bypass depth braking or invent an obstacle position.
                if persistent_route_active and business_reference_xyz is not None:
                    try:
                        active_ugv_route=plan_route_from_message(world_map,ugv.get_location(),business_reference_xyz,
                            standoff_m=float(persistent_target_cfg.get('route_standoff_m',6.0)))
                        persistent_route_cursor=0
                        persistent_route_events.append(dict(event='blocked_route_replanned',timestamp_s=sim_elapsed,
                            target_xyz=business_reference_xyz,route_xyz=active_ugv_route))
                    except (RuntimeError,ValueError) as exc:
                        persistent_route_events.append(dict(event='blocked_replan_rejected',timestamp_s=sim_elapsed,reason=str(exc)))
            if repair_watchdog.failed:
                ugv.apply_control(carla.VehicleControl(throttle=0.,brake=1.))
                ugv_safety['ugv_safety_mode']='bounded_blockage_failure'
            frame = world.tick()
            snapshot = world.get_snapshot()''')
change(file,'                    business_reference_xyz,\n                )\n            actual_uav',
'''                    business_reference_xyz,
                    legal_stop=world_map.get_waypoint(ugv.get_location(),project_to_road=False,
                        lane_type=carla.LaneType.Driving | carla.LaneType.Parking) is not None,
                )
            actual_uav''')
change(file,'            s0.json_dump(run_dirs["evaluation"] / "sensor_business_monitor.json", sensor_approach_monitor.summary())',
'''            s0.json_dump(run_dirs["evaluation"] / "sensor_business_monitor.json", sensor_approach_monitor.summary())
        s0.json_dump(run_dirs['logs']/'r10_recovery_watchdog.json',dict(failed=repair_watchdog.failed,events=repair_watchdog.events))
        if hazard_detector is not None:
            s0.json_dump(run_dirs['logs']/'r10_ground_vehicle_filter.json',dict(
                rejected=hazard_detector.rejected_ground_like_vehicles,
                examples=hazard_detector.vehicle_geometry_rejection_examples))''')
# Label tentative navigation explicitly. It is allowed exploration, not an S1 claim.
change('src/communication/persistent_target.py','            "candidate_id": str(candidate.get("candidate_id", "")),',
'''            "candidate_id": str(candidate.get("candidate_id", "")),
            "evidence_stage": str(candidate.get('evidence_stage','tentative_candidate')),''')
schema=ROOT/'configs/schemas/semantic_target_message_schema.json'
s=json.loads(schema.read_text(encoding='utf-8'));s.setdefault('properties',{})['evidence_stage']={'type':'string'}
schema.write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
# A shorter geometric lookahead follows the mapped curve rather than cutting it.
change(file,'    lookahead = min(len(route) - 1, nearest + max(1, int(lookahead_points)))',
'''    lookahead=nearest
    lookahead_distance=0.
    # 2m waypoint spacing previously produced a 12m chord across corners.
    # Select by metric distance and slow for upcoming route curvature.
    for j in range(nearest+1,min(len(route),nearest+max(2,int(lookahead_points))+1)):
        lookahead_distance+=horizontal_distance(route[j-1],route[j]);lookahead=j
        if lookahead_distance>=4.:break
    if nearest+3<len(route):
        a=np.asarray(route[nearest+1][:2])-np.asarray(route[nearest][:2])
        b=np.asarray(route[min(nearest+5,len(route)-1)][:2])-np.asarray(route[nearest+2][:2])
        if np.linalg.norm(a)>.1 and np.linalg.norm(b)>.1:
            turn=abs(wrap_angle_degrees(math.degrees(math.atan2(b[1],b[0])-math.atan2(a[1],a[0]))))
            if turn>20:target_speed=min(target_speed,2.)''')
# Keep the original application tree and frozen R9 unchanged.
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in ['src','scripts','configs','tests'] for p in (ROOT/folder).rglob('*')
        if p.is_file() and '__pycache__' not in p.parts}
(OUT/'initial_repair_manifest.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8')
print('R10 isolated edits applied',len(hashes),'files')

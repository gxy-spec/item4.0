from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
assert ROOT.name=='CityInspection_GOC_R10'
p=ROOT/'scripts/run_stage1_dynamic.py';s=p.read_text(encoding='utf-8')
anchor='def forward_obstacle_clearance('
helper='''def mapped_depth_corridor(vehicle, route):
    from safety.route_corridor import local_route
    pose=vehicle.get_transform()
    return local_route(route,dict(x=pose.location.x,y=pose.location.y,z=pose.location.z,
        yaw=pose.rotation.yaw,pitch=pose.rotation.pitch,roll=pose.rotation.roll))


'''
assert s.count(anchor)==1;s=s.replace(anchor,helper+anchor)
for old,new in [
('                    current_sim_time,\n                    ugv_camera_to_vehicle,\n                )',
 '                    current_sim_time,\n                    ugv_camera_to_vehicle,\n                    route_vehicle_xy=mapped_depth_corridor(ugv,active_ugv_route if persistent_route_active else route),\n                )'),
('                        local_timestamp,\n                        ugv_camera_to_vehicle,\n                    )',
 '                        local_timestamp,\n                        ugv_camera_to_vehicle,\n                        route_vehicle_xy=mapped_depth_corridor(ugv,active_ugv_route if persistent_route_active else route),\n                    )')]:
    assert s.count(old)==1,(old,s.count(old));s=s.replace(old,new)
p.write_text(s,encoding='utf-8')
p=ROOT/'scripts/run_recovery_campaign.py';s=p.read_text(encoding='utf-8')
start=s.index('    def free_budget(drive):');end=s.index('    potential=actual_root/',start)
s=s[:start]+'''    configured_root=configured_root.resolve()
    actual_root=configured_root
    if configured_root.anchor.upper() not in ['D:\\\\','E:\\\\']:
        raise RuntimeError('R10 formal data must resolve to D/E, never C: '+str(configured_root))
    # Resolve directory junctions before inspecting the physical drive.
    def free_budget(drive):return shutil.disk_usage(drive).free-5*1024**3
    if free_budget(configured_root.anchor)<estimated_raw_bytes:
        drive=max(['D:/','E:/'],key=free_budget)
        if free_budget(drive)<estimated_raw_bytes:raise RuntimeError('Insufficient complete-run payload +5GiB reserve; simulator not launched')
        actual_root=Path(drive)/'CI10'/cohort/p.stem
''' +s[end:]
p.write_text(s,encoding='utf-8')
print('CONNECTED R10 mapped corridor and D/E storage admission')

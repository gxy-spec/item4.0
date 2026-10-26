from pathlib import Path
import sys,json,subprocess,math
import carla
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,rows,simulator_lease
from run_e7_comparison_batch import restart_simulator
from src.navigation.approach_region import plan_observation_region


def main():
    progress=json.loads((OUT/'diagnostics_development_progress.json').read_text(encoding='utf-8'))
    run=Path(progress['upgraded_E2_C']['run_dir']);plan=rows(run/'planning/target_route_events.jsonl')[0]
    own=next(a for a in rows(run/'actors/actor_states.jsonl') if a['role']=='ugv')
    package=Path('D:/CarlaAir/CarlaAir-v0.1.7-Windows11-x86_64')
    from src.perception.body_clearance import rectangle
    poly=rectangle(own['bbox_location_xyz'][:2],own['bbox_extent_xyz'][:2],own['bbox_rotation_pyr'][1])
    with simulator_lease():
        cached=(OUT/'Town10HD.xodr').exists()
        if not cached:restart_simulator(package,'Town10HD','R10 endpoint geometry diagnostic')
        try:
            if cached:world_map=carla.Map('Town10HD',(OUT/'Town10HD.xodr').read_text(encoding='utf-8'))
            else:
                client=carla.Client('127.0.0.1',2000);client.set_timeout(40);world=client.get_world();world_map=world.get_map()
                (OUT/'Town10HD.xodr').write_text(world_map.to_opendrive(),encoding='utf-8')
            route=plan_observation_region(world_map,carla.Location(*plan['start_xyz']),plan['target_xyz'],poly)
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip(route,route[1:]))
            result=dict(original_route_m=plan['route_length_m'],new_approach_region_route_m=length,
                start_xyz=plan['start_xyz'],received_target_xyz=plan['target_xyz'],original_route=plan['route_xyz'],new_route=route,
                scope='Map/received point geometry diagnostic, not a closed-loop success')
            save(OUT/'diagnostics/E2_endpoint_region.json',result)
            print(json.dumps({k:v for k,v in result.items() if k not in ['original_route','new_route']},ensure_ascii=False),flush=True)
        finally:
            if not cached:subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(package/'CarlaAir.ps1'),'--kill'],cwd=package,capture_output=True,timeout=60)


if __name__=='__main__':main()

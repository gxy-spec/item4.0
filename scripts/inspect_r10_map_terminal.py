from pathlib import Path
import json,sys,math,carla
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,rows,save
from src.navigation.approach_region import terminal_geometry

progress=json.loads((OUT/'diagnostics_development_progress.json').read_text(encoding='utf-8'))
run=Path(progress['upgraded_E2_C']['run_dir']);plan=rows(run/'planning/target_route_events.jsonl')[0]
m=carla.Map('Town10HD',(OUT/'Town10HD.xodr').read_text(encoding='utf-8'))
target=plan['target_xyz'];result=[]
for w in m.generate_waypoints(1.):
    p=w.transform.location;distance=math.hypot(p.x-target[0],p.y-target[1])
    if distance>16:continue
    geometry,gap=terminal_geometry(p.x,p.y,w.transform.rotation.yaw,target,(-2.37,2.425,-1.082,1.082))
    result.append(dict(x=p.x,y=p.y,yaw=w.transform.rotation.yaw,is_junction=w.is_junction,
        road_id=w.road_id,lane_id=w.lane_id,width=w.lane_width,gap=gap,geometry=geometry,distance=distance))
save(OUT/'diagnostics/E2_nearby_lane_poses.json',result)
print(json.dumps(dict(total=len(result),geometry_eligible=sum(r['geometry'] for r in result),
    nonjunction_eligible=sum(r['geometry'] and not r['is_junction'] for r in result),
    sample=sorted(result,key=lambda r:r['distance'])[:8],eligible=[r for r in result if r['geometry']][:15]),indent=2))

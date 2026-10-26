"""Quantify raw drivable-footprint flags without treating every flag as collision."""
from pathlib import Path
import sys,json,math,argparse
import carla
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,rows,save


def audit(run):
    m=carla.Map('Town10HD',(OUT/'Town10HD.xodr').read_text(encoding='utf-8'))
    own=[a for a in rows(run/'actors/actor_states.jsonl') if a['role']=='ugv']
    events=[]
    for a in own:
        if a.get('map_drivable_footprint',a.get('legal_stop_footprint')) is not False:continue
        corners=[]
        for x,y in sorted({(round(v[0],4),round(v[1],4)) for v in a['bbox_world_vertices_xyz']}):
            point=carla.Location(x,y,a['z'])
            if m.get_waypoint(point,project_to_road=False,lane_type=carla.LaneType.Driving | carla.LaneType.Parking) is not None:continue
            w=m.get_waypoint(point,project_to_road=True,lane_type=carla.LaneType.Driving | carla.LaneType.Parking)
            if w is None:corners.append(dict(x=x,y=y,projection_available=False));continue
            p=w.transform.location;yaw=math.radians(w.transform.rotation.yaw)
            lateral=abs(-(x-p.x)*math.sin(yaw)+(y-p.y)*math.cos(yaw))
            adjacent=[]
            for side in ['left','right']:
                n=getattr(w,'get_'+side+'_lane')()
                if n is None or n.lane_type not in [carla.LaneType.Driving,carla.LaneType.Parking]:continue
                q=n.transform.location;neighbor_yaw=math.radians(n.transform.rotation.yaw)
                lat=abs(-(x-q.x)*math.sin(neighbor_yaw)+(y-q.y)*math.cos(neighbor_yaw))
                along=abs((x-q.x)*math.cos(neighbor_yaw)+(y-q.y)*math.sin(neighbor_yaw))
                adjacent.append(dict(side=side,road_id=n.road_id,lane_id=n.lane_id,lane_width_m=n.lane_width,
                    absolute_lateral_m=lat,longitudinal_offset_m=along,within_nominal_lateral_width=lat<=n.lane_width/2))
            corners.append(dict(x=x,y=y,road_id=w.road_id,lane_id=w.lane_id,lane_width_m=w.lane_width,
                absolute_lateral_m=lateral,nominal_lane_excess_m=lateral-w.lane_width/2,adjacent_driving_lanes=adjacent))
        events.append(dict(frame=a['frame'],timestamp=a['timestamp'],own_xy=[a['x'],a['y']],corners=corners))
    excess=[c['nominal_lane_excess_m'] for e in events for c in e['corners'] if c.get('nominal_lane_excess_m') is not None]
    result=dict(total_ticks=len(own),raw_false_ticks=len(events),events=events,
        maximum_nominal_lane_excess_m=max(excess,default=None),
        scope='Static map projection audit of own body corners; not obstacle collision or complete traffic-law certification; no outcome threshold adjusted')
    save(run/'r10_map_boundary_audit.json',result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-dir',required=True,type=Path);args=p.parse_args()
    print(json.dumps(audit(args.run_dir),ensure_ascii=False,indent=2))

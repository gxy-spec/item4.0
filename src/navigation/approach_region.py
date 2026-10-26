"""Forward legal-lane search to an observable approach region, not a hidden car.

Only the received point, map, own pose and known own footprint are inputs.
Selecting one nearest lane can require a full one-way circuit even though the
target is visible from a nearby reachable lane. Search all admissible terminal
poses together instead; braking/completion still require local RGB-D evidence.
"""
from __future__ import annotations
import math,heapq,itertools


def terminal_geometry(x,y,yaw,target_xy,own_bounds,minimum_gap=5.,maximum_gap=7.):
    a=math.radians(yaw);dx,dy=target_xy[0]-x,target_xy[1]-y
    forward=dx*math.cos(a)+dy*math.sin(a);lateral=-dx*math.sin(a)+dy*math.cos(a)
    xmin,xmax,ymin,ymax=own_bounds
    gap=math.hypot(max(xmin-forward,0.,forward-xmax),max(ymin-lateral,0.,lateral-ymax))
    visible=forward>0 and abs(math.degrees(math.atan2(lateral,forward)))<=45.
    return visible and minimum_gap<=gap<=maximum_gap,gap


def plan_observation_region(world_map,start_location,target_xyz,own_polygon,resolution=2.,audit=None):
    import carla
    current=world_map.get_waypoint(start_location,project_to_road=True,lane_type=carla.LaneType.Driving)
    if current is None:raise RuntimeError('Own start cannot be projected to a driving lane')
    xmin=min(p[0] for p in own_polygon);xmax=max(p[0] for p in own_polygon)
    ymin=min(p[1] for p in own_polygon);ymax=max(p[1] for p in own_polygon)
    bounds=(xmin,xmax,ymin,ymax)
    def key(w):return (w.road_id,w.section_id,w.lane_id,int(round(w.s/max(.5,resolution/2))))
    def h(w):
        p=w.transform.location
        return max(0.,math.hypot(p.x-target_xyz[0],p.y-target_xyz[1])-11.)
    def terminal(w):
        # User-approved temporary stopped confirmation may be inside a map
        # junction. This is reported separately, not called legal road parking.
        p=w.transform.location;yaw=w.transform.rotation.yaw
        ok,_=terminal_geometry(p.x,p.y,yaw,target_xyz,bounds)
        if not ok:return False
        a=math.radians(yaw);c,s=math.cos(a),math.sin(a)
        for x,y in own_polygon:
            corner=carla.Location(p.x+x*c-y*s,p.y+x*s+y*c,p.z)
            if world_map.get_waypoint(corner,project_to_road=False,lane_type=carla.LaneType.Driving | carla.LaneType.Parking) is None:return False
        return True
    def successors(w):
        for n in w.next(resolution):yield n,None
        if w.is_junction:return
        # The old compatibility planner had forward edges only. Respect map
        # lane-change permissions and same-direction lanes, with a smooth
        # 12m transition wholly outside junctions.
        for side in ['left','right']:
            if side not in str(w.lane_change).lower() and 'both' not in str(w.lane_change).lower():continue
            adjacent=getattr(w,'get_'+side+'_lane')()
            if adjacent is None or adjacent.lane_type!=carla.LaneType.Driving:continue
            error=abs((adjacent.transform.rotation.yaw-w.transform.rotation.yaw+180)%360-180)
            if error>20:continue
            edge=[];end=None
            for d in [2.,4.,6.,8.,10.,12.]:
                aa=w.next(d);bb=adjacent.next(d)
                if len(aa)!=1 or len(bb)!=1 or aa[0].is_junction or bb[0].is_junction:edge=[];break
                allowed=str(aa[0].lane_change).lower()
                if side not in allowed and 'both' not in allowed:edge=[];break
                t=d/12.;blend=t*t*(3-2*t)
                a=aa[0].transform.location;b=bb[0].transform.location
                edge.append([float(a.x+(b.x-a.x)*blend),float(a.y+(b.y-a.y)*blend),float(a.z+(b.z-a.z)*blend)])
                end=bb[0]
            if edge:yield end,dict(side=side,points=edge,start_road=w.road_id,start_lane=w.lane_id,end_lane=end.lane_id)
    start=key(current);waypoints={start:current};parents={start:None};costs={start:0.};edges={}
    order=itertools.count();frontier=[(h(current),next(order),0.,start)];goal=None
    for _ in range(50000):
        if not frontier:break
        _,_,g,k=heapq.heappop(frontier)
        if g>costs[k]+1e-6:continue
        w=waypoints[k]
        if terminal(w):goal=k;break
        for following,edge in successors(w):
            nk=key(following)
            if edge:
                origin=w.transform.location;pts=[[origin.x,origin.y,origin.z],*edge['points']]
                step=sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))
            else:step=float(w.transform.location.distance(following.transform.location))
            ng=g+step
            if ng>=costs.get(nk,float('inf')):continue
            costs[nk]=ng;parents[nk]=k;waypoints[nk]=following
            edges[nk]=edge
            heapq.heappush(frontier,(ng+h(following),next(order),ng,nk))
    if goal is None:raise RuntimeError('No reachable legal forward-facing approach region for received point')
    chain=[];k=goal;changes=[]
    while k is not None:
        p=waypoints[k].transform.location
        edge=edges.get(k)
        chain.append(edge['points'] if edge else [[float(p.x),float(p.y),float(p.z)]])
        if edge:changes.append(edge)
        k=parents[k]
    points=[p for segment in reversed(chain) for p in segment]
    if len(points)==1:points=[[start_location.x,start_location.y,start_location.z],*points]
    if audit is not None:audit.update(method='legal_lane_graph_to_observable_approach_region',lane_changes=list(reversed(changes)),
        endpoint_in_junction=bool(waypoints[goal].is_junction),endpoint_rule='temporary stopped confirmation on drivable footprint; user approved junction annotation',
        target_source='received sensor-derived point only',own_bounds=list(bounds))
    return points

"""Swept centerline support for depth occupancy, from map route and own odometry."""
import numpy as np


def local_route(route,own_state,maximum_range_m=20.):
    from .rgbd_hazard_perception import pose_matrix
    if not route:return None
    inverse=np.linalg.inv(pose_matrix(own_state))
    p=(inverse@np.column_stack([np.asarray(route)[:,:3],np.ones(len(route))]).T).T[:,:2]
    nearest=int(np.argmin(np.linalg.norm(p,axis=1)))
    selected=[];length=0.;previous=np.zeros(2)
    for xy in p[nearest:]:
        if xy[0]<-.5 and not selected:continue
        length+=float(np.linalg.norm(xy-previous));selected.append(xy);previous=xy
        if length>=maximum_range_m:break
    if not selected:return None
    return np.asarray([np.zeros(2),*selected])


def corridor_mask(points_xy,route_xy,half_width_m):
    points=np.asarray(points_xy);route=np.asarray(route_xy)
    minimum=np.full(len(points),np.inf)
    for a,b in zip(route,route[1:]):
        delta=b-a
        if float(delta@delta)<1e-8:continue
        t=np.clip(((points-a)@delta)/float(delta@delta),0.,1.)
        minimum=np.minimum(minimum,np.linalg.norm(points-a-t[:,None]*delta,axis=1))
    return minimum<=half_width_m

"""Metric geometry. Online calls accept only depth, calibration and own geometry.

Target polygons are accepted by the independent evaluator, never the controller.
"""
from __future__ import annotations

import math
import numpy as np


def rectangle(center_xy, extent_xy, yaw_degrees=0.):
    a=math.radians(yaw_degrees);c,s=math.cos(a),math.sin(a)
    return [[center_xy[0]+x*c-y*s,center_xy[1]+x*s+y*c]
            for x,y in [(-extent_xy[0],-extent_xy[1]),(extent_xy[0],-extent_xy[1]),
                        (extent_xy[0],extent_xy[1]),(-extent_xy[0],extent_xy[1])]]


def points_to_polygon_distance(points, polygon):
    import cv2
    p=np.asarray(points,dtype=np.float64)[:,:2]
    poly=np.asarray(polygon,dtype=np.float64)
    distances=np.full(len(p),np.inf)
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        ab=b-a;t=np.clip(((p-a)@ab)/max(float(ab@ab),1e-12),0.,1.)
        distances=np.minimum(distances,np.linalg.norm(p-a-t[:,None]*ab,axis=1))
    for i,point in enumerate(p):
        if cv2.pointPolygonTest(poly.astype(np.float32),tuple(point),False)>=0:distances[i]=0.
    return distances


def polygon_clearance(a,b):
    """Minimum distance of convex body projections, including edge crossings."""
    import cv2
    aa=np.asarray(a,dtype=np.float32);bb=np.asarray(b,dtype=np.float32)
    area,_=cv2.intersectConvexConvex(aa,bb)
    if area>0:return 0.
    return float(min(points_to_polygon_distance(aa,bb).min(),points_to_polygon_distance(bb,aa).min()))


def observed_clearance(depth,box,reference_depth_m,camera_to_vehicle,own_polygon,fov_degrees=100.):
    """Estimate gap to a coherent, elevated visible target surface.

    A partial visible surface is not a complete target body. The caller must
    validate viewpoint coverage independently; returned diagnostics expose
    clipping, depth spread and support rather than inventing hidden geometry.
    """
    if reference_depth_m is None:return {'valid':False,'reason':'missing_reference_depth'}
    h,w=depth.shape;x1,y1,x2,y2=map(float,box)
    left,right=max(0,int(x1)),min(w,int(math.ceil(x2)))
    top,bottom=max(0,int(y1)),min(h,int(math.ceil(y2)))
    if right<=left or bottom<=top:return {'valid':False,'reason':'empty_box'}
    stride=max(1,int(min(right-left,bottom-top)/40))
    vv,uu=np.mgrid[top:bottom:stride,left:right:stride]
    d=depth[top:bottom:stride,left:right:stride].reshape(-1).astype(float)
    u,v=uu.reshape(-1),vv.reshape(-1)
    band=max(.35,min(1.5,float(reference_depth_m)*.035))
    # A truck's central pixel can look through an open bed or window. Keep
    # nearer supported surfaces instead of restricting to that central depth.
    valid=np.isfinite(d)&(d>.3)&(d<=min(45.,float(reference_depth_m)+band))
    if valid.sum()<12:return {'valid':False,'reason':'insufficient_depth_support','support':int(valid.sum())}
    d,u,v=d[valid],u[valid],v[valid]
    focal=w/(2*math.tan(math.radians(fov_degrees)/2))
    camera=np.column_stack([d,(u-w/2)*d/focal,-(v-h/2)*d/focal,np.ones(len(d))])
    local=(np.asarray(camera_to_vehicle)@camera.T).T[:,:3]
    local=local[(local[:,2]>.12)&(local[:,2]<4.5)]
    if len(local)<12:return {'valid':False,'reason':'no_elevated_surface','support':len(local)}
    gaps=points_to_polygon_distance(local,own_polygon)
    gap=float(np.percentile(gaps,2))
    support_point=local[int(np.argmin(abs(gaps-gap)))]
    clipped=left<=1 or right>=w-1 or top<=1 or bottom>=h-1
    # A very near surface contradicting a distant central target cue is an
    # occlusion/association conflict. Do not label the intervening tree or wall
    # as the target, or silently discard it and announce a distant safe gap.
    conflict=gap<1. and float(reference_depth_m)>6.
    return {'valid':not (clipped or conflict),'reason':'clipped_surface' if clipped else ('foreground_depth_conflict' if conflict else 'observed_target_surface'),
            'estimated_body_clearance_m':gap,
            'surface_horizontal_distance_m':float(np.linalg.norm(support_point[:2]-np.asarray(camera_to_vehicle)[:2,3])),
            'observed_surface_vehicle_xyz':support_point.tolist(),
            'optical_depth_m':float(reference_depth_m),'support':len(local),
            'surface_depth_span_m':float(np.ptp(local[:,0])),
            'definition':'own body to observed elevated target surface; calibration required',
            'source':'RGB-D + camera extrinsics + known own body only'}

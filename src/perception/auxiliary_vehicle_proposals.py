"""Complementary generic vehicle boxes; fine types remain learned separately."""
import numpy as np


def suppress_contained_vehicle_parts(proposals,depth):
    def area(b):return max(0,b[2]-b[0])*max(0,b[3]-b[1])
    def optical_depth(b):
        h,w=depth.shape;x1,y1,x2,y2=map(int,b);dx=max(1,(x2-x1)//4);dy=max(1,(y2-y1)//4)
        z=depth[max(0,y1+dy):min(h,y2-dy),max(0,x1+dx):min(w,x2-dx)]
        z=z[np.isfinite(z)&(z>.5)&(z<150)]
        return float(np.median(z)) if len(z)>=4 else None
    keep=[]
    for small in proposals:
        a=small['bbox_xyxy'];discard=False
        for large in proposals:
            if large is small or not large['proposal_source'].endswith('_coco_aux') or large['detector_score']<.15:continue
            b=large['bbox_xyxy']
            if area(b)<area(a)*1.5:continue
            intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
            if intersection/max(1,area(a))<.85:continue
            da,db=optical_depth(a),optical_depth(b)
            # A genuinely closer foreground vehicle is a separate object.
            if da is not None and db is not None and da>=db-1.5:discard=True;break
        if not discard:keep.append(small)
    return keep

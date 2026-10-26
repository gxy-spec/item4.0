"""Conservative connected-depth completion of a partial vehicle proposal.

No semantic ground truth is accepted. A proposal is changed only when one
bounded, depth-consistent component covers its existing visible mask and
extends it. Road planes, clipped components and excessive growth are rejected.
"""
import cv2,numpy as np


def refine_depth_proposal(depth, proposal):
    box=proposal['bbox_xyxy'];x1,y1,x2,y2=[int(round(v)) for v in box]
    h,w=depth.shape;bw=x2-x1;bh=y2-y1
    if bw<6 or bh<6:return proposal
    left=max(0,int(x1-.65*bw));right=min(w,int(x2+.65*bw))
    top=max(0,int(y1-1.8*bh));bottom=min(h,int(y2+1.8*bh))
    if right<=left or bottom<=top:return proposal
    seed=np.zeros((bottom-top,right-left),np.uint8)
    original_mask=proposal.get('_instance_mask')
    if original_mask is not None:
        seed[:]=original_mask[top:bottom,left:right]
        seed[:max(0,y1-top)]=0;seed[max(0,y2-top):]=0
        seed[:,:max(0,x1-left)]=0;seed[:,max(0,x2-left):]=0
    else:
        a=max(0,x1-left+bw//4);b=min(right-left,x2-left-bw//4)
        c=max(0,y1-top+bh//4);d=min(bottom-top,y2-top-bh//4);seed[c:d,a:b]=1
    patch=depth[top:bottom,left:right];valid=np.isfinite(patch)&(patch>.5)&(patch<150)
    values=patch[(seed>0)&valid]
    if len(values)<12:return proposal
    centre=float(np.median(values));tolerance=max(.45,min(1.2,centre*.018))
    foreground=(valid&(np.abs(patch-centre)<=tolerance)).astype(np.uint8)
    count,labels,stats,_=cv2.connectedComponentsWithStats(foreground,8)
    overlap=np.bincount(labels[(seed>0)&valid],minlength=count);overlap[0]=0
    chosen=int(overlap.argmax())
    if not chosen or overlap[chosen]<.7*len(values):return proposal
    px,py,pw,ph,area=stats[chosen];new=[left+int(px),top+int(py),left+int(px+pw),top+int(py+ph)]
    # Boundary contact indicates a component whose completeness is unknown.
    if px==0 or py==0 or px+pw==right-left or py+ph==bottom-top:return proposal
    # Preserve validated detector pixels (e.g. wheels at a different depth)
    # rather than requiring them all to lie in the same rear-face plane.
    new=[min(new[0],x1),min(new[1],y1),max(new[2],x2),max(new[3],y2)]
    nw,nh=new[2]-new[0],new[3]-new[1]
    ratio=(nw*nh)/(bw*bh)
    if not 1.12<=ratio<=3.2:return proposal
    if area<.4*nw*nh:return proposal
    mask=np.zeros((h,w),bool);mask[top:bottom,left:right]=labels==chosen
    mask[top:bottom,left:right] |= seed>0
    return {**proposal,'bbox_xyxy':[float(v) for v in new],'_instance_mask':mask,
            'detector_bbox_xyxy':list(box),'depth_refinement':'bounded_connected_depth_component_v1',
            'depth_component_reference_m':centre}

import numpy as np
from src.perception.depth_proposal_refinement import refine_depth_proposal


def test_partial_rear_box_completes_without_actor_truth():
    depth=np.full((150,160),50.);depth[35:100,55:105]=6.
    mask=np.zeros_like(depth,bool);mask[70:98,57:103]=True
    p={'bbox_xyxy':[57.,70.,103.,98.],'_instance_mask':mask}
    r=refine_depth_proposal(depth,p)
    assert r['bbox_xyxy']==[55.,35.,105.,100.]
    assert r['detector_bbox_xyxy']==p['bbox_xyxy']


def test_road_plane_or_unbounded_component_cannot_expand_box():
    depth=np.full((150,160),6.);p={'bbox_xyxy':[57.,70.,103.,98.]}
    assert refine_depth_proposal(depth,p) is p


def test_adjacent_different_depth_object_is_not_merged():
    depth=np.full((150,160),50.);depth[35:100,55:105]=6.;depth[35:100,105:130]=9.
    p={'bbox_xyxy':[57.,70.,103.,98.]};r=refine_depth_proposal(depth,p)
    assert r['bbox_xyxy'][2]==105.


def test_wheels_at_different_depth_do_not_prevent_rear_face_completion():
    depth=np.full((150,160),50.);depth[35:96,55:105]=6.;depth[96:101,57:103]=8.
    mask=np.zeros_like(depth,bool);mask[70:101,57:103]=True
    p={'bbox_xyxy':[57.,70.,103.,101.],'_instance_mask':mask}
    r=refine_depth_proposal(depth,p)
    assert r['bbox_xyxy']==[55.,35.,105.,101.]
    assert r['_instance_mask'][100,60]

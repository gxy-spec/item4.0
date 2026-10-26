import numpy as np
from src.perception.body_clearance import rectangle,observed_clearance


def test_near_foreground_with_distant_target_depth_cannot_claim_target_gap():
    depth=np.full((100,100),12.);depth[40:60,30:45]=2.5
    mount=np.eye(4);mount[2,3]=1.5
    r=observed_clearance(depth,[30,40,70,60],12.,mount,rectangle([0,0],[2,1]))
    assert not r['valid'] and r['reason']=='foreground_depth_conflict'

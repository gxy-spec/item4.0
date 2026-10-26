import numpy as np
from src.safety.route_corridor import local_route,corridor_mask


def test_curved_corridor_rejects_only_outside_obstacles():
    route=np.array([[0,0],[4,0],[6,2],[6,6]])
    points=[[3,0],[6,5],[9,0],[3,3],[5,2]]
    assert corridor_mask(points,route,1.2).tolist()==[True,True,False,False,True]


def test_route_transform_uses_only_own_odometry():
    result=local_route([[10,10,0],[10,15,0],[10,20,0]],dict(x=10,y=10,z=0,yaw=90))
    assert np.allclose(result[-1],[10,0])

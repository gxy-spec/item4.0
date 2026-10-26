import numpy as np
from src.safety.rgbd_hazard_perception import vehicle_depth_support


def test_flat_road_is_rejected_but_missing_depth_keeps_hazard():
    h,w=200,200;fx=100.;mount=np.eye(4);mount[2,3]=1.5
    vv=np.arange(h)[:,None];road=np.broadcast_to(1.5*fx/np.maximum(vv-99.5,.1),(h,w)).copy()
    accepted,e=vehicle_depth_support(road,[40,120,160,190],mount,0,fx)
    assert not accepted and e['verdict']=='flat_ground_rejected'
    assert vehicle_depth_support(np.full_like(road,np.nan),[40,120,160,190],mount,0,fx)[0]


def test_low_vehicle_and_upright_obstacle_are_retained():
    mount=np.eye(4);mount[2,3]=1.5
    for box in [[40,80,160,130],[40,105,160,125]]:
        assert vehicle_depth_support(np.full((200,200),5.),box,mount,0,100.)[0]

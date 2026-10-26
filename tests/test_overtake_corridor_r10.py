from types import SimpleNamespace
import numpy as np
from scripts.run_stage1_dynamic import mapped_depth_corridor,effective_control_route
from src.safety.route_corridor import corridor_mask


def test_guard_tracks_actual_passing_lane_and_keeps_real_passing_lane_obstacle():
    base=[[float(x),0.,0.] for x in range(101)]
    maneuver=dict(start_s_m=40.,end_s_m=95.,side_offset_m=-3.5)
    class Vehicle:
        def get_transform(self):
            return SimpleNamespace(location=SimpleNamespace(x=65.,y=-3.5,z=0.),rotation=SimpleNamespace(yaw=0.,pitch=0.,roll=0.))
    points=np.array([[5.,3.5],[5.,0.]]) # old-lane car; actual passing-lane obstacle
    old=mapped_depth_corridor(Vehicle(),base)
    new=mapped_depth_corridor(Vehicle(),base,maneuver)
    assert corridor_mask(points,old,1.6).tolist()==[True,False]
    assert corridor_mask(points,new,1.6).tolist()==[False,True]
    assert effective_control_route(base,maneuver)[65][1]==-3.5

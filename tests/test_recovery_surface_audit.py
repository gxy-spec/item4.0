import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_recovery_campaign import distance_outside_actor_footprint


def test_long_bus_rear_surface_is_valid_without_being_near_geometric_centre():
    bus=dict(xyz=[10,20,0],extent_xy=[6,1.3],yaw_degrees=90)
    assert distance_outside_actor_footprint([10,25.8,2],bus)==0
    assert abs(distance_outside_actor_footprint([10,27,2],bus)-1)<1e-6
    assert distance_outside_actor_footprint([14,25.8,2],bus)>2

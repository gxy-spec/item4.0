"""The regional terminal must not stop early or override safety priority."""
from types import SimpleNamespace as NS
import sys
import pytest
from scripts.run_stage1_dynamic import follow_route,apply_safe_route_control


class Own:
    def get_location(self):return NS(x=0.,y=0.,z=0.)
    def get_transform(self):return NS(location=self.get_location(),rotation=NS(yaw=0.))
    def get_velocity(self):return NS(x=0.,y=0.,z=0.)
    def apply_control(self,c):self.control=c


def test_regional_endpoint_tolerance_changes_only_end_braking(monkeypatch):
    monkeypatch.setitem(sys.modules,'carla',NS(VehicleControl=lambda **kw:NS(**kw)))
    own=Own();route=[[0.,0.,0.],[2.,0.,0.]]
    follow_route(own,route,2.)
    assert own.control.brake==1.
    follow_route(own,route,2.,end_stop_tolerance_m=.5)
    assert own.control.throttle>0 and own.control.brake==0


@pytest.mark.parametrize('state,hazard',[('STOP',{}),('CLEAR',{'forced_stop_reason':'pedestrian'})])
def test_short_endpoint_never_bypasses_depth_or_pedestrian(monkeypatch,state,hazard):
    monkeypatch.setitem(sys.modules,'carla',NS(VehicleControl=lambda **kw:NS(**kw)))
    own=Own()
    result=apply_safe_route_control(own,[[0.,0.,0.],[2.,0.,0.]],2.,None,[],
        dict(enabled=True,obstacle_source='ugv_rgbd',target_stop_enabled=False,route_end_stop_tolerance_m=.5),
        sensor_observation=dict(state=state,nearest_obstacle_m=1.),hazard_decision=hazard,
        target_xyz=[10.,0.,0.])
    assert own.control.brake==1. and own.control.throttle==0.
    assert result['ugv_safety_mode']!='route_complete'

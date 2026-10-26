import numpy as np
import pytest
from src.perception.body_clearance import rectangle,polygon_clearance,observed_clearance
from src.safety.recovery_watchdog import ProgressWatchdog


@pytest.mark.parametrize('extent',[2.,3.,4.,6.])
@pytest.mark.parametrize('yaw',[0.,30.,90.])
@pytest.mark.parametrize('gap',[2.5,5.,8.5])
def test_polygon_gap_is_independent_of_target_length(extent,yaw,gap):
    own=rectangle([0,0],[2,1]);target=rectangle([0,0],[extent,1],yaw)
    # Put the nearest target vertex opposite the middle of the own front edge.
    # An x-only shift leaves a lateral offset for long rotated vehicles.
    vertex=min(target,key=lambda p:p[0])
    shifted=[[p[0]+2+gap-vertex[0],p[1]-vertex[1]] for p in target]
    assert polygon_clearance(own,shifted)==pytest.approx(gap,abs=1e-5)


def test_overlap_is_zero_even_when_edges_cross():
    assert polygon_clearance(rectangle([0,0],[4,.5]),rectangle([0,0],[4,.5],90))==0


def test_surface_estimator_rejects_empty_depth_and_clipped_target():
    own=rectangle([0,0],[2,1]);mount=np.eye(4);mount[2,3]=1.5
    assert not observed_clearance(np.full((100,100),np.nan),[20,20,80,80],5,mount,own)['valid']
    assert not observed_clearance(np.full((100,100),7.),[0,20,70,80],7,mount,own)['valid']
    value=observed_clearance(np.full((100,100),7.),[30,40,70,60],7,mount,own)
    assert value['valid'] and value['estimated_body_clearance_m']==pytest.approx(5,abs=.01)


def test_watchdog_does_not_disable_safety_and_has_bounded_failure():
    w=ProgressWatchdog();assert w.update(0,[0,0],'wait_vehicle') is None
    assert w.update(8,[0,0],'wait_vehicle')=='reassess'
    assert w.update(36,[0,0],'wait_vehicle')=='failed'
    assert w.events[-1]['event']=='bounded_blockage_failure'
    w=ProgressWatchdog();w.update(0,[0,0],'wait_vehicle')
    assert w.update(40,[2,0],'wait_vehicle') is None

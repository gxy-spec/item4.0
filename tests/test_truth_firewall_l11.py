import pytest
from src.perception.truth_firewall import DeniedActor,TruthAccessDenied


@pytest.mark.parametrize('name',['get_location','get_transform','bounding_box','attributes','type_id','id'])
def test_target_and_obstacle_truth_is_not_readable(name):
    events=[];proxy=DeniedActor(object(),'target',events)
    with pytest.raises(TruthAccessDenied):getattr(proxy,name)
    assert len(events)==1 and events[0]['denied']

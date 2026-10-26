import numpy as np
from src.perception.auxiliary_vehicle_proposals import suppress_contained_vehicle_parts


def test_whole_bus_suppresses_part_but_preserves_nearer_vehicle():
    whole=dict(bbox_xyxy=[10,10,90,90],proposal_source='yolo26x_coco_aux',detector_score=.9)
    part=dict(bbox_xyxy=[60,70,80,80],proposal_source='custom',detector_score=.8)
    z=np.full((100,100),10.)
    assert suppress_contained_vehicle_parts([whole,part],z)==[whole]
    z[70:80,60:80]=5.
    assert suppress_contained_vehicle_parts([whole,part],z)==[whole,part]

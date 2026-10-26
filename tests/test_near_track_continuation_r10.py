from src.perception.body_approach import BodyApproachMonitor
from test_body_approach_r10 import candidate


def test_near_semantically_confirmed_object_can_be_tracked_through_unknown_crop():
    m=BodyApproachMonitor()
    for i in range(3):m.observe([candidate()],i*.25,[10,0,1]);m.update(i*.25,0,[10,0,1])
    unknown={**candidate(),'category':'unknown','color':'unknown','target_color_match':False,'category_match_score':0.}
    for i in range(3,15):m.observe([unknown],i*.25,[10,0,1]);m.update(i*.25,0,[10,0,1])
    assert m.completed and m.latest['observed_category']=='unknown' and m.latest['category']=='van'
    fresh=BodyApproachMonitor()
    for i in range(15):fresh.observe([unknown],i*.25,[10,0,1]);fresh.update(i*.25,0,[10,0,1])
    assert not fresh.confirmed


def test_return_of_semantic_prediction_does_not_break_an_existing_visible_track():
    m=BodyApproachMonitor()
    unknown={**candidate(),'category':'unknown','color':'unknown','target_color_match':False,'category_match_score':0.}
    for i in range(14):
        c=unknown if i in [4,5,6] else candidate()
        m.observe([c],i*.25,[10,0,1]);m.update(i*.25,0,[10,0,1])
    assert m.completed


def test_incompatible_identity_or_disconnected_crop_cannot_continue_confirmation():
    for bad in [dict(color='blue'),dict(category='bus'),dict(world_position_xyz=[15,0,1]),dict(bbox_xyxy=[100,100,160,160])]:
        m=BodyApproachMonitor()
        for i in range(3):m.observe([candidate()],i*.25,[10,0,1])
        wrong={**candidate(),**bad,'target_color_match':False,'category_match_score':0.}
        for i in range(3,15):m.observe([wrong],i*.25,[10,0,1]);m.update(i*.25,0,[10,0,1])
        assert not m.completed

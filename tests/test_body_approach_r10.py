from src.perception.body_approach import BodyApproachMonitor


def candidate(gap=5.,side=40,range_m=8.,**extra):
    return dict(candidate_id='c',frame_id=1,category='van',color='red',target_color_match=True,
        category_match_score=1.,depth_m=7.,world_position_xyz=[10.,0.,1.],bbox_xyxy=[10,10,10+side,10+side],
        body_clearance=dict(valid=True,estimated_body_clearance_m=gap,surface_horizontal_distance_m=range_m),**extra)


def test_strict_near_confirmation_and_hold():
    m=BodyApproachMonitor()
    for i in range(14):
        t=i*.25;m.observe([candidate()],t,[10,0,1]);m.update(t,0,[10,0,1])
    assert m.confirmed and m.completed


def test_far_or_tiny_observations_do_not_confirm():
    for c in [candidate(side=8),candidate(range_m=65)]:
        m=BodyApproachMonitor()
        for i in range(15):m.observe([c],i*.25,[10,0,1]);m.update(i*.25,0,[10,0,1])
        assert not m.confirmed and not m.completed


def test_stale_moving_wrong_target_and_gap_cannot_complete():
    for gap,speed,ref,legal in [(2.5,0,[10,0,1],True),(8.,0,[10,0,1],True),
                               (5.,1.,[10,0,1],True),(5.,0,[30,0,1],True),(5.,0,[10,0,1],False)]:
        m=BodyApproachMonitor()
        for i in range(18):
            t=i*.25;m.observe([candidate(gap)],t,ref);m.update(t,speed,ref,legal)
        assert not m.completed
    m=BodyApproachMonitor()
    for i in range(3):m.observe([candidate()],i*.25,[10,0,1])
    for i in range(3,30):m.update(i*.25,0,[10,0,1])
    assert not m.completed


def test_only_distinct_fresh_observations_count_in_hold():
    m=BodyApproachMonitor()
    for i in range(3):m.observe([candidate()],i*.25,[10,0,1])
    for i in range(30):m.update(.5+i*.05,0,[10,0,1])
    assert not m.completed

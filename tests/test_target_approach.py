from src.perception.target_approach import TargetApproachMonitor


def candidate(**extra):
    return dict(candidate_id='c1',temporal_confirmed=True,target_color_match=True,
                category_match_score=1.,depth_m=5.,world_position_xyz=[10.,0.,1.],**extra)


def test_fresh_confirmed_observations_and_stationary_hold_required():
    m=TargetApproachMonitor()
    for i in range(26):
        t=i*.1;m.observe([candidate()],t,[10.,0.,1.]);m.update(t,.1,[10.,0.,1.])
    assert m.confirmed and m.completed


def test_old_ugv_evidence_and_wrong_memory_cannot_complete():
    m=TargetApproachMonitor();m.observe([candidate()],0.,[10.,0.,1.])
    for t in [2.,3.,4.]:assert not m.update(t,0.,[10.,0.,1.])
    for i in range(30):
        t=5+i*.1;m.observe([candidate()],t,[30.,0.,1.]);assert not m.update(t,0.,[30.,0.,1.])


def test_moving_robot_does_not_count_as_stopped_approach():
    m=TargetApproachMonitor()
    for i in range(40):
        t=i*.1;m.observe([candidate()],t,[10.,0.,1.]);assert not m.update(t,2.,[10.,0.,1.])


def test_surface_braking_requires_fresh_confirmed_same_target():
    m=TargetApproachMonitor()
    c={**candidate(),'depth_m':4.4}
    assert not m.braking_request(0.,[10.,0.,1.])
    m.observe([c],0.,[10.,0.,1.])
    assert m.braking_request(.25,[10.,0.,1.])
    assert not m.braking_request(2.,[10.,0.,1.])
    assert not m.braking_request(.25,[30.,0.,1.])
    assert not m.completed


def test_braking_does_not_relax_three_metre_completion_minimum():
    m=TargetApproachMonitor()
    for i in range(40):
        t=i*.1;m.observe([{**candidate(),'depth_m':2.984}],t,[10.,0.,1.])
        assert m.braking_request(t,[10.,0.,1.])
        assert not m.update(t,0.,[10.,0.,1.])
    assert len([e for e in m.events if e['event']=='sensor_target_braking_started'])==1


def test_target_farther_than_braking_threshold_keeps_route_control():
    m=TargetApproachMonitor();m.observe([candidate()],0.,[10.,0.,1.])
    assert not m.braking_request(.25,[10.,0.,1.])


def test_confirmed_target_switch_releases_previous_completion_hold():
    m=TargetApproachMonitor()
    for i in range(25):
        t=i*.1;m.observe([candidate()],t,[10.,0.,1.]);m.update(t,0.,[10.,0.,1.])
    assert m.completed
    assert not m.update(3.,0.,[30.,0.,1.])
    assert not m.completed and not m.confirmed


def test_uncertain_crop_continues_only_previously_confirmed_geometric_identity():
    m=TargetApproachMonitor()
    c=candidate(category='car',color='yellow',bbox_xyxy=[10,10,100,100])
    m.observe([c],0.,[10.,0.,1.]);m.update(0.,0.,[10.,0.,1.])
    uncertain={**c,'candidate_id':'c2','category':'unknown','color':'unknown',
               'temporal_confirmed':False,'category_match_score':0.,'target_color_match':False}
    for i in range(1,26):
        m.observe([uncertain],i*.1,[10.,0.,1.]);m.update(i*.1,0.,[10.,0.,1.])
    assert m.completed and m.latest['observed_category']=='unknown'
    assert m.latest['category']=='car'
    fresh=TargetApproachMonitor();fresh.observe([uncertain],0.,[10.,0.,1.])
    assert not fresh.confirmed
    m=TargetApproachMonitor();m.observe([c],0.,[10.,0.,1.])
    for bad in [{**uncertain,'world_position_xyz':[15.,0.,1.]},
                {**uncertain,'category':'bus'}, {**uncertain,'color':'blue'},
                {**uncertain,'bbox_xyxy':[150,150,200,200]}]:
        m.observe([bad],.2,[10.,0.,1.]);assert m.latest['candidate_id']=='c1'

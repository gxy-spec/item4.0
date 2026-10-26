from copy import deepcopy
import pytest
from src.perception.strict_completion import verify_completion


def box(x):return [[x+dx,dy,dz] for dx in [-2,2] for dy in [-1,1] for dz in [0,2]]


def fixture():
    own=[dict(frame=i,timestamp=i*.05,speed_mps=0.,map_drivable_footprint=True,bbox_world_vertices_xyz=box(0)) for i in range(41)]
    target={i:dict(actor_id=42,bbox_world_vertices_xyz=box(9)) for i in range(41)}
    obs=[dict(timestamp_s=i*.25,frame_id=i*5,candidate_id=str(i),identity_correct=True,matched_actor_id=42,
              estimated_body_clearance_m=5.) for i in range(1,9)]
    return dict(event=dict(hold_start_s=0.,timestamp_s=2.),own_states=own,target_states=target,local_observations=obs,
                origin_s=0.,expected_actor_id=42,episode_collisions=0,s1_pass=True,s2_pass=True)


def test_real_full_hold_is_accepted():assert verify_completion(**fixture())['passed']


@pytest.mark.parametrize('fault',['short_hold','missing_first','missing_last','internal_gap','duplicate_tick','duplicate_observation',
    'stale_observation','wrong_instance','wrong_body_identity','no_discovery','no_confirmation','collision','negative_task',
    'moving','too_close','too_far','missing_geometry','nonfinite_geometry','unknown_road','invalid_estimate'])
def test_false_success_is_rejected(fault):
    a=fixture()
    if fault=='short_hold':a['event']['timestamp_s']=1.9
    elif fault=='missing_first':a['own_states']=a['own_states'][1:]
    elif fault=='missing_last':a['own_states']=a['own_states'][:-1]
    elif fault=='internal_gap':a['own_states'].pop(20)
    elif fault=='duplicate_tick':a['own_states'].insert(20,deepcopy(a['own_states'][20]))
    elif fault=='duplicate_observation':a['local_observations'][4]=deepcopy(a['local_observations'][3])
    elif fault=='stale_observation':a['local_observations']=a['local_observations'][:2]+a['local_observations'][-2:]
    elif fault=='wrong_instance':a['local_observations'][2]['matched_actor_id']=99
    elif fault=='wrong_body_identity':a['target_states'][15]['actor_id']=99
    elif fault=='no_discovery':a['s1_pass']=False
    elif fault=='no_confirmation':a['s2_pass']=False
    elif fault=='collision':a['episode_collisions']=1
    elif fault=='negative_task':a['expected_present']=False
    elif fault=='moving':a['own_states'][20]['speed_mps']=.31
    elif fault=='too_close':a['target_states'][20]['bbox_world_vertices_xyz']=box(6.9)
    elif fault=='too_far':a['target_states'][20]['bbox_world_vertices_xyz']=box(12.1)
    elif fault=='missing_geometry':a['target_states'][20].pop('bbox_world_vertices_xyz')
    elif fault=='nonfinite_geometry':a['target_states'][20]['bbox_world_vertices_xyz'][0][0]=float('nan')
    elif fault=='unknown_road':a['own_states'][20]['map_drivable_footprint']=None
    elif fault=='invalid_estimate':a['local_observations'][2]['estimated_body_clearance_m']=float('nan')
    assert not verify_completion(**a)['passed']

"""Budget-route decisions preserve existing routes unless a shorter region fits."""
from src.navigation.budgeted_approach import observed_budget_speed,select_task_route


def history(speed=1.7):
    return [dict(timestamp=i*.05,ugv_speed_mps=speed,ugv_safety_mode='cruise') for i in range(201)]


def choose(original,candidate,remaining,nominal=2.,past=None):
    called=[]
    def planner(*a,**kw):called.append(True);return candidate
    r,a=select_task_route(None,None,[50,0,1],[[-2,-1],[2,-1],[2,1],[-2,1]],original,
        remaining,nominal,history() if past is None else past,10.,candidate_planner=planner)
    return r,a,called


def test_mature_own_speed_accounts_for_controller_steady_error():
    v,a=observed_budget_speed(history(),2.,10.)
    assert abs(v-1.7)<1e-9 and a['mature']


def test_startup_acceleration_cannot_define_permanent_slow_budget_speed():
    v,a=observed_budget_speed(history(.7)[:20],2.,1.)
    assert v==2. and not a['mature']


def test_waiting_or_future_data_not_used_as_cruise_speed():
    rows=history()+[dict(timestamp=11,ugv_speed_mps=.5,ugv_safety_mode='cruise')]*50
    rows +=[dict(timestamp=8,ugv_speed_mps=0.,ugv_safety_mode='wait_pedestrian')]*50
    assert observed_budget_speed(rows,2.,10.)[0]==1.7


def test_feasible_original_is_returned_identically_without_query():
    old=[[0,0,0],[100,0,0]];r,a,called=choose(old,[[0,0,0],[50,0,0]],190.)
    assert r is old and not called and not a['accepted'] and a['chosen_end_stop_tolerance_m']==3.


def test_far_plan_replaced_only_by_shorter_budget_feasible_region():
    old=[[0,0,0],[760,0,0]];new=[[0,0,0],[275,0,0]]
    r,a,c=choose(old,new,405.)
    assert r is new and c and a['accepted'] and a['chosen_end_stop_tolerance_m']==.5


def test_no_feasible_candidate_does_not_change_existing_control():
    old=[[0,0,0],[712,0,0]];r,a,c=choose(old,[[0,0,0],[676,0,0]],143.)
    assert r is old and c and not a['accepted'] and a['chosen_end_stop_tolerance_m']==3.


def test_query_error_keeps_original_route():
    old=[[0,0,0],[760,0,0]]
    def bad(*a,**k):raise RuntimeError('No legal terminal')
    r,a=select_task_route(None,None,[10,0,0],[],old,405.,2.,history(),10.,candidate_planner=bad)
    assert r is old and not a['accepted'] and a['query_error']=='No legal terminal'

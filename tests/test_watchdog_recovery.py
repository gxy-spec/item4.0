"""Blocked road clearing later must recover; stale/unsafe evidence never clears it."""
from src.safety.recovery_watchdog import ProgressWatchdog


def failed():
    w=ProgressWatchdog();w.update(0,[0,0],'wait_no_safe_overtake_gap')
    assert w.update(36,[0,0],'wait_no_safe_overtake_gap')=='failed'
    return w


def test_blocked_attempt_resumes_only_after_continuous_fresh_clear():
    w=failed()
    assert w.update(100,[0,0],'cruise',safe_to_resume=True)=='failed'
    assert w.update(100.5,[0,0],'cruise',safe_to_resume=True)=='failed'
    assert w.update(101,[0,0],'cruise',safe_to_resume=True)=='resumed'
    assert not w.failed and w.ever_failed
    assert [e['event'] for e in w.events][-1]=='blocked_route_resumed_after_fresh_clear'
    assert w.update(101.1,[.1,0],'cruise',safe_to_resume=True) is None


def test_clear_mode_without_new_sensor_evidence_never_releases_hold():
    w=failed()
    for t in [100,102,120]:assert w.update(t,[0,0],'cruise')=='failed'
    assert w.failed


def test_new_hazard_interrupts_clear_interval():
    w=failed();w.update(100,[0,0],'cruise',safe_to_resume=True)
    assert w.update(100.8,[0,0],'wait_pedestrian',safe_to_resume=False)=='failed'
    assert w.update(101,[0,0],'cruise',safe_to_resume=True)=='failed'
    assert w.update(101.9,[0,0],'cruise',safe_to_resume=True)=='failed'
    assert w.update(102,[0,0],'cruise',safe_to_resume=True)=='resumed'


def test_permanent_block_is_still_bounded_failure():
    w=failed()
    for t in [50,100,190]:assert w.update(t,[0,0],'wait_no_safe_overtake_gap',safe_to_resume=True)=='failed'
    assert w.failed and len([e for e in w.events if e['event']=='bounded_blockage_failure'])==1

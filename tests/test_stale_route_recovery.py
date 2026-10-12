from src.communication.persistent_target import stale_route_recovery_action


def test_expired_message_keeps_a_bounded_coast_window() -> None:
    assert stale_route_recovery_action("coast_then_patrol", 1.0, 2.0, 5.0, 8.0) == "coast"
    assert stale_route_recovery_action("coast_then_patrol", 5.0, 2.0, 5.0, 8.0) == "resume_patrol"
    assert stale_route_recovery_action("coast_then_patrol", 1.0, 8.0, 5.0, 8.0) == "resume_patrol"


def test_legacy_immediate_return_and_legacy_hold_actions_remain_explicit() -> None:
    assert stale_route_recovery_action("return_to_patrol", 0.0, 0.0, 5.0, 8.0) == "return_to_patrol"
    assert stale_route_recovery_action("hold_after_grace", 4.9, 2.0, 5.0, 8.0) == "coast"
    assert stale_route_recovery_action("hold_after_grace", 5.0, 2.0, 5.0, 8.0) == "safe_hold"

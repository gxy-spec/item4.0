from src.communication.persistent_target import PersistentTargetSelector


def candidate(x: float = 0.0) -> dict:
    return {
        "candidate_id": "candidate-1",
        "color": "red",
        "color_score": 0.9,
        "detector_score": 0.8,
        "depth_m": 15.0,
        "depth_reliability": 0.8,
        "world_position_xyz": [x, 1.0, 0.5],
        "temporal_confirmed": True,
    }


def test_repeated_messages_are_sequenced_and_selected_from_both_sources() -> None:
    selector = PersistentTargetSelector(delay_s=0.05, ttl_s=1.0)
    uav1 = selector.send("uav", 10, 0.5, candidate())
    ugv1 = selector.send("ugv", 10, 0.5, candidate(0.2))
    assert uav1["sequence_number"] == ugv1["sequence_number"] == 1
    assert len(selector.advance(0.55)) == 2
    selected = selector.select_and_log(0.55)
    assert selected is not None
    assert selected["source"] in {"uav", "ugv"}
    uav2 = selector.send("uav", 20, 1.0, candidate())
    assert uav2["sequence_number"] == 2
    assert len(selector.advance(1.05)) == 1


def test_duplicate_out_of_order_and_expired_messages_do_not_replace_fresh_state() -> None:
    selector = PersistentTargetSelector(delay_s=0.0, ttl_s=1.0)
    fresh = selector.send("uav", 10, 0.5, candidate())
    selector.advance(0.5)
    duplicate = dict(fresh)
    duplicate["receive_timestamp_s"] = 0.6
    selector.pending.append(duplicate)
    old = dict(fresh)
    old["sequence_number"] = 0
    old["message_id"] = "old"
    old["receive_timestamp_s"] = 0.6
    selector.pending.append(old)
    expired = selector.send("ugv", 11, 0.5, candidate(0.3))
    expired["expires_at_s"] = 0.55
    expired["receive_timestamp_s"] = 0.6
    selector.pending[-1]["expires_at_s"] = 0.55
    selector.advance(0.6)
    events = [event["event"] for event in selector.events]
    assert "rejected_duplicate_or_out_of_order" in events
    assert "rejected_expired" in events
    assert selector.select(0.6)["message_id"] == fresh["message_id"]
    assert selector.select(2.0) is None


def test_cross_device_consensus_and_continuity_reject_far_red_false_candidates() -> None:
    selector = PersistentTargetSelector(
        delay_s=0.0,
        ttl_s=1.0,
        consensus_radius_m=8.0,
        maximum_position_jump_m=12.0,
        require_cross_device_consensus=True,
    )
    selector.send("uav", 10, 0.5, candidate(0.0))
    selector.advance(0.5)
    assert selector.select_and_log(0.5) is None
    selector.send("ugv", 10, 0.5, candidate(2.0))
    selector.advance(0.5)
    first = selector.select_and_log(0.5)
    assert first is not None
    selector.send("uav", 20, 0.8, candidate(80.0))
    selector.advance(0.8)
    assert selector.select_and_log(0.8) is None


def test_source_policy_can_isolate_ugv_local_control() -> None:
    selector = PersistentTargetSelector(delay_s=0.0, ttl_s=1.0)
    selector.send("uav", 10, 0.5, candidate(0.0))
    selector.send("ugv", 10, 0.5, candidate(0.2))
    selector.advance(0.5)
    selected = selector.select(0.5, allowed_sources={"ugv"})
    assert selected is not None
    assert selected["source"] == "ugv"
    assert selector.select(0.5, allowed_sources=set()) is None


def test_messages_queued_during_outage_are_dropped_not_buffered_as_fresh() -> None:
    selector = PersistentTargetSelector(delay_s=0.05, ttl_s=1.0)
    selector.send("uav", 10, 0.5, candidate())
    assert selector.advance(0.55, delivery_enabled=False) == []
    assert selector.select(0.55) is None
    assert any(event["event"] == "dropped_during_outage" for event in selector.events)

from src.communication.persistent_target import PersistentTargetMemory, PersistentTargetSelector
from src.perception.task_spec import TaskSpec


def test_task_conditioned_blue_car_admitted_but_wrong_category_rejected() -> None:
    task = TaskSpec.from_task_config({"task_id": "blue-car", "parsed_goal": {
        "target": {"category": "car", "attributes": {"color": "blue"}}}})
    selector = PersistentTargetSelector(delay_s=0.0, task_spec=task)
    memory = PersistentTargetMemory(task_spec=task)
    good = {**candidate(), "category": "car", "color": "blue", "target_color": "blue", "target_color_match": True}
    selector.send("uav", 1, 0.0, good)
    received = selector.advance(0.0)[0]
    assert received["category"] == "car"
    assert received["logical_target_id"] == "blue-car"
    assert selector.select(0.0) is not None
    assert memory.consider(received, 0.0) == "initial"
    wrong = selector.send("ugv", 2, 0.1, {**good, "category": "van"})
    assert memory.consider(wrong, 0.1) == "rejected"
    selector.advance(0.1)
    assert selector.select(0.1, {"ugv"}) is None


def test_task_conditioned_van_does_not_inherit_coco_car_fallback() -> None:
    task = TaskSpec.legacy_red_van()
    selector = PersistentTargetSelector(delay_s=0.0, task_spec=task)
    selector.send("uav", 1, 0.0, {**candidate(), "category": "car", "target_color_match": True})
    selector.advance(0.0)
    assert selector.select(0.0) is None


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


def test_external_arrival_uses_radio_time_without_old_fixed_delay() -> None:
    selector = PersistentTargetSelector(delay_s=0.05, ttl_s=1.0)
    message = selector.send("uav", 10, 0.5, candidate(), external_delivery=True)
    assert not selector.pending
    assert message["receive_timestamp_s"] is None
    received = selector.receive_external({**message, "received_at_s": 1.49})
    assert received is not None
    assert received["receive_timestamp_s"] == 1.49
    assert selector.events[-1]["event"] == "received"
    assert selector.receive_external({**message, "received_at_s": 1.49}) is None
    late = selector.send("uav", 11, 2.0, candidate(), external_delivery=True)
    assert selector.receive_external({**late, "received_at_s": 3.0}) is None


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


def test_memory_admission_consensus_is_optional_and_uses_fresh_other_device() -> None:
    selector = PersistentTargetSelector(delay_s=0.0, ttl_s=1.0, require_cross_device_consensus=True)
    selector.send("uav", 1, 0.0, candidate())
    first = selector.advance(0.0)[0]
    assert not selector.corroborated_by_other_source(first, 0.0, {"uav", "ugv"})
    assert selector.select(0.0) is None
    selector.send("ugv", 2, 0.1, candidate(0.3))
    second = selector.advance(0.1)[0]
    assert selector.corroborated_by_other_source(second, 0.1, {"uav", "ugv"})
    assert not selector.corroborated_by_other_source(second, 0.1, {"ugv"})
    assert not selector.corroborated_by_other_source(second, 1.0, {"uav", "ugv"})


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


def test_expired_other_source_cannot_corroborate_fresh_message() -> None:
    selector = PersistentTargetSelector(
        delay_s=0.0,
        ttl_s=1.0,
        require_cross_device_consensus=True,
        consensus_radius_m=8.0,
    )
    selector.send("uav", 1, 0.0, candidate(0.0))
    selector.advance(0.0)
    selector.send("ugv", 2, 0.9, candidate(0.2))
    selector.advance(0.9)
    assert selector.select(1.0) is None
    assert set(selector.latest_by_source) == {"ugv"}


def test_message_expires_exactly_at_source_timestamp_plus_ttl() -> None:
    selector = PersistentTargetSelector(delay_s=0.0, ttl_s=1.0)
    selector.send("uav", 1, 2.0, candidate())
    selector.advance(2.0)
    assert selector.select(2.999) is not None
    assert selector.select(3.0) is None


def test_message_arriving_at_ttl_boundary_is_rejected() -> None:
    selector = PersistentTargetSelector(delay_s=1.0, ttl_s=1.0)
    selector.send("uav", 1, 2.0, candidate())
    assert selector.advance(3.0) == []
    assert selector.select(3.0) is None
    assert any(event["event"] == "rejected_expired" for event in selector.events)


def memory_message(source: str, number: int, x: float, now_s: float, score: float = 0.7) -> dict:
    return {
        "message_id": f"{source}_{number}", "source": source,
        "source_timestamp_s": now_s, "expires_at_s": now_s + 1.0,
        "class_hypothesis": "vehicle", "color": "red", "depth_valid": True,
        "world_position_xyz": [x, 0.0, 0.5], "selection_score": score,
    }


def test_target_memory_survives_packet_ttl_and_rejects_isolated_challenger() -> None:
    memory = PersistentTargetMemory()
    assert memory.consider(memory_message("uav", 1, 0, 0), 0.05) == "initial"
    assert memory.current["world_position_xyz"][0] == 0
    assert memory.consider(memory_message("ugv", 1, 80, 8), 8.05) == "challenger"
    assert memory.current["world_position_xyz"][0] == 0
    assert memory.consider(memory_message("uav", 2, 1, 9), 9.05) == "refresh"
    assert not memory.challenger
    assert memory.consider(memory_message("uav", 3, 80, 10), 11.0) == "rejected"


def test_target_memory_switch_needs_repeated_cross_device_high_margin_evidence() -> None:
    memory = PersistentTargetMemory()
    assert memory.consider(memory_message("uav", 1, 0, 0), 0.05) == "initial"
    assert memory.consider(memory_message("uav", 2, 80, 3, 0.85), 3.05) == "challenger"
    assert memory.consider(memory_message("uav", 3, 81, 3.5, 0.85), 3.55) == "challenger"
    assert memory.consider(memory_message("ugv", 1, 80, 4, 0.86), 4.05) == "switched"
    assert memory.current["world_position_xyz"][0] >= 80
    assert sum(e["event"] == "memory_confirmed_target_switch" for e in memory.events) == 1

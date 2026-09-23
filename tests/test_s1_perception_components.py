from __future__ import annotations

from src.communication.semantic_channel import SemanticChannel
from src.perception.target_ranker import (
    LocalTargetVerifier,
    LocalVerificationConfig,
    TargetRanker,
    TransmissionGateConfig,
)
from src.task.perception_state_machine import PerceptionTaskStateMachine


def candidate(**overrides):
    value = {
        "candidate_id": "10_00",
        "bbox_xyxy": [10.0, 10.0, 60.0, 35.0],
        "category": "truck",
        "proposal_source": "yolo26x_coco",
        "color": "red",
        "color_score": 1.0,
        "red_pixel_ratio": 0.78,
        "detector_score": 0.55,
        "temporal_hits": 4,
        "world_position_xyz": [-39.5, -41.0, 1.5],
    }
    value.update(overrides)
    return value


def test_transmission_gate_rejects_red_only_regions_and_accepts_stable_detector_candidate():
    ranker = TargetRanker(TransmissionGateConfig())
    assert ranker.select([candidate(proposal_source="instruction_guided_red_region")]) is None
    selected = ranker.select([candidate()])
    assert selected is not None
    assert selected["track_id"].startswith("uav_track_")


def test_semantic_channel_is_explicitly_non_oracle_and_delayed_one_tick():
    channel = SemanticChannel(delay_ticks=1)
    sent = channel.send({"target_world_position_xyz": [1.0, 2.0, 3.0]}, 10, 100, 5.0)
    assert sent is not None and sent["oracle"] is False
    assert channel.receive(10, 100, 5.0) is None
    received = channel.receive(11, 101, 5.05)
    assert received is not None
    assert received["message_type"] == "target_candidate_v1"
    assert abs(received["latency_ms"] - 50.0) < 1e-6


def test_local_verifier_requires_position_color_temporal_and_van_like_evidence():
    verifier = LocalTargetVerifier(LocalVerificationConfig())
    result = verifier.evaluate(20, 1.0, [candidate()], [-39.2, -41.1, 1.5])
    assert result["confirmed"] is True
    rejected = verifier.evaluate(21, 1.1, [candidate(color="other")], [-39.2, -41.1, 1.5])
    assert rejected["confirmed"] is False


def test_s1_state_machine_reaches_completed_through_local_confirmation():
    machine = PerceptionTaskStateMachine("task", "instruction")
    transitions = [
        "SEARCHING",
        "UAV_CANDIDATE_CONFIRMED",
        "MESSAGE_SENT",
        "TARGET_RECEIVED",
        "PLANNING_TO_OBSERVATION_POINT",
        "NAVIGATING",
    ]
    for index, state in enumerate(transitions):
        machine.transition(state, index, index * 0.1, "test")
    machine.start_local_confirmation(7, 0.7)
    machine.mark_target_verified(8, 0.8)
    machine.mark_ugv_arrived(9, 0.9)
    machine.mark_uav_route_complete(10, 1.0)
    assert machine.transitions_valid is True
    assert machine.state == "COMPLETED"

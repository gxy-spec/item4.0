"""Boundary cases where a wrong transport event changes task evidence."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from communication.a2g_transport import A2GTransportConfig, A2GTransportQueue  # noqa: E402


def config(**changes: object) -> A2GTransportConfig:
    mapping = yaml.safe_load((ROOT / "configs/communication/a2g_transport_v1.yaml").read_text(
        encoding="utf-8"
    ))["transport_link"]
    return replace(A2GTransportConfig.from_mapping(mapping, bandwidth_hz=10e6), **changes)


def sample_queue(*, rate_cap_bit_s: float = 800.0, ttl_s: float = 1.0,
                 processing_s: float = 0.0, propagation_speed_m_s: float = 299_792_458.0) -> A2GTransportQueue:
    queue = A2GTransportQueue(config(rate_cap_bit_s=rate_cap_bit_s, ttl_s=ttl_s,
                                     processing_s=processing_s,
                                     propagation_speed_m_s=propagation_speed_m_s), "geo")
    queue.update_channel(0.0, snr_db=30.0, distance_m=10.0)
    return queue


def probe(queue: A2GTransportQueue, name: str, byte_count: int, *, observed: float = 0.0,
          enqueued: float = 0.0) -> None:
    queue.enqueue({"message_id": name, "source_timestamp_s": observed},
                  enqueue_timestamp_s=enqueued, payload_bytes_override=byte_count)


def test_fcfs_subtick_completion_and_propagation_do_not_hold_transmitter() -> None:
    queue = sample_queue()
    probe(queue, "first", 40)
    probe(queue, "second", 20)
    queue.advance_to(0.8)
    assert [item.message_id for item in queue.records] == ["first", "second"]
    assert queue.records[0].t_finish == pytest.approx(0.4)
    assert queue.records[1].t_start == pytest.approx(0.4)
    assert queue.records[1].t_finish == pytest.approx(0.6)
    assert queue.records[0].t_arr > queue.records[0].t_finish
    assert [item["message_id"] for item in queue.take_deliveries()] == ["first", "second"]
    assert queue.summary()["transmitted_bits"] == pytest.approx(480.0)
    assert queue.summary()["mean_aoi_after_first_delivery_s"] == pytest.approx(0.6)


def test_pause_keeps_progress_and_expiration_counts_partially_sent_bits() -> None:
    queue = sample_queue(ttl_s=0.8)
    probe(queue, "held", 80)
    queue.update_channel(0.25, snr_db=0.0, distance_m=10.0)
    assert queue.records[0].sent_bits == pytest.approx(200.0)
    queue.update_channel(0.75, snr_db=30.0, distance_m=10.0)
    queue.advance_to(0.8)
    assert queue.records[0].status == "expired_in_service"
    assert queue.records[0].sent_bits == pytest.approx(240.0)
    assert queue.take_deliveries() == []
    assert [item["event"] for item in queue.events].count("service_paused") == 1
    assert [item["event"] for item in queue.events].count("service_resumed") == 1


def test_completion_before_expiry_but_information_available_at_boundary_is_expired() -> None:
    queue = sample_queue(processing_s=0.8, propagation_speed_m_s=100.0)
    probe(queue, "late_parse", 10)
    queue.advance_to(1.1)
    message = queue.records[0]
    assert message.t_finish == pytest.approx(0.1)
    assert message.t_arr == pytest.approx(1.0)
    assert message.status == "expired_after_transmission"
    assert queue.take_deliveries() == []


def test_transmission_reaching_ttl_exactly_is_cancelled() -> None:
    queue = sample_queue()
    probe(queue, "exact", 100)
    queue.advance_to(1.0)
    assert queue.records[0].status == "expired_in_service"
    assert queue.records[0].t_finish is None
    assert queue.records[0].sent_bits == pytest.approx(800.0)


def test_expired_waiter_does_not_block_later_fresh_message() -> None:
    queue = sample_queue()
    probe(queue, "long", 200)
    probe(queue, "stale_waiter", 20)
    probe(queue, "fresh", 10, observed=0.9, enqueued=0.9)
    queue.advance_to(1.2)
    assert [item.status for item in queue.records] == [
        "expired_in_service", "expired_in_queue", "available_valid"
    ]
    assert queue.records[2].t_start == pytest.approx(1.0)
    assert queue.records[2].t_finish == pytest.approx(1.1)


def test_variable_rate_integral_and_episode_censoring() -> None:
    queue = sample_queue(rate_cap_bit_s=10e6, ttl_s=3.0)
    queue.update_channel(0.0, snr_db=6.0, distance_m=10.0)
    first_rate = queue.rate_bit_s
    probe(queue, "large", 100_000)
    queue.update_channel(0.1, snr_db=12.0, distance_m=10.0)
    second_rate = queue.rate_bit_s
    assert second_rate > first_rate
    expected_finish = 0.1 + (800_000 - first_rate * 0.1) / second_rate
    queue.advance_to(expected_finish + 0.01)
    assert queue.records[0].t_finish == pytest.approx(expected_finish)
    assert queue.summary()["delivered_valid"] == 1
    probe(queue, "unfinished", 1_000_000, observed=queue.now_s, enqueued=queue.now_s)
    queue.end_episode(queue.now_s + 0.02)
    assert queue.records[1].status == "episode_censored"
    assert queue.records[1].sent_bits > 0


def test_serialization_excludes_receiver_and_log_fields() -> None:
    cfg = config()
    payload = {name: "value" for name in cfg.sender_fields}
    payload["message_id"] = "m1"
    payload["source_timestamp_s"] = 0.0
    payload.update({"status": "sent", "receive_timestamp_s": None,
                    "event": "sent", "oracle_target_truth": [1, 2, 3]})
    sender, encoded = cfg.serialize_sender(payload)
    assert set(sender) == set(cfg.sender_fields)
    assert b"oracle_target_truth" not in encoded
    assert b"receive_timestamp_s" not in encoded
    assert math.isfinite(cfg.rate_bit_s(20.0))


def test_mbps_semantic_packet_finishes_despite_sub_bit_roundoff() -> None:
    queue = sample_queue(rate_cap_bit_s=2_000_000.0)
    probe(queue, "semantic", 704)
    queue.advance_to(0.05)
    assert queue.records[0].status == "available_valid"
    assert queue.records[0].t_finish == pytest.approx(704 * 8 / 2_000_000)
    assert [item["message_id"] for item in queue.take_deliveries()] == ["semantic"]


def test_large_simulation_timestamp_does_not_false_expire_fast_semantic_packet() -> None:
    queue = sample_queue(rate_cap_bit_s=100_000_000.0)
    queue.update_channel(269.0, snr_db=30.0, distance_m=120.0)
    probe(queue, "late_in_episode", 687, observed=269.296528361738,
          enqueued=269.346528362483)
    queue.advance_to(270.4)
    assert queue.records[0].status == "available_valid"
    assert queue.records[0].t_arr < queue.records[0].t_exp

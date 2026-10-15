"""Single-transmitter A2G message service with exact simulated event times.

The radio rate is left-held between channel samples. This module has no CARLA
dependency and never reads a future channel sample or wall-clock time.
"""

from __future__ import annotations

import heapq
import json
import math
import statistics
from dataclasses import dataclass, field
from typing import Any


_EPS = 1e-10


def _bit_tolerance(total_bits: float) -> float:
    """Absorb floating-point integration error, never a meaningful payload bit."""
    return max(1e-7, total_bits * 1e-10)


@dataclass(frozen=True)
class A2GTransportConfig:
    bandwidth_hz: float
    snr_working_threshold_db: float
    snr_gap_db: float
    resource_fraction: float
    spectral_efficiency_cap_bit_s_hz: float
    rate_cap_bit_s: float
    ttl_s: float
    header_bytes: int
    processing_s: float
    propagation_speed_m_s: float
    sender_fields: tuple[str, ...]

    @classmethod
    def from_mapping(cls, mapping: dict[str, Any], *, bandwidth_hz: float) -> "A2GTransportConfig":
        service = mapping["service"]
        queue = mapping["queue"]
        receiver = mapping["receiver"]
        wire = mapping["wire"]
        config = cls(
            bandwidth_hz=float(bandwidth_hz),
            snr_working_threshold_db=float(service["snr_working_threshold_db"]),
            snr_gap_db=float(service["snr_gap_db"]),
            resource_fraction=float(service["resource_fraction"]),
            spectral_efficiency_cap_bit_s_hz=float(service["spectral_efficiency_cap_bit_s_hz"]),
            rate_cap_bit_s=float(service["rate_cap_bit_s"]),
            ttl_s=float(queue["ttl_s"]),
            header_bytes=int(wire["explicit_header_bytes"]),
            processing_s=float(receiver["semantic_processing_s"]),
            propagation_speed_m_s=float(receiver["propagation_speed_m_s"]),
            sender_fields=tuple(str(name) for name in wire["sender_fields"]),
        )
        positive = ("bandwidth_hz", "spectral_efficiency_cap_bit_s_hz", "rate_cap_bit_s",
                    "ttl_s", "propagation_speed_m_s")
        if any(not math.isfinite(getattr(config, key)) or getattr(config, key) <= 0 for key in positive):
            raise ValueError("Transport bandwidth, ceilings, TTL, and propagation speed must be positive")
        if not 0 < config.resource_fraction <= 1 or config.snr_gap_db < 0:
            raise ValueError("Invalid resource fraction or SNR gap")
        if config.header_bytes < 0 or config.processing_s < 0 or not config.sender_fields:
            raise ValueError("Invalid header, receiver processing, or sender field contract")
        if len(set(config.sender_fields)) != len(config.sender_fields):
            raise ValueError("Sender fields must be unique")
        return config

    def rate_bit_s(self, snr_db: float) -> float:
        if not math.isfinite(snr_db):
            raise ValueError("SNR must be finite")
        if snr_db < self.snr_working_threshold_db:
            return 0.0
        gamma = 10.0 ** (snr_db / 10.0)
        gap = 10.0 ** (self.snr_gap_db / 10.0)
        spectral = min(math.log2(1.0 + gamma / gap), self.spectral_efficiency_cap_bit_s_hz)
        return min(self.rate_cap_bit_s, self.resource_fraction * self.bandwidth_hz * spectral)

    def serialize_sender(self, payload: dict[str, Any]) -> tuple[dict[str, Any], bytes]:
        missing = [key for key in self.sender_fields if key not in payload]
        if missing:
            raise ValueError(f"A2G sender fields missing: {missing}")
        sender = {key: payload[key] for key in self.sender_fields}
        encoded = json.dumps(sender, sort_keys=True, separators=(",", ":"),
                             ensure_ascii=False, allow_nan=False).encode("utf-8")
        return sender, encoded


@dataclass
class _Message:
    payload: dict[str, Any]
    message_id: str
    t_obs: float
    t_enq: float
    t_exp: float
    payload_bytes: int
    header_bytes: int
    total_bits: float
    sent_bits: float = 0.0
    t_start: float | None = None
    t_finish: float | None = None
    t_arr: float | None = None
    status: str = "enqueued"


@dataclass
class A2GTransportQueue:
    config: A2GTransportConfig
    group: str
    now_s: float | None = None
    snr_db: float | None = None
    distance_m: float | None = None
    rate_bit_s: float = 0.0
    waiting: list[_Message] = field(default_factory=list)
    active: _Message | None = None
    arrivals: list[tuple[float, int, _Message]] = field(default_factory=list)
    records: list[_Message] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    _deliveries: list[dict[str, Any]] = field(default_factory=list)
    _arrival_number: int = 0
    _latest_observation_s: float | None = None

    def _event(self, kind: str, message: _Message | None = None, **fields: Any) -> None:
        item = {"event": kind, "timestamp_s": self.now_s, "group": self.group,
                "snr_db": self.snr_db, "rate_bit_s": self.rate_bit_s,
                "distance_m": self.distance_m, "queue_length": len(self.waiting)}
        if message is not None:
            item.update({"message_id": message.message_id,
                         "source_timestamp_s": message.t_obs,
                         "enqueue_timestamp_s": message.t_enq,
                         "expires_at_s": message.t_exp,
                         "payload_bytes": message.payload_bytes,
                         "header_bytes": message.header_bytes,
                         "total_bits": message.total_bits,
                         "sent_bits": message.sent_bits})
        item.update(fields)
        self.events.append(item)

    def _start_next(self) -> bool:
        if self.active is not None or not self.waiting:
            return False
        message = self.waiting.pop(0)
        if message.t_exp <= self.now_s:
            message.status = "expired_in_queue"
            self._event(message.status, message)
            return True
        message.t_start = self.now_s
        message.status = "in_service"
        self.active = message
        self._event("service_started", message)
        if self.rate_bit_s == 0.0:
            self._event("service_paused", message)
        return True

    def _process_now(self) -> bool:
        changed = False
        while self.arrivals and self.arrivals[0][0] <= self.now_s + _EPS:
            _, _, message = heapq.heappop(self.arrivals)
            message.status = ("available_valid" if message.t_arr < message.t_exp
                              else "expired_after_transmission")
            self._event(message.status, message, arrival_timestamp_s=message.t_arr)
            if message.status == "available_valid":
                if self._latest_observation_s is None or message.t_obs > self._latest_observation_s:
                    self._latest_observation_s = message.t_obs
                self._deliveries.append({**message.payload,
                                         "source_timestamp_s": message.t_obs,
                                         "expires_at_s": message.t_exp,
                                         "received_at_s": message.t_arr,
                                         "receive_timestamp_s": message.t_arr,
                                         "status": "received",
                                         "a2g_payload_bytes": message.payload_bytes,
                                         "a2g_sent_bits": message.sent_bits,
                                         "a2g_group": self.group})
            changed = True

        remaining: list[_Message] = []
        for message in self.waiting:
            if message.t_exp <= self.now_s + _EPS:
                message.status = "expired_in_queue"
                self._event(message.status, message)
                changed = True
            else:
                remaining.append(message)
        self.waiting = remaining

        if self.active is not None:
            message = self.active
            if message.t_exp <= self.now_s + _EPS:
                message.status = "expired_in_service"
                self._event(message.status, message)
                self.active = None
                changed = True
            elif message.total_bits - message.sent_bits <= _bit_tolerance(message.total_bits):
                message.sent_bits = message.total_bits
                message.t_finish = self.now_s
                message.t_arr = (self.now_s + self.distance_m / self.config.propagation_speed_m_s
                                 + self.config.processing_s)
                message.status = "transmission_finished"
                self._event(message.status, message, finish_timestamp_s=message.t_finish,
                            arrival_timestamp_s=message.t_arr)
                self._arrival_number += 1
                heapq.heappush(self.arrivals, (message.t_arr, self._arrival_number, message))
                self.active = None
                changed = True
        if self._start_next():
            changed = True
        return changed

    def advance_to(self, target_s: float) -> None:
        if not math.isfinite(target_s):
            raise ValueError("Transport time must be finite")
        if self.now_s is None:
            self.now_s = target_s
            return
        if target_s < self.now_s - _EPS:
            raise ValueError("Transport clock cannot move backward")
        while self.now_s < target_s - _EPS:
            while self._process_now():
                pass
            next_s = target_s
            completion_due = False
            if self.arrivals:
                next_s = min(next_s, self.arrivals[0][0])
            if self.active is not None:
                next_s = min(next_s, self.active.t_exp)
                if self.rate_bit_s > 0:
                    finish_s = self.now_s + max(0.0, self.active.total_bits - self.active.sent_bits) / self.rate_bit_s
                    if finish_s < self.active.t_exp and finish_s <= next_s:
                        next_s = finish_s
                        completion_due = True
            if self.waiting:
                next_s = min(next_s, *(message.t_exp for message in self.waiting))
            interval = next_s - self.now_s
            if interval < -_EPS:
                raise AssertionError("Transport scheduler generated a past event")
            if interval <= _EPS:
                self.now_s = next_s
                if completion_due and self.active is not None:
                    self.active.sent_bits = self.active.total_bits
                if not self._process_now():
                    self.now_s = target_s
                continue
            if self.active is not None:
                self.active.sent_bits = min(self.active.total_bits,
                                            self.active.sent_bits + interval * self.rate_bit_s)
                if completion_due:
                    self.active.sent_bits = self.active.total_bits
            self.now_s = next_s
        self.now_s = target_s
        while self._process_now():
            pass

    def update_channel(self, timestamp_s: float, *, snr_db: float, distance_m: float) -> None:
        if not math.isfinite(distance_m) or distance_m <= 0:
            raise ValueError("A2G distance must be finite and positive")
        self.advance_to(timestamp_s)
        old_rate = self.rate_bit_s
        self.snr_db = float(snr_db)
        self.distance_m = float(distance_m)
        self.rate_bit_s = self.config.rate_bit_s(self.snr_db)
        self._event("channel_sample")
        if self.active is not None and old_rate > 0 and self.rate_bit_s == 0:
            self._event("service_paused", self.active)
        if self.active is not None and old_rate == 0 and self.rate_bit_s > 0:
            self._event("service_resumed", self.active)

    def enqueue(self, payload: dict[str, Any], *, enqueue_timestamp_s: float,
                payload_bytes_override: int | None = None) -> _Message:
        self.advance_to(enqueue_timestamp_s)
        if self.distance_m is None or self.snr_db is None:
            raise ValueError("Sample the current A2G channel before enqueuing")
        if payload_bytes_override is None:
            sender, encoded = self.config.serialize_sender(payload)
            payload_bytes = len(encoded)
        else:
            if payload_bytes_override <= 0:
                raise ValueError("Probe payload size must be positive")
            sender = dict(payload)
            payload_bytes = int(payload_bytes_override)
        t_obs = float(sender["source_timestamp_s"])
        if not math.isfinite(t_obs) or t_obs > enqueue_timestamp_s + _EPS:
            raise ValueError("Source observation must precede enqueue time")
        message = _Message(
            payload=sender, message_id=str(sender["message_id"]),
            t_obs=t_obs, t_enq=float(enqueue_timestamp_s),
            t_exp=t_obs + self.config.ttl_s,
            payload_bytes=payload_bytes, header_bytes=self.config.header_bytes,
            total_bits=8.0 * (payload_bytes + self.config.header_bytes),
        )
        if any(item.message_id == message.message_id for item in self.records):
            raise ValueError(f"Duplicate A2G message ID: {message.message_id}")
        self.records.append(message)
        if message.t_exp <= self.now_s:
            message.status = "expired_before_enqueue"
            self._event(message.status, message)
            return message
        self.waiting.append(message)
        self._event("enqueued", message)
        while self._process_now():
            pass
        return message

    def take_deliveries(self) -> list[dict[str, Any]]:
        delivered = self._deliveries
        self._deliveries = []
        return delivered

    def age_of_information_s(self, timestamp_s: float) -> float | None:
        if self.now_s is None or timestamp_s > self.now_s + _EPS:
            raise ValueError("AoI query cannot read future transport state")
        if self._latest_observation_s is None:
            return None
        return timestamp_s - self._latest_observation_s

    def end_episode(self, timestamp_s: float) -> None:
        self.advance_to(timestamp_s)
        for message in [*self.waiting, *([self.active] if self.active is not None else [])]:
            message.status = "episode_censored"
            self._event(message.status, message)
        self.waiting.clear()
        self.active = None
        for _, _, message in self.arrivals:
            message.status = "episode_censored"
            self._event(message.status, message)
        self.arrivals.clear()

    def summary(self) -> dict[str, Any]:
        statuses = {status: sum(item.status == status for item in self.records)
                    for status in sorted({item.status for item in self.records})}
        valid = sorted((item for item in self.records if item.status == "available_valid"),
                       key=lambda item: float(item.t_arr))
        ages = [float(item.t_arr - item.t_obs) for item in valid]
        waits = [float(item.t_start - item.t_enq) for item in valid]
        aoi_mean: float | None = None
        if valid and self.now_s is not None and self.now_s > valid[0].t_arr:
            latest_observation = valid[0].t_obs
            previous_time = float(valid[0].t_arr)
            integral = 0.0
            for item in valid[1:]:
                delta = float(item.t_arr) - previous_time
                integral += (previous_time - latest_observation) * delta + delta * delta / 2.0
                latest_observation = max(latest_observation, item.t_obs)
                previous_time = float(item.t_arr)
            delta = self.now_s - previous_time
            integral += (previous_time - latest_observation) * delta + delta * delta / 2.0
            aoi_mean = integral / (self.now_s - float(valid[0].t_arr))
        return {
            "group": self.group, "messages_enqueued_or_rejected": len(self.records),
            "status_counts": statuses,
            "transmitted_bits": sum(item.sent_bits for item in self.records),
            "delivered_valid": statuses.get("available_valid", 0),
            "expired_total": sum(count for status, count in statuses.items()
                                 if status.startswith("expired_")),
            "episode_censored": statuses.get("episode_censored", 0),
            "mean_valid_age_s": statistics.mean(ages) if ages else None,
            "median_valid_age_s": statistics.median(ages) if ages else None,
            "mean_valid_queue_wait_s": statistics.mean(waits) if waits else None,
            "mean_aoi_after_first_delivery_s": aoi_mean,
        }

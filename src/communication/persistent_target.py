"""Deterministic repeated target-state messages and freshness-aware selection."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class PersistentTargetSelector:
    delay_s: float = 0.05
    ttl_s: float = 1.0
    consensus_radius_m: float = 10.0
    maximum_position_jump_m: float = 12.0
    require_cross_device_consensus: bool = False
    latest_sequence: dict[str, int] = field(default_factory=dict)
    latest_by_source: dict[str, dict[str, Any]] = field(default_factory=dict)
    pending: list[dict[str, Any]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    last_selected_xyz: list[float] | None = None

    def send(self, source: str, frame: int, timestamp_s: float, candidate: dict[str, Any]) -> dict[str, Any]:
        sequence = self.latest_sequence.get(source, 0) + 1
        self.latest_sequence[source] = sequence
        xyz = candidate.get("world_position_xyz")
        message = {
            "message_id": f"{source}_{sequence:06d}",
            "message_type": "target_state_v1",
            "source": source,
            "destination": "coordination_module",
            "logical_target_id": "red_van_candidate_001",
            "sequence_number": sequence,
            "source_frame": int(frame),
            "source_timestamp_s": float(timestamp_s),
            "sent_timestamp_s": float(timestamp_s),
            "expires_at_s": float(timestamp_s) + self.ttl_s,
            "class_hypothesis": "vehicle",
            "color": str(candidate.get("color", "unknown")),
            "class_score": float(candidate.get("detector_score", 0.0)),
            "color_score": float(candidate.get("color_score", 0.0)),
            "depth_m": candidate.get("depth_m"),
            "depth_valid": candidate.get("depth_m") is not None and xyz is not None,
            "depth_reliability": float(candidate.get("depth_reliability", 0.0)),
            "world_position_xyz": None if xyz is None else [float(value) for value in xyz],
            "temporal_confirmed": bool(candidate.get("temporal_confirmed", False)),
            "candidate_id": str(candidate.get("candidate_id", "")),
            "score_components": {},
            "status": "sent",
        }
        message["score_components"] = {
            "task_match": 1.0 if message["class_hypothesis"] == "vehicle" and message["color"] == "red" else 0.0,
            "depth_reliability": message["depth_reliability"],
            "freshness": 1.0,
            "temporal_continuity": 1.0 if message["temporal_confirmed"] else 0.5,
        }
        message["selection_score"] = (
            0.40 * message["score_components"]["task_match"]
            + 0.30 * message["score_components"]["depth_reliability"]
            + 0.20 * message["score_components"]["freshness"]
            + 0.10 * message["score_components"]["temporal_continuity"]
        )
        message["receive_timestamp_s"] = float(timestamp_s) + self.delay_s
        self.pending.append(message)
        self.events.append({"event": "sent", **message})
        return message

    def advance(self, now_s: float, delivery_enabled: bool = True) -> list[dict[str, Any]]:
        delivered: list[dict[str, Any]] = []
        remaining: list[dict[str, Any]] = []
        for message in self.pending:
            if float(message["receive_timestamp_s"]) > float(now_s) + 1e-9:
                remaining.append(message)
                continue
            if not delivery_enabled:
                self.events.append({"event": "dropped_during_outage", **message})
                continue
            source = str(message["source"])
            sequence = int(message["sequence_number"])
            previous = int(self.latest_sequence.get(f"received:{source}", 0))
            if sequence <= previous:
                self.events.append({"event": "rejected_duplicate_or_out_of_order", **message})
                continue
            if float(message["expires_at_s"]) < float(now_s):
                self.events.append({"event": "rejected_expired", **message})
                continue
            self.latest_sequence[f"received:{source}"] = sequence
            received = {**message, "status": "received", "received_at_s": float(now_s)}
            self.latest_by_source[source] = received
            delivered.append(received)
            self.events.append({"event": "received", **received})
        self.pending = remaining
        return delivered

    def select(self, now_s: float, allowed_sources: set[str] | None = None) -> dict[str, Any] | None:
        eligible = []
        for source, message in list(self.latest_by_source.items()):
            age = float(now_s) - float(message["source_timestamp_s"])
            if age > self.ttl_s or float(message["expires_at_s"]) < float(now_s):
                self.events.append({"event": "expired", "source": source, "message_id": message["message_id"], "age_s": age})
                del self.latest_by_source[source]
                continue
            if allowed_sources is not None and source not in allowed_sources:
                continue
            if not message["depth_valid"] or message["color"] != "red":
                continue
            xyz = message.get("world_position_xyz")
            if xyz is None:
                continue
            if self.require_cross_device_consensus:
                corroborated = any(
                    other_source != source
                    and other.get("world_position_xyz") is not None
                    and ((float(xyz[0]) - float(other["world_position_xyz"][0])) ** 2
                         + (float(xyz[1]) - float(other["world_position_xyz"][1])) ** 2) ** 0.5
                    <= self.consensus_radius_m
                    for other_source, other in self.latest_by_source.items()
                )
                if not corroborated:
                    continue
            continuity = 1.0
            if self.last_selected_xyz is not None:
                jump = ((float(xyz[0]) - self.last_selected_xyz[0]) ** 2
                        + (float(xyz[1]) - self.last_selected_xyz[1]) ** 2) ** 0.5
                if jump > self.maximum_position_jump_m:
                    self.events.append({
                        "event": "rejected_discontinuous_target_update",
                        "source": source,
                        "message_id": message["message_id"],
                        "position_jump_m": jump,
                    })
                    continue
                continuity = max(0.0, 1.0 - jump / max(self.maximum_position_jump_m, 1e-9))
            candidate = dict(message)
            candidate["age_s"] = age
            candidate["score_components"] = dict(message["score_components"])
            candidate["score_components"]["freshness"] = max(0.0, 1.0 - age / max(self.ttl_s, 1e-9))
            candidate["score_components"]["temporal_continuity"] = continuity
            candidate["selection_score"] = (
                0.40 * candidate["score_components"]["task_match"]
                + 0.30 * candidate["score_components"]["depth_reliability"]
                + 0.20 * candidate["score_components"]["freshness"]
                + 0.10 * candidate["score_components"]["temporal_continuity"]
            )
            eligible.append(candidate)
        if not eligible:
            return None
        return max(eligible, key=lambda item: (item["selection_score"], -item["age_s"], item["source"]))

    def select_and_log(self, now_s: float, allowed_sources: set[str] | None = None) -> dict[str, Any] | None:
        selected = self.select(now_s, allowed_sources=allowed_sources)
        if selected is None:
            self.events.append({"event": "no_fresh_target_state", "timestamp_s": float(now_s)})
        else:
            self.last_selected_xyz = [float(value) for value in selected["world_position_xyz"]]
            self.events.append({"event": "selected", "timestamp_s": float(now_s), **selected})
        return selected

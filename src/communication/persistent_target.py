"""Deterministic repeated target-state messages and freshness-aware selection."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

try:
    from perception.task_spec import TaskSpec, canonical_category
except ModuleNotFoundError:
    from ..perception.task_spec import TaskSpec, canonical_category


def stale_route_recovery_action(
    expiry_policy: str,
    stale_duration_s: float,
    stale_distance_m: float,
    grace_s: float,
    maximum_distance_m: float,
) -> str:
    """Choose bounded stale-route behavior without treating TTL as task cancellation."""
    if expiry_policy == "coast_then_patrol":
        if stale_duration_s >= grace_s or stale_distance_m >= maximum_distance_m:
            return "resume_patrol"
        return "coast"
    if expiry_policy == "hold_after_grace":
        return "safe_hold" if stale_duration_s >= grace_s else "coast"
    return "return_to_patrol"


@dataclass
class PersistentTargetSelector:
    delay_s: float = 0.05
    ttl_s: float = 1.0
    consensus_radius_m: float = 10.0
    maximum_position_jump_m: float = 12.0
    require_cross_device_consensus: bool = False
    task_spec: TaskSpec | None = None
    latest_sequence: dict[str, int] = field(default_factory=dict)
    latest_by_source: dict[str, dict[str, Any]] = field(default_factory=dict)
    pending: list[dict[str, Any]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)
    last_selected_xyz: list[float] | None = None

    def matches_task(self, message: dict[str, Any]) -> bool:
        if self.task_spec is None:
            return message.get("class_hypothesis") == "vehicle" and message.get("color") == "red"
        category = canonical_category(message.get("category"))
        expected = self.task_spec.target.category
        category_match = category == expected or (
            expected == "vehicle" and category in {"van", "car", "truck", "bus", "motorcycle", "bicycle"}
        )
        return category_match and bool(message.get("target_color_match", False))

    def send(self, source: str, frame: int, timestamp_s: float, candidate: dict[str, Any],
             *, external_delivery: bool = False) -> dict[str, Any]:
        sequence = self.latest_sequence.get(source, 0) + 1
        self.latest_sequence[source] = sequence
        xyz = candidate.get("world_position_xyz")
        message = {
            "message_id": f"{source}_{sequence:06d}",
            "message_type": "target_state_v1",
            "source": source,
            "destination": "coordination_module",
            "logical_target_id": "red_van_candidate_001" if self.task_spec is None else self.task_spec.task_id,
            "sequence_number": sequence,
            "source_frame": int(frame),
            "source_timestamp_s": float(timestamp_s),
            "sent_timestamp_s": float(timestamp_s),
            "expires_at_s": float(timestamp_s) + self.ttl_s,
            "class_hypothesis": "vehicle",
            "category": str(candidate.get("category", "vehicle")),
            "target_color": candidate.get("target_color"),
            "target_color_match": bool(candidate.get("target_color_match", False)),
            "color": str(candidate.get("color", "unknown")),
            "class_score": float(candidate.get("type_probability", candidate.get("detector_score", 0.0))),
            "color_score": float(candidate.get("color_score", 0.0)),
            "depth_m": candidate.get("depth_m"),
            "depth_valid": candidate.get("depth_m") is not None and xyz is not None,
            "depth_reliability": float(candidate.get("depth_reliability", 0.0)),
            "world_position_xyz": None if xyz is None else [float(value) for value in xyz],
            "temporal_confirmed": bool(candidate.get("temporal_confirmed", False)),
            "candidate_id": str(candidate.get("candidate_id", "")),
            "evidence_stage": str(candidate.get('evidence_stage','tentative_candidate')),
            "score_components": {},
            "status": "sent",
        }
        message["score_components"] = {
            "task_match": 1.0 if self.matches_task(message) else 0.0,
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
        message["receive_timestamp_s"] = None if external_delivery else float(timestamp_s) + self.delay_s
        if not external_delivery:
            self.pending.append(message)
        self.events.append({"event": "sent", **message})
        return message

    def receive_external(self, message: dict[str, Any]) -> dict[str, Any] | None:
        """Admit an externally transported packet using its actual arrival time."""
        arrived = float(message["received_at_s"])
        source = str(message["source"])
        sequence = int(message["sequence_number"])
        previous = int(self.latest_sequence.get(f"received:{source}", 0))
        if sequence <= previous:
            self.events.append({"event": "rejected_duplicate_or_out_of_order", **message})
            return None
        if float(message["expires_at_s"]) <= arrived:
            self.events.append({"event": "rejected_expired", **message})
            return None
        self.latest_sequence[f"received:{source}"] = sequence
        received = {**message, "status": "received", "receive_timestamp_s": arrived}
        self.latest_by_source[source] = received
        self.events.append({"event": "received", **received})
        return received

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
            if float(message["expires_at_s"]) <= float(now_s):
                self.events.append({"event": "rejected_expired", **message})
                continue
            self.latest_sequence[f"received:{source}"] = sequence
            received = {**message, "status": "received", "received_at_s": float(now_s)}
            self.latest_by_source[source] = received
            delivered.append(received)
            self.events.append({"event": "received", **received})
        self.pending = remaining
        return delivered

    def corroborated_by_other_source(
        self, message: dict[str, Any], now_s: float, allowed_sources: set[str]
    ) -> bool:
        """Require a fresh, eligible position from another device for memory admission."""
        xyz = message.get("world_position_xyz")
        if xyz is None:
            return False
        for source, other in self.latest_by_source.items():
            if source == message.get("source") or source not in allowed_sources:
                continue
            other_xyz = other.get("world_position_xyz")
            if (
                other_xyz is None
                or not other.get("depth_valid")
                or not self.matches_task(other)
                or float(other.get("expires_at_s", -math.inf)) <= now_s
                or now_s - float(other.get("source_timestamp_s", -math.inf)) >= self.ttl_s
            ):
                continue
            if math.hypot(float(xyz[0]) - float(other_xyz[0]),
                          float(xyz[1]) - float(other_xyz[1])) <= self.consensus_radius_m:
                return True
        return False

    def select(self, now_s: float, allowed_sources: set[str] | None = None) -> dict[str, Any] | None:
        # Prune every source before consensus; a fresh entry must never be
        # corroborated by an expired entry later in insertion order.
        for source, message in list(self.latest_by_source.items()):
            age = float(now_s) - float(message["source_timestamp_s"])
            if age >= self.ttl_s or float(message["expires_at_s"]) <= float(now_s):
                self.events.append({"event": "expired", "source": source, "message_id": message["message_id"], "age_s": age})
                del self.latest_by_source[source]
        eligible = []
        for source, message in list(self.latest_by_source.items()):
            age = float(now_s) - float(message["source_timestamp_s"])
            if allowed_sources is not None and source not in allowed_sources:
                continue
            if not message["depth_valid"] or not self.matches_task(message):
                continue
            xyz = message.get("world_position_xyz")
            if xyz is None:
                continue
            if self.require_cross_device_consensus:
                if not self.corroborated_by_other_source(
                    message, now_s, allowed_sources if allowed_sources is not None else set(self.latest_by_source)
                ):
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


@dataclass
class PersistentTargetMemory:
    """Keep a committed task target independently of individual packet TTLs.

    A message can refresh or challenge the memory only while it is fresh.  The
    simulator's target actor/ground truth is deliberately not consulted here.
    Candidate IDs are frame-local and the wire logical_target_id is shared by
    all detections, so association uses perceived position and attributes.
    """

    association_radius_m: float = 10.0
    challenger_window_s: float = 4.0
    challenger_min_updates: int = 3
    challenger_min_score_margin: float = 0.08
    challenger_old_unseen_s: float = 2.0
    challenger_min_sources: int = 2
    task_spec: TaskSpec | None = None
    current: dict[str, Any] | None = None
    current_score: float = 0.0
    last_confirmed_at_s: float | None = None
    challenger: list[dict[str, Any]] = field(default_factory=list)
    events: list[dict[str, Any]] = field(default_factory=list)

    @staticmethod
    def _distance(a: dict[str, Any], b: dict[str, Any]) -> float:
        aa, bb = a["world_position_xyz"], b["world_position_xyz"]
        return math.hypot(float(aa[0]) - float(bb[0]), float(aa[1]) - float(bb[1]))

    def consider(self, message: dict[str, Any], now_s: float) -> str:
        """Return initial, refresh, switched, challenger, or rejected."""
        now_s = float(now_s)
        xyz = message.get("world_position_xyz")
        if (
            float(message.get("expires_at_s", -math.inf)) <= now_s
            or not PersistentTargetSelector(task_spec=self.task_spec).matches_task(message)
            or not message.get("depth_valid")
            or not isinstance(xyz, (list, tuple))
            or len(xyz) < 2
            or not all(math.isfinite(float(v)) for v in xyz[:2])
        ):
            self.events.append({"event": "memory_rejected_invalid_or_expired", "timestamp_s": now_s,
                                "message_id": message.get("message_id")})
            return "rejected"
        score = float(message.get("selection_score", 0.0))
        if self.current is None:
            self.current = dict(message)
            self.current_score = score
            self.last_confirmed_at_s = now_s
            self.events.append({"event": "memory_initial_target", "timestamp_s": now_s,
                                "message_id": message.get("message_id"), "target_xyz": list(xyz)})
            return "initial"
        if self._distance(message, self.current) <= self.association_radius_m:
            self.last_confirmed_at_s = now_s
            self.current_score = max(self.current_score, score)
            self.challenger.clear()
            self.events.append({"event": "memory_same_target_refresh", "timestamp_s": now_s,
                                "message_id": message.get("message_id")})
            return "refresh"
        # A different observation never revokes the route by itself.  Require
        # repeated, spatially consistent evidence from the configured number
        # of sources and a margin over the committed target's original score.
        self.challenger = [
            item for item in self.challenger
            if now_s - float(item["received_at_s"]) <= self.challenger_window_s
            and self._distance(item, message) <= self.association_radius_m
        ]
        if not any(item.get("message_id") == message.get("message_id") for item in self.challenger):
            self.challenger.append({**message, "received_at_s": now_s})
        sources = {str(item.get("source")) for item in self.challenger}
        best_score = max(float(item.get("selection_score", 0.0)) for item in self.challenger)
        old_unseen_s = now_s - float(
            self.last_confirmed_at_s if self.last_confirmed_at_s is not None else now_s
        )
        confirmed = (
            len(self.challenger) >= self.challenger_min_updates
            and len(sources) >= self.challenger_min_sources
            and best_score >= self.current_score + self.challenger_min_score_margin
            and old_unseen_s >= self.challenger_old_unseen_s
        )
        if confirmed:
            best = max(self.challenger, key=lambda item: float(item.get("selection_score", 0.0)))
            previous = self.current
            self.current = dict(best)
            self.current_score = best_score
            self.last_confirmed_at_s = now_s
            self.challenger.clear()
            self.events.append({"event": "memory_confirmed_target_switch", "timestamp_s": now_s,
                                "old_message_id": previous.get("message_id"),
                                "message_id": best.get("message_id"),
                                "old_target_xyz": previous.get("world_position_xyz"),
                                "target_xyz": best.get("world_position_xyz"),
                                "sources": sorted(sources)})
            return "switched"
        self.events.append({"event": "memory_challenger_unconfirmed", "timestamp_s": now_s,
                            "message_id": message.get("message_id"),
                            "candidate_xyz": list(xyz), "updates": len(self.challenger),
                            "sources": sorted(sources), "score": best_score,
                            "required_score": self.current_score + self.challenger_min_score_margin,
                            "old_unseen_s": old_unseen_s})
        return "challenger"

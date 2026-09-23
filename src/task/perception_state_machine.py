"""Auditable S1 perception-driven task state machine."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


ALLOWED_TRANSITIONS = {
    "INITIALIZING": {"SEARCHING", "FAILED"},
    "SEARCHING": {"UAV_CANDIDATE_CONFIRMED", "UAV_ROUTE_COMPLETE", "FAILED"},
    "UAV_CANDIDATE_CONFIRMED": {"MESSAGE_SENT", "FAILED"},
    "MESSAGE_SENT": {"TARGET_RECEIVED", "FAILED"},
    "TARGET_RECEIVED": {"PLANNING_TO_OBSERVATION_POINT", "FAILED"},
    "PLANNING_TO_OBSERVATION_POINT": {"NAVIGATING", "FAILED"},
    "NAVIGATING": {"LOCAL_CONFIRMING", "FAILED"},
    "LOCAL_CONFIRMING": {"TARGET_VERIFIED", "CANDIDATE_REJECTED", "FAILED"},
    "CANDIDATE_REJECTED": {"SEARCHING", "FAILED"},
    "TARGET_VERIFIED": {"ARRIVED_SAFE", "FAILED"},
    "ARRIVED_SAFE": {"UAV_ROUTE_COMPLETE", "COMPLETED", "FAILED"},
    "UAV_ROUTE_COMPLETE": {"COMPLETED", "UAV_CANDIDATE_CONFIRMED", "FAILED"},
    "COMPLETED": set(),
    "FAILED": set(),
}


@dataclass
class PerceptionTaskStateMachine:
    task_id: str
    instruction: str
    state: str = "INITIALIZING"
    events: list[dict[str, Any]] = field(default_factory=list)
    transitions_valid: bool = True
    ugv_arrived: bool = False
    uav_route_complete: bool = False
    local_confirmation_started: bool = False
    target_verified: bool = False

    def transition(self, new_state: str, frame: int, timestamp: float, trigger: str, **details: Any) -> None:
        allowed = new_state in ALLOWED_TRANSITIONS.get(self.state, set())
        self.transitions_valid = self.transitions_valid and allowed
        previous = self.state
        self.state = new_state
        self.events.append(
            {
                "frame": int(frame),
                "timestamp": float(timestamp),
                "previous_state": previous,
                "new_state": new_state,
                "trigger": trigger,
                "transition_valid": bool(allowed),
                **details,
            }
        )

    def start_local_confirmation(self, frame: int, timestamp: float, **details: Any) -> None:
        if self.local_confirmation_started:
            return
        self.local_confirmation_started = True
        self.transition("LOCAL_CONFIRMING", frame, timestamp, "entered_local_confirmation_range", **details)

    def mark_target_verified(self, frame: int, timestamp: float, **details: Any) -> None:
        if self.target_verified:
            return
        self.target_verified = True
        self.transition("TARGET_VERIFIED", frame, timestamp, "ugv_rgb_depth_confirmation", **details)

    def mark_ugv_arrived(self, frame: int, timestamp: float, **details: Any) -> None:
        if self.ugv_arrived:
            return
        self.ugv_arrived = True
        self.transition("ARRIVED_SAFE", frame, timestamp, "safe_standoff_held_after_local_verification", **details)
        self._complete_if_ready(frame, timestamp)

    def mark_uav_route_complete(self, frame: int, timestamp: float, **details: Any) -> None:
        if self.uav_route_complete:
            return
        self.uav_route_complete = True
        if self.ugv_arrived:
            self.transition("UAV_ROUTE_COMPLETE", frame, timestamp, "all_uav_waypoints_reached", **details)
        elif self.state == "SEARCHING":
            self.transition("UAV_ROUTE_COMPLETE", frame, timestamp, "all_uav_waypoints_reached", **details)
        self._complete_if_ready(frame, timestamp)

    def _complete_if_ready(self, frame: int, timestamp: float) -> None:
        if self.ugv_arrived and self.uav_route_complete and self.state != "COMPLETED":
            self.transition("COMPLETED", frame, timestamp, "uav_route_and_verified_ugv_arrival_complete")

"""Auditable S0 Oracle task state machine."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


ALLOWED_TRANSITIONS = {
    "INITIALIZING": {"SEARCHING", "FAILED"},
    "SEARCHING": {"ORACLE_TARGET_VISIBLE", "UAV_ROUTE_COMPLETE", "FAILED"},
    "ORACLE_TARGET_VISIBLE": {"MESSAGE_SENT", "FAILED"},
    "MESSAGE_SENT": {"TARGET_RECEIVED", "FAILED"},
    "TARGET_RECEIVED": {"PLANNING", "FAILED"},
    "PLANNING": {"NAVIGATING", "FAILED"},
    "NAVIGATING": {"ARRIVED_SAFE", "FAILED"},
    "ARRIVED_SAFE": {"UAV_ROUTE_COMPLETE", "COMPLETED", "FAILED"},
    "UAV_ROUTE_COMPLETE": {"COMPLETED", "ORACLE_TARGET_VISIBLE", "FAILED"},
    "COMPLETED": set(),
    "FAILED": set(),
}


@dataclass
class OracleTaskStateMachine:
    task_id: str
    instruction: str
    state: str = "INITIALIZING"
    events: list[dict[str, Any]] = field(default_factory=list)
    transitions_valid: bool = True
    ugv_arrived: bool = False
    uav_route_complete: bool = False

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

    def mark_ugv_arrived(self, frame: int, timestamp: float, **details: Any) -> None:
        if self.ugv_arrived:
            return
        self.ugv_arrived = True
        self.transition("ARRIVED_SAFE", frame, timestamp, "safe_standoff_held", **details)
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
            self.transition("COMPLETED", frame, timestamp, "uav_route_and_ugv_arrival_complete")

"""Deterministic single-message channel for S0 Oracle acceptance."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any


@dataclass
class OracleChannel:
    enabled: bool = True
    delay_ticks: int = 1
    messages: list[dict[str, Any]] = field(default_factory=list)
    _pending: dict[str, Any] | None = None
    _deliver_tick: int | None = None

    def send(self, payload: dict[str, Any], tick_index: int, frame: int, timestamp: float) -> dict[str, Any] | None:
        if not self.enabled or self._pending is not None or self.messages:
            return None
        body = dict(payload)
        body.update(
            {
                "message_id": "oracle_target_0001",
                "source": "uav",
                "destination": "ugv",
                "message_type": "oracle_target_state",
                "oracle": True,
                "sent_frame": int(frame),
                "sent_timestamp": float(timestamp),
                "received_frame": None,
                "received_timestamp": None,
                "latency_ms": None,
                "status": "sent",
            }
        )
        body["payload_bytes"] = len(json.dumps(body, ensure_ascii=False).encode("utf-8"))
        self._pending = body
        self._deliver_tick = int(tick_index + max(1, self.delay_ticks))
        return dict(body)

    def receive(self, tick_index: int, frame: int, timestamp: float) -> dict[str, Any] | None:
        if self._pending is None or self._deliver_tick is None or tick_index < self._deliver_tick:
            return None
        message = self._pending
        message["received_frame"] = int(frame)
        message["received_timestamp"] = float(timestamp)
        message["latency_ms"] = float((timestamp - message["sent_timestamp"]) * 1000.0)
        message["status"] = "delivered"
        self.messages.append(dict(message))
        self._pending = None
        self._deliver_tick = None
        return dict(message)

from __future__ import annotations

from src.communication.oracle_channel import OracleChannel
from src.task.oracle_state_machine import OracleTaskStateMachine


def test_oracle_channel_delivers_once_after_configured_tick() -> None:
    channel = OracleChannel(enabled=True, delay_ticks=1)
    sent = channel.send({"target_world_position_xyz": [1.0, 2.0, 0.0]}, 4, 100, 5.0)
    assert sent is not None and sent["oracle"] is True
    assert channel.receive(4, 100, 5.0) is None
    received = channel.receive(5, 101, 5.05)
    assert received is not None
    assert received["status"] == "delivered"
    assert received["received_frame"] == 101
    assert len(channel.messages) == 1


def test_oracle_state_machine_reaches_completed_in_order() -> None:
    machine = OracleTaskStateMachine("task", "instruction")
    transitions = [
        "SEARCHING",
        "ORACLE_TARGET_VISIBLE",
        "MESSAGE_SENT",
        "TARGET_RECEIVED",
        "PLANNING",
        "NAVIGATING",
    ]
    for frame, state in enumerate(transitions, start=1):
        machine.transition(state, frame, frame * 0.05, "test")
    machine.mark_ugv_arrived(10, 0.5)
    machine.mark_uav_route_complete(20, 1.0)
    assert machine.transitions_valid
    assert machine.state == "COMPLETED"

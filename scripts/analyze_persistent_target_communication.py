#!/usr/bin/env python3
"""Summarize and audit an E7 persistent UAV/UGV target-message run."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT / "src"))

from communication.persistent_target import PersistentTargetSelector


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    communication_dir = run_dir / "communication"
    preview_dir = run_dir / "preview"
    preview_dir.mkdir(parents=True, exist_ok=True)
    messages = read_jsonl(communication_dir / "messages.jsonl")
    events = read_jsonl(communication_dir / "communication_events.jsonl")
    route_events = read_jsonl(run_dir / "planning" / "target_route_events.jsonl")
    source_counts = Counter(str(row.get("source")) for row in messages)
    sequence_checks: dict[str, bool] = {}
    for source in ("uav", "ugv"):
        seq = [int(row["sequence_number"]) for row in messages if row.get("source") == source]
        sequence_checks[source] = seq == sorted(set(seq))
    event_counts = Counter(str(row.get("event")) for row in events)

    expiry_probe: dict[str, Any] = {}
    if messages:
        probe = PersistentTargetSelector(delay_s=0.05, ttl_s=1.0)
        for source in ("uav", "ugv"):
            sent = [row for row in messages if row.get("source") == source]
            if not sent:
                continue
            last = sent[-1]
            candidate = {
                "candidate_id": last.get("candidate_id", "offline-replay"),
                "color": last.get("color"),
                "color_score": last.get("color_score", 0.0),
                "detector_score": last.get("class_score", 0.0),
                "depth_m": last.get("depth_m"),
                "depth_reliability": last.get("depth_reliability", 0.0),
                "world_position_xyz": last.get("world_position_xyz"),
                "temporal_confirmed": last.get("temporal_confirmed", False),
            }
            probe.send(source, int(last.get("source_frame", -1)), float(last["source_timestamp_s"]), candidate)
        last_source_time = max(float(row["source_timestamp_s"]) for row in messages)
        probe.advance(last_source_time + 0.05)
        fresh_before_expiry = probe.select(last_source_time + 0.5) is not None
        stale_after_expiry = probe.select(last_source_time + 1.1) is None
        expiry_probe = {
            "scope": "offline protocol replay only; no new simulator data collected",
            "fresh_message_selected_before_ttl": fresh_before_expiry,
            "expired_message_rejected_after_ttl": stale_after_expiry,
            "ttl_s": 1.0,
            "passed": fresh_before_expiry and stale_after_expiry,
        }

    safety_rows: list[dict[str, str]] = []
    safety_path = run_dir / "safety" / "safety_state.csv"
    if safety_path.exists():
        with safety_path.open("r", encoding="utf-8-sig", newline="") as handle:
            safety_rows = list(csv.DictReader(handle))
    stale_follow_commands = sum(
        str(row.get("stale_target_follow_command", "false")).lower() == "true"
        for row in safety_rows
    )
    route_return_events = [
        row for row in route_events if row.get("event") == "target_route_expired_return_to_patrol"
    ]

    audit = {
        "run_dir": str(run_dir),
        "simulator_run_status": json.loads((run_dir / "validation_report.json").read_text(encoding="utf-8")).get("status"),
        "sent_message_count_by_source": {source: int(source_counts.get(source, 0)) for source in ("uav", "ugv")},
        "received_message_events": int(event_counts.get("received", 0)),
        "selected_state_events": int(event_counts.get("selected", 0)),
        "sequence_numbers_strictly_increasing": sequence_checks,
        "stale_target_follow_commands": stale_follow_commands,
        "live_expiry_transition": {
            "target_route_returned_to_patrol_after_ttl": bool(route_return_events),
            "route_return_event_count": len(route_return_events),
            "last_transition": route_return_events[-1] if route_return_events else None,
            "stale_target_follow_commands": stale_follow_commands,
            "passed": bool(route_return_events) and stale_follow_commands == 0,
        },
        "offline_expiry_probe": expiry_probe,
        "interpretation": (
            "Simulator logs establish repeated communication, selected-message route following, and a planned end-of-run message cutoff "
            "that exercised TTL expiry and return to patrol. No random packet loss was tested. Duplicate/out-of-order rejection is covered by unit tests."
        ),
    }
    (communication_dir / "communication_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    figure, (ax_count, ax_source) = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    colors = {"uav": "#2878B5", "ugv": "#E07A1F"}
    for source in ("uav", "ugv"):
        rows = [row for row in messages if row.get("source") == source]
        times = [float(row["source_timestamp_s"]) for row in rows]
        if times:
            ax_count.hist(times, bins=30, alpha=0.55, color=colors[source], label=f"{source.upper()} sent")
    ax_count.set(title="Continuous target-state messages", xlabel="Simulation time (s)", ylabel="Messages / time bin")
    ax_count.legend(frameon=False)
    selected = [row for row in events if row.get("event") == "selected"]
    selected_by_source = {source: [row for row in selected if row.get("source") == source] for source in ("uav", "ugv")}
    for index, source in enumerate(("uav", "ugv"), start=1):
        times = [float(row.get("timestamp_s", row.get("received_at_s", 0.0))) for row in selected_by_source[source]]
        scores = [float(row.get("selection_score", 0.0)) for row in selected_by_source[source]]
        if times:
            ax_source.scatter(times, [index] * len(times), s=20, c=scores, vmin=0.0, vmax=1.0, cmap="viridis", edgecolors="none")
    ax_source.set_yticks([1, 2], ["UAV selected", "UGV selected"])
    ax_source.set(title="Selected information source and selection score", xlabel="Simulation time (s)", ylim=(0.5, 2.5))
    figure.colorbar(plt.cm.ScalarMappable(cmap="viridis", norm=plt.Normalize(0.0, 1.0)), ax=ax_source, label="Selection score")
    figure.savefig(preview_dir / "persistent_communication_timeline.png", dpi=180)
    plt.close(figure)
    print(json.dumps({"audit": str(communication_dir / "communication_audit.json"), "figure": str(preview_dir / "persistent_communication_timeline.png"), "summary": audit}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

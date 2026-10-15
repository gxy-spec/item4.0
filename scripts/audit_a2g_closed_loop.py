"""Audit one online A2G closed-loop acceptance run from saved artifacts."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run = args.run_dir.resolve()

    validation = json.loads((run / "validation_report.json").read_text(encoding="utf-8"))
    transport = json.loads((run / "communication/a2g_transport_summary.json").read_text(encoding="utf-8"))
    safety_audit = json.loads((run / "safety/obstacle_safety_audit.json").read_text(encoding="utf-8"))
    radio = read_jsonl(run / "communication/a2g_transport_events_geo.jsonl")
    communication = read_jsonl(run / "communication/communication_events.jsonl")
    routes = read_jsonl(run / "planning/target_route_events.jsonl")
    safety = list(csv.DictReader((run / "safety/safety_state.csv").open(encoding="utf-8-sig", newline="")))
    trace = list(csv.DictReader((run / "communication/spatial_link_trace.csv").open(encoding="utf-8-sig", newline="")))
    t0 = float(safety[0]["timestamp"])
    fade = transport["diagnostic_attenuation_intervals_s"]
    if len(fade) != 1:
        raise ValueError("This acceptance audit expects one declared diagnostic attenuation interval")
    start_s, end_s = float(fade[0]["start_s"]), float(fade[0]["end_s"])

    def relative(timestamp_s: float) -> float:
        return float(timestamp_s) - t0

    available = {event["message_id"]: event for event in radio if event["event"] == "available_valid"}
    expired = {event["message_id"]: event for event in radio if event["event"].startswith("expired_")}
    received = {event["message_id"]: event for event in communication if event["event"] == "received"}
    selected = {event["message_id"]: event for event in communication if event["event"] == "selected"}
    route_plans = [event for event in routes if event["event"] == "target_route_planned_from_selected_message"]
    during_fade = [row for row in safety if start_s <= relative(row["timestamp"]) < end_s]
    stale = [row for row in safety if row["ugv_obstacle_sensor_state"] == "SENSOR_STALE"]
    stale_late = [row for row in stale if relative(row["timestamp"]) >= 7.2]
    recovered_control = [row for row in safety if relative(row["timestamp"]) > 8.5]
    attenuated = [row for row in trace if start_s <= relative(row["timestamp_s"]) < end_s]
    after_fade = [row for row in trace if relative(row["timestamp_s"]) >= end_s]
    checks: dict[str, bool] = {
        "runner_acceptance": validation["status"] == "PASS",
        "valid_packets_match_receiver": set(available) == set(received),
        "radio_arrival_times_preserved": all(
            abs(float(received[mid]["received_at_s"]) - float(event["arrival_timestamp_s"])) < 1e-8
            for mid, event in available.items()
        ),
        "expired_packets_never_received_or_selected": bool(expired) and not (set(expired) & (set(received) | set(selected))),
        "selected_packets_are_valid": bool(selected) and set(selected) <= set(received),
        "valid_packet_age_under_ttl": all(
            float(event["arrival_timestamp_s"]) < float(event["expires_at_s"])
            for event in available.values()
        ),
        "fade_stops_radio_service": bool(attenuated) and all(
            float(row["geo_effective_snr_db"]) < 5.0 for row in attenuated
        ) and any(event["event"] == "service_paused" for event in radio),
        "service_recovers_after_fade": bool(after_fade) and any(
            float(row["geo_effective_snr_db"]) >= 5.0 for row in after_fade
        ) and any(relative(event["timestamp_s"]) > end_s for event in available.values()),
        "valid_uav_message_creates_route": bool(route_plans) and all(
            event["source"] == "uav" and event["message_id"] in available
            and float(event["timestamp_s"]) >= float(available[event["message_id"]]["arrival_timestamp_s"])
            for event in route_plans
        ),
        "committed_target_survives_fade": bool(during_fade) and all(
            row["target_tracking_mode"] == "memory_target"
            and row["target_route_message_id"] == route_plans[0]["message_id"]
            for row in during_fade
        ) if route_plans else False,
        "sensor_stale_has_priority": bool(stale_late) and all(
            row["ugv_safety_mode"] == "sensor_stale_stop"
            and float(row["ugv_speed_mps"]) < 0.1 for row in stale_late
        ),
        "rgbd_guard_recovers": bool(recovered_control) and any(
            row["ugv_obstacle_sensor_state"] == "CLEAR" and float(row["ugv_speed_mps"]) > 0.5
            for row in recovered_control
        ),
        "no_collision_or_actor_truth_control": (
            safety_audit["online_obstacle_actor_reads"] == 0
            and validation["summary"]["ugv_collision_count"] == 0
            and validation["summary"]["uav_collision_count"] == 0
        ),
    }
    results = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "run_dir": str(run),
        "checks": checks,
        "counts": {
            "sent": sum(event["event"] == "sent" for event in communication),
            "valid_arrivals": len(available),
            "expired": len(expired),
            "selected": len(selected),
            "target_routes_planned": len(route_plans),
            "memory_target_control_ticks_during_fade": len(during_fade),
            "sensor_stale_ticks": len(stale),
        },
        "fade_interval_s": fade[0],
        "first_valid_arrival_s": min(relative(event["timestamp_s"]) for event in available.values()),
        "first_valid_arrival_after_fade_s": min(
            relative(event["timestamp_s"]) for event in available.values()
            if relative(event["timestamp_s"]) > end_s
        ),
    }
    output = run / "closed_loop_audit.json"
    output.write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": results["status"], "passed": sum(checks.values()),
                      "total": len(checks), "counts": results["counts"], "output": str(output)},
                     ensure_ascii=False, indent=2))
    return 0 if results["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

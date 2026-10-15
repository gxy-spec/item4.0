"""Audit the repaired full E1-C run used to freeze A2G channel v1."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import yaml


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def read_jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--baseline-dir", required=True, type=Path)
    parser.add_argument("--sensitivity-dir", required=True, type=Path)
    args = parser.parse_args()

    run = args.run_dir.resolve()
    baseline = args.baseline_dir.resolve()
    sensitivity = args.sensitivity_dir.resolve()
    validation = read_json(run / "validation_report.json")
    baseline_validation = read_json(baseline / "validation_report.json")
    transport = read_json(run / "communication/a2g_transport_summary.json")
    sensitivity_summary = read_json(sensitivity / "sensitivity_summary.json")
    safety_audit = read_json(run / "safety/obstacle_safety_audit.json")
    resolved = yaml.safe_load((run / "resolved_config.yaml").read_text(encoding="utf-8"))
    radio = read_jsonl(run / "communication/a2g_transport_events_geo.jsonl")
    communication = read_jsonl(run / "communication/communication_events.jsonl")
    routes = read_jsonl(run / "planning/target_route_events.jsonl")
    safety = list(csv.DictReader((run / "safety/safety_state.csv").open(encoding="utf-8-sig", newline="")))

    available = {e["message_id"]: e for e in radio if e["event"] == "available_valid"}
    expired = {e["message_id"]: e for e in radio if e["event"].startswith("expired_")}
    received_uav = {
        e["message_id"]: e for e in communication
        if e["event"] == "received" and e["source"] == "uav"
    }
    selected_uav = {
        e["message_id"]: e for e in communication
        if e["event"] == "selected" and e["source"] == "uav"
    }
    route_plans = [e for e in routes if e["event"] == "target_route_planned_from_selected_message"]
    current = validation["summary"]
    old = baseline_validation["summary"]
    memory = current["persistent_target_communication"]

    checks: dict[str, bool] = {
        "full_runner_acceptance": validation["status"] == "PASS",
        "full_duration_190_s": abs(float(current["duration_seconds"]) - 190.0) < 1e-9,
        "repaired_persistent_memory_policy": memory["expiry_policy"] == "persistent_target_memory",
        "sensitivity_suite_passed": sensitivity_summary["status"] == "PASS",
        "sensitivity_design_complete": (
            sensitivity_summary["case_count"] == 21
            and sensitivity_summary["result_rows"] == 42
            and all(sensitivity_summary["checks"].values())
        ),
        "valid_radio_packets_match_uav_receiver": set(available) == set(received_uav),
        "no_expired_or_censored_reference_packets": (
            not expired
            and transport["groups"]["geo"]["expired_total"] == 0
            and transport["groups"]["geo"]["episode_censored"] == 0
        ),
        "radio_arrival_times_preserved": all(
            abs(float(received_uav[mid]["received_at_s"]) - float(event["arrival_timestamp_s"])) < 1e-8
            for mid, event in available.items()
        ),
        "valid_packet_age_under_ttl": bool(available) and all(
            float(event["arrival_timestamp_s"]) < float(event["expires_at_s"])
            for event in available.values()
        ),
        "selected_uav_packets_are_valid": bool(selected_uav) and set(selected_uav) <= set(available),
        "uav_packet_created_the_only_target_route": (
            len(route_plans) == 1
            and route_plans[0]["source"] == "uav"
            and route_plans[0]["message_id"] in available
        ),
        "committed_route_was_never_revoked_by_ttl": memory["target_route_return_to_patrol_count"] == 0,
        "memory_route_remained_rgbd_guarded": (
            memory["target_route_follow_ticks"] > 0
            and len(safety) >= memory["target_route_follow_ticks"]
            and current["ugv_rgbd_obstacle_guard"]["sensor_stale_ticks"] == 0
        ),
        "no_collision_or_online_actor_truth_control": (
            current["ugv_collision_count"] == 0
            and current["uav_collision_count"] == 0
            and safety_audit["online_obstacle_actor_reads"] == 0
            and memory["online_target_truth_reads"] == 0
        ),
        "task_result_matches_repaired_baseline": (
            abs(float(current["ugv_final_target_distance_m"]) - float(old["ugv_final_target_distance_m"])) <= 0.25
            and abs(float(current["ugv_path_m"]) - float(old["ugv_path_m"])) <= 0.25
            and memory["target_route_plan_count"] == old["persistent_target_communication"]["target_route_plan_count"]
        ),
        "reference_rate_cap_is_10_mbps": float(resolved["transport_link"]["service"]["rate_cap_bit_s"]) == 10_000_000.0,
    }

    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "run_dir": str(run),
        "baseline_dir": str(baseline),
        "sensitivity_dir": str(sensitivity),
        "checks": checks,
        "metrics": {
            "radio_valid_messages": len(available),
            "mean_valid_age_ms": 1000.0 * float(transport["groups"]["geo"]["mean_valid_age_s"]),
            "target_route_follow_ticks": memory["target_route_follow_ticks"],
            "final_target_distance_m": current["ugv_final_target_distance_m"],
            "baseline_final_target_distance_m": old["ugv_final_target_distance_m"],
            "ugv_path_m": current["ugv_path_m"],
            "baseline_ugv_path_m": old["ugv_path_m"],
            "collisions": current["ugv_collision_count"] + current["uav_collision_count"],
        },
        "scope_note": (
            "The full E1 route is geometrically LoS. LoS/NLoS transition coverage belongs to the "
            "dedicated fixed-trajectory spatial test included by the sensitivity suite."
        ),
    }
    output = run / "freeze_candidate_audit.json"
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "passed": sum(checks.values()),
                      "total": len(checks), "metrics": result["metrics"],
                      "output": str(output)}, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

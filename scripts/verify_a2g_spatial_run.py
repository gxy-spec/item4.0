#!/usr/bin/env python3
"""Independently replay a saved A2G spatial trace against its frozen config."""

from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from communication.a2g_spatial import A2GSpatialConfig, evaluate_track  # noqa: E402


def verify(run_dir: Path) -> dict:
    resolved = yaml.safe_load((run_dir / "resolved_config.yaml").read_text(encoding="utf-8"))
    settings = A2GSpatialConfig.from_mapping(resolved["spatial_link"], run_seed=int(resolved["random_seed"]))
    with (run_dir / "communication" / "spatial_link_trace.csv").open(encoding="utf-8-sig", newline="") as handle:
        stored = list(csv.DictReader(handle))
    observations = []
    for row in stored:
        observations.append({
            "frame": int(row["frame"]), "timestamp_s": float(row["timestamp_s"]),
            "uav_antenna_xyz_m": ast.literal_eval(row["uav_antenna_xyz_m"]),
            "ugv_antenna_xyz_m": ast.literal_eval(row["ugv_antenna_xyz_m"]),
            "horizontal_m": float(row["horizontal_m"]),
            "height_m": float(row["height_m"]),
            "distance_m": float(row["distance_m"]),
            "elevation_deg": float(row["elevation_deg"]),
            "geo_los": int(row["geo_los"]),
            "first_static_blocker": row["first_static_blocker"] or None,
            "ray_hit_count": int(row["ray_hit_count"]),
        })
    replayed = evaluate_track(observations, settings)
    numeric_columns = (
        "p_los", "latent_y", "fspl_db", "noise_dbm", "geo_shadow_db", "prob_shadow_db",
        "geo_path_loss_db", "prob_path_loss_db", "geo_received_dbm", "prob_received_dbm",
        "geo_snr_db", "prob_snr_db", "geo_snr_linear", "prob_snr_linear",
    )
    differences = [
        abs(float(stored_row[key]) - float(replay_row[key]))
        for stored_row, replay_row in zip(stored, replayed)
        for key in numeric_columns
    ]
    max_difference = max(differences, default=math.inf)
    ticks = round(float(resolved["simulation"]["duration_seconds"])
                  / float(resolved["simulation"]["fixed_delta_seconds"]))
    expected = (ticks + settings.sample_every_n_ticks - 1) // settings.sample_every_n_ticks
    checks = {
        "sample_count": len(stored) == expected,
        "replayed_values": max_difference < 1e-8,
        "monotonic_time_and_frame": all(
            b["timestamp_s"] > a["timestamp_s"] and b["frame"] > a["frame"]
            for a, b in zip(observations, observations[1:])
        ),
        "geometry_blocker_consistency": all(
            (row["geo_los"] == 0) == (row["first_static_blocker"] is not None)
            for row in observations
        ),
    }
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "run_dir": str(run_dir), "checks": checks,
        "sample_count": len(stored), "expected_sample_count": expected,
        "maximum_numeric_replay_difference": max_difference,
    }
    destination = run_dir / "communication" / "spatial_link_verification.json"
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    result = verify(args.run_dir.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())

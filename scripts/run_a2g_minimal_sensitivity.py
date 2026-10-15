"""Run the frozen minimal one-factor A2G sensitivity design on one trajectory."""

from __future__ import annotations

import argparse
import ast
import csv
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from communication.a2g_spatial import A2GSpatialConfig, evaluate_track  # noqa: E402
from communication.a2g_transport import A2GTransportConfig, A2GTransportQueue  # noqa: E402


SEEDS = (1001, 2001, 3001)
CASES = (
    {"case_id": "cap_0p5", "factor": "rate_cap", "level": 0.5, "cap_mbps": 0.5,
     "attenuation_db": 0.0, "threshold_db": 5.0},
    {"case_id": "cap_2", "factor": "rate_cap", "level": 2.0, "cap_mbps": 2.0,
     "attenuation_db": 0.0, "threshold_db": 5.0},
    {"case_id": "reference", "factor": "rate_cap", "level": 10.0, "cap_mbps": 10.0,
     "attenuation_db": 0.0, "threshold_db": 5.0},
    {"case_id": "atten_10", "factor": "attenuation", "level": 10.0, "cap_mbps": 10.0,
     "attenuation_db": 10.0, "threshold_db": 5.0},
    {"case_id": "atten_15", "factor": "attenuation", "level": 15.0, "cap_mbps": 10.0,
     "attenuation_db": 15.0, "threshold_db": 5.0},
    {"case_id": "threshold_0_atten15", "factor": "threshold", "level": 0.0,
     "cap_mbps": 10.0, "attenuation_db": 15.0, "threshold_db": 0.0},
    {"case_id": "threshold_10_atten15", "factor": "threshold", "level": 10.0,
     "cap_mbps": 10.0, "attenuation_db": 15.0, "threshold_db": 10.0},
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_observations(path: Path) -> list[dict[str, Any]]:
    rows = list(csv.DictReader(path.open(encoding="utf-8-sig", newline="")))
    result = []
    for row in rows:
        result.append({
            "frame": int(row["frame"]),
            "timestamp_s": float(row["timestamp_s"]),
            "uav_antenna_xyz_m": tuple(float(v) for v in ast.literal_eval(row["uav_antenna_xyz_m"])),
            "ugv_antenna_xyz_m": tuple(float(v) for v in ast.literal_eval(row["ugv_antenna_xyz_m"])),
            "horizontal_m": float(row["horizontal_m"]),
            "height_m": float(row["height_m"]),
            "distance_m": float(row["distance_m"]),
            "elevation_deg": float(row["elevation_deg"]),
            "geo_los": int(row["geo_los"]),
            "first_static_blocker": row["first_static_blocker"] or None,
            "ray_hit_count": int(row["ray_hit_count"]),
        })
    if not result:
        raise ValueError("Spatial trace is empty")
    return result


def actual_semantic_sizes(path: Path, config: A2GTransportConfig) -> list[int]:
    messages = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    messages = [row for row in messages if row.get("event") == "sent" and row.get("source") == "uav"]
    sizes = [len(config.serialize_sender(row)[1]) for row in messages]
    if not sizes:
        raise ValueError("No repaired-version UAV semantic messages found")
    return sizes


def make_offers(start_s: float, end_s: float, semantic_sizes: list[int],
                feature_bytes: int, raw_bytes: int) -> list[tuple[float, str, str, int]]:
    offers = []
    index = 0
    while start_s + index * 0.5 < end_s - 0.25:
        offers.append((start_s + index * 0.5, f"semantic_{index:03d}", "semantic",
                       semantic_sizes[index % len(semantic_sizes)]))
        index += 1
    for offset, message_id, kind, size in (
        (6.0, "feature_0", "feature", feature_bytes),
        (8.0, "raw_0", "raw", raw_bytes),
        (12.0, "feature_1", "feature", feature_bytes),
    ):
        if start_s + offset < end_s:
            offers.append((start_s + offset, message_id, kind, size))
    return offers


def run_queue(rows: list[dict[str, Any]], offers: list[tuple[float, str, str, int]],
              config: A2GTransportConfig, group: str, attenuation_db: float,
              end_s: float) -> tuple[A2GTransportQueue, list[float]]:
    timeline = [(float(row["timestamp_s"]), 0, row) for row in rows]
    timeline.extend((when, 1, (message_id, kind, size)) for when, message_id, kind, size in offers)
    timeline.sort(key=lambda item: (item[0], item[1]))
    queue = A2GTransportQueue(config, group)
    rates = []
    for when, event_kind, content in timeline:
        if event_kind == 0:
            snr_db = float(content[f"{group}_snr_db"]) - attenuation_db
            queue.update_channel(when, snr_db=snr_db, distance_m=float(content["distance_m"]))
            rates.append(config.rate_bit_s(snr_db))
        else:
            message_id, _, size = content
            queue.enqueue({"message_id": message_id, "source_timestamp_s": when},
                          enqueue_timestamp_s=when, payload_bytes_override=size)
    queue.end_episode(end_s)
    return queue, rates


def summarize(queue: A2GTransportQueue, offers: list[tuple[float, str, str, int]],
              rates: list[float], case: dict[str, Any], seed: int, group: str) -> dict[str, Any]:
    kinds = {message_id: kind for _, message_id, kind, _ in offers}
    valid = [row for row in queue.records if row.status == "available_valid"]
    row: dict[str, Any] = {
        **case, "seed": seed, "group": group,
        "zero_rate_fraction": sum(value == 0 for value in rates) / len(rates),
        "variable_rate_fraction": sum(0 < value < queue.config.rate_cap_bit_s for value in rates) / len(rates),
        "median_rate_mbps": statistics.median(rates) / 1e6,
        "median_valid_age_ms": (1000 * statistics.median(item.t_arr - item.t_obs for item in valid)
                                if valid else None),
        "mean_aoi_s": queue.summary()["mean_aoi_after_first_delivery_s"],
    }
    for kind in ("semantic", "feature", "raw"):
        records = [item for item in queue.records if kinds[item.message_id] == kind]
        valid_count = sum(item.status == "available_valid" for item in records)
        expired_count = sum(item.status.startswith("expired_") for item in records)
        row[f"{kind}_offered"] = len(records)
        row[f"{kind}_valid"] = valid_count
        row[f"{kind}_expired"] = expired_count
        row[f"{kind}_delivery_fraction"] = valid_count / len(records) if records else None
    return row


def check_monotonic(rows: list[dict[str, Any]], factor: str, ordered_cases: list[str]) -> bool:
    selected = {(row["seed"], row["group"], row["case_id"]): row for row in rows}
    for seed in SEEDS:
        for group in ("geo", "prob"):
            sequence = [selected[(seed, group, case_id)] for case_id in ordered_cases]
            for kind in ("semantic", "feature", "raw"):
                values = [row[f"{kind}_valid"] for row in sequence]
                if factor == "rate_cap" and any(right < left for left, right in zip(values, values[1:])):
                    return False
                if factor != "rate_cap" and any(right > left for left, right in zip(values, values[1:])):
                    return False
    return True


def plot_results(rows: list[dict[str, Any]], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "font.family": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
        "axes.linewidth": 0.8, "savefig.dpi": 300,
    })
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.2))
    colors = {"semantic": "#0072B2", "feature": "#E69F00", "raw": "#D55E00"}
    labels = {"semantic": "真实目标语义", "feature": "特征占位载荷", "raw": "图像占位载荷"}

    cap_ids = ["cap_0p5", "cap_2", "reference"]
    caps = [0.5, 2, 10]
    for kind in ("semantic", "feature", "raw"):
        means = []
        lows = []
        highs = []
        for case_id in cap_ids:
            values = [100 * row[f"{kind}_delivery_fraction"] for row in rows if row["case_id"] == case_id]
            means.append(statistics.mean(values)); lows.append(min(values)); highs.append(max(values))
        axes[0].errorbar(caps, means, yerr=[np.subtract(means, lows), np.subtract(highs, means)],
                         marker="o", lw=1.6, capsize=3, color=colors[kind], label=labels[kind])
    axes[0].set_xscale("log")
    axes[0].set_xticks(caps, ["0.5", "2", "10"])
    axes[0].set_xlabel("业务速率上限（Mb/s）")
    axes[0].set_ylabel("TTL 内到达率（%）")
    axes[0].set_title("(a) 载荷大小与带宽上限")
    axes[0].legend(frameon=False, fontsize=8.8)

    for axis, case_ids, levels, title, xlabel in (
        (axes[1], ["reference", "atten_10", "atten_15"], [0, 10, 15],
         "(b) 目标语义对附加衰减的敏感性", "附加衰减（dB）"),
        (axes[2], ["threshold_0_atten15", "atten_15", "threshold_10_atten15"], [0, 5, 10],
         "(c) 边界链路下的接收门限", "停传门限（dB）"),
    ):
        for group, color, marker, label in (
            ("geo", "#0072B2", "o", "几何射线"), ("prob", "#009E73", "s", "仰角概率"),
        ):
            means, lows, highs = [], [], []
            for case_id in case_ids:
                values = [100 * row["semantic_delivery_fraction"] for row in rows
                          if row["case_id"] == case_id and row["group"] == group]
                means.append(statistics.mean(values)); lows.append(min(values)); highs.append(max(values))
            axis.errorbar(levels, means, yerr=[np.subtract(means, lows), np.subtract(highs, means)],
                          marker=marker, lw=1.6, capsize=3, color=color, label=label)
        axis.set_xlabel(xlabel)
        axis.set_title(title)
        axis.legend(frameon=False, fontsize=8.8)
    for axis in axes:
        axis.set_ylim(-4, 104)
        axis.set_yticks([0, 25, 50, 75, 100])
        axis.grid(axis="y", alpha=0.25, linewidth=0.6)
    axes[1].set_ylabel("目标语义 TTL 内到达率（%）")
    axes[2].set_ylabel("目标语义 TTL 内到达率（%）")
    fig.suptitle("A2G 最小规模敏感性实验：3 个配对信道种子（均值及范围）", fontsize=14, y=1.02)
    fig.tight_layout()
    for extension in ("png", "pdf"):
        fig.savefig(output.with_suffix(f".{extension}"), dpi=300, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--messages", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--spatial-config", type=Path,
                        default=ROOT / "configs/communication/a2g_spatial_v1.yaml")
    parser.add_argument("--transport-config", type=Path,
                        default=ROOT / "configs/communication/a2g_transport_v1.yaml")
    args = parser.parse_args()

    spatial_mapping = yaml.safe_load(args.spatial_config.read_text(encoding="utf-8"))["spatial_link"]
    transport_mapping = yaml.safe_load(args.transport_config.read_text(encoding="utf-8"))["transport_link"]
    base_transport = A2GTransportConfig.from_mapping(
        transport_mapping, bandwidth_hz=float(spatial_mapping["bandwidth_hz"])
    )
    observations = load_observations(args.trace)
    period_s = float(transport_mapping["clock"]["rate_sample_period_s"])
    start_s = float(observations[0]["timestamp_s"])
    end_s = float(observations[-1]["timestamp_s"]) + period_s
    semantic_sizes = actual_semantic_sizes(args.messages, base_transport)
    offers = make_offers(start_s, end_s, semantic_sizes,
                         int(transport_mapping["probes"]["feature_bytes_placeholder"]),
                         int(transport_mapping["probes"]["raw_rgb_bytes_placeholder"]))

    tracks = {}
    for seed in SEEDS:
        spatial_config = A2GSpatialConfig.from_mapping(spatial_mapping, run_seed=seed)
        tracks[seed] = evaluate_track(observations, spatial_config)
    repeat = evaluate_track(observations, A2GSpatialConfig.from_mapping(spatial_mapping, run_seed=SEEDS[0]))
    repeatable = all(
        left["geo_snr_db"] == right["geo_snr_db"] and left["prob_snr_db"] == right["prob_snr_db"]
        for left, right in zip(tracks[SEEDS[0]], repeat)
    )

    rows = []
    event_dir = args.output / "events"
    event_dir.mkdir(parents=True, exist_ok=True)
    for case in CASES:
        config = replace(base_transport, rate_cap_bit_s=case["cap_mbps"] * 1e6,
                         snr_working_threshold_db=case["threshold_db"])
        for seed in SEEDS:
            for group in ("geo", "prob"):
                queue, rates = run_queue(tracks[seed], offers, config, group,
                                         case["attenuation_db"], end_s)
                rows.append(summarize(queue, offers, rates, case, seed, group))
                event_path = event_dir / f"{case['case_id']}_s{seed}_{group}.jsonl"
                event_path.write_text("".join(json.dumps(item, ensure_ascii=False) + "\n"
                                               for item in queue.events), encoding="utf-8")

    checks = {
        "design_has_21_paired_cases": len(CASES) * len(SEEDS) == 21 and len(rows) == 42,
        "same_seed_is_exactly_repeatable": repeatable,
        "rate_cap_response_is_monotonic": check_monotonic(
            rows, "rate_cap", ["cap_0p5", "cap_2", "reference"]),
        "attenuation_response_is_monotonic": check_monotonic(
            rows, "attenuation", ["reference", "atten_10", "atten_15"]),
        "threshold_response_is_monotonic_at_boundary": check_monotonic(
            rows, "threshold", ["threshold_0_atten15", "atten_15", "threshold_10_atten15"]),
        "valid_delivery_is_observed": any(row["semantic_valid"] > 0 for row in rows),
        "ttl_expiry_is_observed": any(
            row["semantic_expired"] + row["feature_expired"] + row["raw_expired"] > 0 for row in rows),
        "zero_and_variable_rate_are_observed": (
            any(row["zero_rate_fraction"] > 0 for row in rows)
            and any(row["variable_rate_fraction"] > 0 for row in rows)
        ),
        "real_repaired_message_sizes_used": len(semantic_sizes) >= 1,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "case_metrics.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    result = {
        "status": "PASS" if all(checks.values()) else "FAIL",
        "design": "7 unique one-factor conditions x 3 paired channel seeds; geo/prob both reported",
        "case_count": 21, "result_rows": len(rows), "seeds": list(SEEDS), "cases": list(CASES),
        "checks": checks,
        "payloads": {
            "semantic": "serialized UAV messages from repaired persistent_target_memory E1-C run",
            "semantic_message_count_in_source": len(semantic_sizes),
            "semantic_bytes": {"min": min(semantic_sizes), "median": statistics.median(semantic_sizes),
                               "max": max(semantic_sizes)},
            "feature_bytes_placeholder": int(transport_mapping["probes"]["feature_bytes_placeholder"]),
            "raw_rgb_bytes_placeholder": int(transport_mapping["probes"]["raw_rgb_bytes_placeholder"]),
        },
        "sources": {
            "trace": str(args.trace.resolve()), "trace_sha256": sha256(args.trace),
            "messages": str(args.messages.resolve()), "messages_sha256": sha256(args.messages),
            "spatial_config": str(args.spatial_config.resolve()), "spatial_config_sha256": sha256(args.spatial_config),
            "transport_config": str(args.transport_config.resolve()), "transport_config_sha256": sha256(args.transport_config),
        },
        "interpretation_boundary": (
            "Semantic payload sizes are real repaired-version messages. Feature and raw-image sizes are declared "
            "placeholders and support transport feasibility only. This is parameter sensitivity, not field calibration."
        ),
    }
    (args.output / "sensitivity_summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    plot_results(rows, args.output / "a2g_minimal_sensitivity")
    print(json.dumps({"status": result["status"], "checks": checks,
                      "cases": result["case_count"], "rows": len(rows),
                      "output": str(args.output.resolve())}, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

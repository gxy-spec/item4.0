#!/usr/bin/env python3
"""Replay one fixed trajectory under a declared adverse link-budget condition."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from communication.a2g_transport import A2GTransportConfig, A2GTransportQueue  # noqa: E402


def read_trace(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    keys = ("timestamp_s", "distance_m", "geo_snr_db", "prob_snr_db",
            "geo_los", "prob_los")
    return [{key: float(row[key]) for key in keys} for row in rows]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    spatial_path = ROOT / "configs/communication/a2g_spatial_v1.yaml"
    transport_path = ROOT / "configs/communication/a2g_transport_v1.yaml"
    spatial = yaml.safe_load(spatial_path.read_text(encoding="utf-8"))["spatial_link"]
    transport = yaml.safe_load(transport_path.read_text(encoding="utf-8"))["transport_link"]
    base = A2GTransportConfig.from_mapping(transport, bandwidth_hz=float(spatial["bandwidth_hz"]))
    config = replace(base, rate_cap_bit_s=10_000_000.0)

    # Both values are already declared endpoints in a2g_spatial_v1.yaml.
    tx_power_dbm = 10.0
    eta_nlos_db = 30.0
    tx_delta_db = tx_power_dbm - float(spatial["transmit_power_dbm"])
    nlos_delta_db = eta_nlos_db - float(spatial["eta_nlos_db"])
    rows = read_trace(args.trace)
    start_s = rows[0]["timestamp_s"]
    end_s = rows[-1]["timestamp_s"] + float(transport["clock"]["rate_sample_period_s"])
    adjusted: list[dict[str, float]] = []
    for row in rows:
        item = dict(row)
        for group in ("geo", "prob"):
            item[f"{group}_adjusted_snr_db"] = (
                row[f"{group}_snr_db"] + tx_delta_db
                - nlos_delta_db * (1.0 - row[f"{group}_los"])
            )
            item[f"{group}_rate_bit_s"] = config.rate_bit_s(item[f"{group}_adjusted_snr_db"])
        adjusted.append(item)

    offers = []
    index = 0
    while start_s + 0.5 * index < end_s - 0.25:
        when = start_s + 0.5 * index
        offers.append((when, f"semantic_{index:03d}"))
        index += 1
    timeline = [(row["timestamp_s"], 0, row) for row in adjusted]
    timeline.extend((when, 1, message_id) for when, message_id in offers)
    timeline.sort(key=lambda item: (item[0], item[1]))

    args.output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for group in ("geo", "prob"):
        queue = A2GTransportQueue(config, group)
        for when, kind, content in timeline:
            if kind == 0:
                queue.update_channel(
                    when, snr_db=content[f"{group}_adjusted_snr_db"],
                    distance_m=content["distance_m"],
                )
            else:
                queue.enqueue(
                    {"message_id": content, "source_timestamp_s": when},
                    enqueue_timestamp_s=when, payload_bytes_override=704,
                )
        queue.end_episode(end_s)
        rates = [row[f"{group}_rate_bit_s"] for row in adjusted]
        sample = queue.summary()
        sample.update({
            "zero_rate_sample_fraction": sum(rate == 0 for rate in rates) / len(rates),
            "below_cap_nonzero_sample_fraction": sum(0 < rate < config.rate_cap_bit_s for rate in rates) / len(rates),
            "at_cap_sample_fraction": sum(abs(rate - config.rate_cap_bit_s) < 1e-6 for rate in rates) / len(rates),
            "snr_db_min": min(row[f"{group}_adjusted_snr_db"] for row in adjusted),
            "snr_db_max": max(row[f"{group}_adjusted_snr_db"] for row in adjusted),
        })
        summaries[group] = sample
        (args.output / f"events_{group}.jsonl").write_text(
            "".join(json.dumps(event, ensure_ascii=False) + "\n" for event in queue.events),
            encoding="utf-8",
        )

    gap = 10 ** (config.snr_gap_db / 10.0)
    needed_spectral = config.rate_cap_bit_s / (config.resource_fraction * config.bandwidth_hz)
    cap_snr_db = 10 * math.log10(gap * (2 ** needed_spectral - 1))
    result = {
        "purpose": "mechanism validation only; not a calibrated formal channel condition",
        "same_saved_trajectory": str(args.trace),
        "changed_link_budget": {
            "transmit_power_dbm": {"baseline": spatial["transmit_power_dbm"], "validation": tx_power_dbm},
            "eta_nlos_db": {"baseline": spatial["eta_nlos_db"], "validation": eta_nlos_db},
            "values_are_declared_sensitivity_endpoints": True,
        },
        "unchanged": {"trajectory": True, "shadow_realization": True, "ttl_s": config.ttl_s,
                      "semantic_probe_bytes": 704, "rate_cap_mbps": 10.0},
        "working_threshold_snr_db": config.snr_working_threshold_db,
        "snr_needed_for_10_mbps_db": cap_snr_db,
        "groups": summaries,
    }
    (args.output / "summary.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                         "font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False})
    time_s = [row["timestamp_s"] - start_s for row in adjusted]
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.6), sharex=True)
    styles = (("geo", "#225B91", "-", "几何射线"),
              ("prob", "#D0822C", "--", "仰角概率"))
    for group, color, style, label in styles:
        axes[0].step(time_s, [row[f"{group}_adjusted_snr_db"] for row in adjusted],
                     where="post", color=color, linestyle=style, linewidth=1.7, label=label)
        axes[1].step(time_s, [row[f"{group}_rate_bit_s"] / 1e6 for row in adjusted],
                     where="post", color=color, linestyle=style, linewidth=1.7, label=label)
    axes[0].axhline(config.snr_working_threshold_db, color="#B33A3A", linewidth=1.1,
                    label="5 dB 停传门限")
    axes[0].axhline(cap_snr_db, color="#555555", linewidth=1.1, linestyle=":",
                    label=f"达到 10 Mb/s 所需 SNR（{cap_snr_db:.1f} dB）")
    axes[0].set_ylabel("SNR（dB）")
    axes[0].legend(ncol=2, frameon=False)
    axes[1].set_ylabel("可用传输速率（Mb/s）")
    axes[1].set_xlabel("同一条轨迹经过时间（秒）")
    axes[1].set_ylim(-0.3, 10.7)
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.set_xlim(0, end_s - start_s)
    fig.suptitle("弱链路机制验证：同一轨迹，10 dBm 发射功率，30 dB NLoS 损耗")
    fig.tight_layout()
    for extension in ("png", "pdf"):
        fig.savefig(args.output / f"weak_link_validation.{extension}", dpi=300,
                    bbox_inches="tight")
    plt.close(fig)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Create the compact publication-style summary for the perception baseline."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(r"E:\CarlaAirData\CityInspection_GOC\perception_benchmark_v1\results")


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    output = ROOT / "perception_stage_summary"
    output.mkdir(parents=True, exist_ok=True)
    entries = [
        {
            "name": "YOLO26x\nCOCO + HSV",
            "validation": read(ROOT / "yolo26x_hsv_uav_validation" / "threshold_calibration.json")["chosen"],
            "latency": read(ROOT / "yolo26x_hsv_uav_validation" / "summary.json")["median_latency_ms"],
        },
        {
            "name": "YOLOE-26s\nzero-shot",
            "validation": read(ROOT / "yoloe26s_zero_shot_uav_validation" / "threshold_calibration.json")["chosen"],
            "latency": read(ROOT / "yoloe26s_zero_shot_uav_validation" / "summary.json")["median_latency_ms"],
            "test": read(ROOT / "yoloe26s_zero_shot_uav_test_calibrated" / "summary.json"),
        },
        {
            "name": "YOLOE-26s\nnarrow pilot tune",
            "validation": read(ROOT / "yoloe26s_domain_pilot_uav_validation_raw" / "threshold_calibration.json")["chosen"],
            "latency": read(ROOT / "yoloe26s_domain_pilot_uav_validation_raw" / "summary.json")["median_latency_ms"],
            "test": read(ROOT / "yoloe26s_domain_pilot_uav_test_calibrated" / "summary.json"),
        },
    ]

    with (output / "model_comparison.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "model", "validation_recall", "validation_precision", "validation_fpr",
            "test_recall", "test_precision", "test_fpr", "median_latency_ms",
        ])
        writer.writeheader()
        for item in entries:
            test = item.get("test", {})
            writer.writerow({
                "model": item["name"].replace("\n", " "),
                "validation_recall": item["validation"]["recall"],
                "validation_precision": item["validation"]["precision"],
                "validation_fpr": item["validation"]["negative_false_trigger_rate"],
                "test_recall": test.get("selection_recall"),
                "test_precision": test.get("selection_frame_precision"),
                "test_fpr": test.get("negative_false_trigger_rate"),
                "median_latency_ms": item["latency"],
            })

    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 9, "axes.titlesize": 11,
        "axes.labelsize": 9, "axes.spines.top": False, "axes.spines.right": False,
    })
    colors = ["#6B7280", "#0F6CBD", "#D97706"]
    fig, axes = plt.subplots(1, 3, figsize=(11.4, 3.5), constrained_layout=True)

    ax = axes[0]
    for item, color in zip(entries, colors):
        val = item["validation"]
        ax.scatter(val["recall"], val["precision"], s=85, color=color, edgecolor="white", linewidth=0.8)
        ax.annotate(item["name"].replace("\n", " "), (val["recall"], val["precision"]),
                    xytext=(4, 5), textcoords="offset points", fontsize=7.7)
    ax.axhline(0.75, color="#9CA3AF", linestyle="--", linewidth=1)
    ax.set(xlim=(0, 0.65), ylim=(0, 0.9), xlabel="Recall", ylabel="Precision",
           title="(a) Validation operating point")
    ax.grid(alpha=0.18)

    ax = axes[1]
    test_entries = entries[1:]
    x = np.arange(len(test_entries))
    width = 0.23
    metrics = [
        ("selection_recall", "Recall", "#0F6CBD"),
        ("selection_frame_precision", "Precision", "#2E8B57"),
        ("negative_false_trigger_rate", "False trigger rate", "#C43D3D"),
    ]
    for offset, (key, label, color) in enumerate(metrics):
        values = [item["test"][key] for item in test_entries]
        bars = ax.bar(x + (offset - 1) * width, values, width, label=label, color=color)
        ax.bar_label(bars, fmt="%.2f", fontsize=7, padding=2)
    ax.set_xticks(x, [item["name"] for item in test_entries])
    ax.set_ylim(0, 0.65)
    ax.set_ylabel("Frame-level rate")
    ax.set_title("(b) Held-out scene test")
    ax.legend(frameon=False, fontsize=7, loc="upper right")
    ax.grid(axis="y", alpha=0.18)

    ax = axes[2]
    names = [item["name"] for item in entries]
    latencies = [item["latency"] for item in entries]
    bars = ax.barh(np.arange(len(entries)), latencies, color=colors)
    ax.set_yticks(np.arange(len(entries)), names)
    ax.invert_yaxis()
    ax.set_xlabel("Median latency per frame (ms)")
    ax.set_title("(c) Detector runtime")
    ax.bar_label(bars, fmt="%.1f ms", fontsize=8, padding=3)
    ax.set_xlim(0, max(latencies) * 1.25)
    ax.grid(axis="x", alpha=0.18)

    fig.suptitle("Open-vocabulary perception baseline and domain-adaptation pilot", fontsize=12, fontweight="semibold")
    fig.savefig(output / "perception_stage_summary.png", dpi=300, bbox_inches="tight")
    fig.savefig(output / "perception_stage_summary.pdf", bbox_inches="tight")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

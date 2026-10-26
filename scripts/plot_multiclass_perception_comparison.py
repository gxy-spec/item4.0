#!/usr/bin/env python3
"""Plot a protocol-matched held-out comparison from evaluation manifests."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("adapted", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    records = [json.loads(path.read_text(encoding="utf-8")) for path in (args.baseline, args.adapted)]
    for key in ("dataset", "split", "classes", "image_size", "batch_size", "protocol"):
        if records[0][key] != records[1][key]:
            raise ValueError(f"Evaluation mismatch: {key}")
    args.output.mkdir(parents=True, exist_ok=True)
    labels = ["Zero-shot", "Town10HD adapted"]
    keys = ["metrics/mAP50(B)", "metrics/mAP50-95(B)", "metrics/mAP50(M)", "metrics/mAP50-95(M)"]
    names = ["Box AP50", "Box AP50:95", "Mask AP50", "Mask AP50:95"]
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(8.4, 3.8), layout="constrained")
    x = np.arange(4)
    for index, (record, color) in enumerate(zip(records, ("#67788A", "#167B79"))):
        bars = ax.bar(x + (index - 0.5) * 0.34, [record["metrics"][key] for key in keys],
                      width=0.34, label=labels[index], color=color)
        ax.bar_label(bars, fmt="%.3f", padding=3, fontsize=9)
    ax.set(xticks=x, xticklabels=names, ylim=(0, 1), ylabel="Average precision",
           title="Held-out Town10HD regions · four vehicle categories")
    ax.set_axisbelow(True)
    ax.grid(axis="y", alpha=0.18)
    ax.legend(frameon=False, loc="upper left")
    fig.savefig(args.output / "test_comparison.png", dpi=300)
    fig.savefig(args.output / "test_comparison.pdf")
    plt.close(fig)
    with (args.output / "test_comparison.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["model", *names, "inference_ms_batch8"])
        for name, record in zip(labels, records):
            writer.writerow([name, *[record["metrics"][key] for key in keys], record["speed_ms_per_image"]["inference"]])
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

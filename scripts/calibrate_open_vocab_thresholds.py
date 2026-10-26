#!/usr/bin/env python3
"""Choose perception thresholds on validation predictions, never on test data."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("predictions", type=Path, help="frame_metrics.jsonl from validation split")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-precision", type=float, default=0.75)
    parser.add_argument("--maximum-negative-fpr", type=float, default=0.05)
    return parser.parse_args()


def load(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def evaluate(rows: list[dict[str, Any]], detector_threshold: float, color_threshold: float) -> dict[str, Any]:
    selected = []
    for row in rows:
        candidates = [item for item in row["predictions"] if (
            item["category_match_score"] >= 0.5
            and item["detector_score"] >= detector_threshold
            and item["target_color_ratio"] >= color_threshold
        )]
        selected.append(max(candidates, key=lambda item: item["selection_score"], default=None))
    evaluable = [index for index, row in enumerate(rows) if row["evaluable_target"]]
    negative = [index for index, row in enumerate(rows) if row["negative_frame"]]
    correct = sum(
        selected[index] is not None and selected[index]["contains_target_center"]
        for index in evaluable
    )
    selected_count = sum(item is not None for item in selected)
    false_triggers = sum(selected[index] is not None for index in negative)
    recall = correct / max(1, len(evaluable))
    precision = correct / max(1, selected_count)
    negative_fpr = false_triggers / max(1, len(negative))
    f1 = 2.0 * precision * recall / max(1e-12, precision + recall)
    return {
        "detector_threshold": detector_threshold,
        "color_threshold": color_threshold,
        "recall": recall,
        "precision": precision,
        "negative_false_trigger_rate": negative_fpr,
        "f1": f1,
        "correct_frames": correct,
        "selected_frames": selected_count,
    }


def main() -> int:
    args = parse_args()
    rows = load(args.predictions.resolve())
    grid = [
        evaluate(rows, detector, color)
        for detector in (0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10)
        for color in (0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80)
    ]
    feasible = [item for item in grid if (
        item["precision"] >= args.minimum_precision
        and item["negative_false_trigger_rate"] <= args.maximum_negative_fpr
    )]
    chosen = max(
        feasible or grid,
        key=lambda item: (item["recall"], item["f1"], item["precision"]),
    )
    payload = {
        "schema_version": "open_vocab_threshold_calibration_v1",
        "source_predictions": str(args.predictions.resolve()),
        "selection_rule": {
            "minimum_precision": args.minimum_precision,
            "maximum_negative_false_trigger_rate": args.maximum_negative_fpr,
        },
        "constraint_satisfied": bool(feasible),
        "chosen": chosen,
        "grid": grid,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in payload.items() if key != "grid"}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

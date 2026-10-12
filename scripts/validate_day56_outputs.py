#!/usr/bin/env python3
"""Validate all paired-seed replay files and sparse-frame candidate joins."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2

ROOT = Path("E:/CarlaAirData/CityInspection_GOC/e7_prechannel_repair/day56_paired_seeds")


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main() -> int:
    records = []
    for seed in (1001, 2001, 3001):
        summary = json.loads((ROOT / f"S{seed}" / "paired_acceptance.json").read_text(encoding="utf-8"))
        for group in "ABC":
            run_dir = Path(summary["metrics"][group]["run_dir"])
            video = Path(summary["videos"][group])
            with (run_dir / "synchronization" / "frame_index.csv").open(encoding="utf-8-sig", newline="") as stream:
                saved = {int(row["frame"]) for row in csv.DictReader(stream) if int(row["sensor_files_saved"]) == 1}
            box_counts = {}
            for role in ("uav", "ugv"):
                rows = jsonl(run_dir / "perception" / f"{role}_target_candidates_synchronized.jsonl")
                box_counts[role] = sum(len(row["candidates"]) for row in rows if int(row["frame_id"]) in saved)
            cap = cv2.VideoCapture(str(video))
            opened = cap.isOpened()
            frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if opened else 0
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) if opened else 0
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) if opened else 0
            first_ok = cap.read()[0] if opened else False
            if opened and frames > 1:
                cap.set(cv2.CAP_PROP_POS_FRAMES, frames - 1)
                last_ok = cap.read()[0]
            else:
                last_ok = False
            cap.release()
            passed = opened and first_ok and last_ok and frames == len(saved) and width == 1920 and height == 1080
            passed = passed and sum(box_counts.values()) > 0 and video.stat().st_size > 1_000_000
            records.append({"seed": seed, "group": group, "passed": bool(passed), "video": str(video),
                            "video_frames": frames, "saved_source_frames": len(saved),
                            "resolution": [width, height], "candidate_boxes_joined_to_saved_frames": box_counts,
                            "bytes": video.stat().st_size})
    report = {"status": "PASS" if all(record["passed"] for record in records) else "FAIL",
              "videos": records, "scope": "file integrity and source-frame/candidate join; does not establish perceptual correctness"}
    path = ROOT / "replay_validation.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"status": report["status"], "passed": sum(r["passed"] for r in records),
                      "total": len(records), "report": str(path)}, ensure_ascii=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Build and validate sparse synchronized five-view replays for a frozen batch."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import cv2


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--builder", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=2.0)
    args = parser.parse_args()
    root = args.root.resolve()
    progress = json.loads((root / "batch_progress.json").read_text(encoding="utf-8"))
    results = progress.get("results", [])
    if progress.get("status") != "COMPLETE" or len(results) != 36:
        raise RuntimeError("Refusing to build replays: batch is not COMPLETE with 36 indexed runs")
    if any(item.get("status") != "PASS" for item in results):
        raise RuntimeError("Refusing to build replays: one or more indexed runs did not PASS")

    entries: list[dict] = []
    for index, item in enumerate(results, 1):
        run_dir = Path(item["run_dir"])
        report_path = Path(item["validation_report"])
        native = json.loads(report_path.read_text(encoding="utf-8"))
        if native.get("status") != "PASS":
            raise RuntimeError(f"Native validation not PASS for {item['label']}: {report_path}")
        output = run_dir / "preview" / f"{item['label']}_five_view_boxed_sparse.mp4"
        command = [sys.executable, str(args.builder.resolve()), "--run-dir", str(run_dir),
                   "--fps", str(args.fps), "--output", str(output)]
        print(f"[{index}/36] Rendering {item['label']}", flush=True)
        subprocess.run(command, check=True)

        manifest_path = output.with_suffix(".json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        frame_rows = [row for row in read_csv(run_dir / "synchronization" / "frame_index.csv")
                      if int(row.get("sensor_files_saved", "1")) == 1]
        expected_ids = [int(row["frame"]) for row in frame_rows]
        source_spreads = [int(row["frame_spread"]) for row in frame_rows]
        if not expected_ids or any(spread != 0 for spread in source_spreads):
            raise RuntimeError(f"No exact-frame samples or nonzero frame spread in {item['label']}")
        for frame in expected_ids:
            for sensor_path in (
                run_dir / "sensors" / "uav" / "rgb" / f"{frame:08d}.png",
                run_dir / "sensors" / "uav" / "depth_colour" / f"{frame:08d}.png",
                run_dir / "sensors" / "ugv" / "rgb" / f"{frame:08d}.png",
                run_dir / "sensors" / "ugv" / "depth_colour" / f"{frame:08d}.png",
            ):
                if not sensor_path.is_file():
                    raise FileNotFoundError(f"{item['label']} missing synchronized input {sensor_path}")

        capture = cv2.VideoCapture(str(output))
        decoded = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ok_first, first = capture.read()
        if decoded > 1:
            capture.set(cv2.CAP_PROP_POS_FRAMES, decoded - 1)
        ok_last, _ = capture.read()
        capture.release()
        if not (ok_first and ok_last and first is not None):
            raise RuntimeError(f"Video decode failed for {item['label']}: {output}")
        if decoded != len(expected_ids) or (width, height) != (1920, 1080):
            raise RuntimeError(
                f"Video geometry/count mismatch for {item['label']}: "
                f"decoded={decoded}, expected={len(expected_ids)}, size={width}x{height}"
            )
        first_frame = run_dir / "preview" / f"{item['label']}_five_view_first_frame.png"
        if not cv2.imwrite(str(first_frame), first):
            raise RuntimeError(f"Could not write preview frame: {first_frame}")
        if int(manifest.get("frame_count", -1)) != len(expected_ids):
            raise RuntimeError(f"Manifest frame count mismatch for {item['label']}")
        if manifest.get("frame_alignment", "").find("exact CARLA frame ID") < 0:
            raise RuntimeError(f"Manifest does not confirm exact frame alignment for {item['label']}")
        if not manifest.get("sparse_sensor_archive"):
            raise RuntimeError(f"Expected sparse archive disclosure missing for {item['label']}")
        entries.append({
            "label": item["label"], "case": item["case"], "group": item["group"],
            "seed": item.get("seed"), "run_dir": str(run_dir),
            "native_validation": str(report_path), "native_status": native.get("status"),
            "video": str(output), "video_manifest": str(manifest_path),
            "first_frame_preview": str(first_frame), "video_status": "PASS",
            "frame_count": decoded, "source_sample_count": len(expected_ids),
            "source_frame_spread_max": max(source_spreads), "width": width, "height": height,
            "median_persisted_sample_interval_seconds": manifest.get("median_persisted_sample_interval_seconds"),
            "sparse_sensor_archive": manifest.get("sparse_sensor_archive"),
            "first_source_frame": expected_ids[0], "last_source_frame": expected_ids[-1],
            "video_bytes": output.stat().st_size,
        })
        print(f"[{index}/36] PASS {item['label']} ({decoded} saved samples)", flush=True)

    report = {
        "status": "PASS" if len(entries) == 36 else "FAIL",
        "batch_root": str(root), "batch_status": progress.get("status"),
        "replays_expected": 36, "replays_validated": len(entries),
        "fps": args.fps,
        "scope": "Sparse synchronized replay of persisted sensor samples; not a continuous/dense video.",
        "checks": {
            "all_36_native_validation_reports_pass": all(e["native_status"] == "PASS" for e in entries),
            "all_36_videos_decode_first_and_last_frames": len(entries) == 36,
            "exact_saved_sample_count_matches_video_frames": all(e["frame_count"] == e["source_sample_count"] for e in entries),
            "four_sensor_streams_exist_per_source_frame": len(entries) == 36,
            "zero_four_stream_frame_spread": all(e["source_frame_spread_max"] == 0 for e in entries),
            "1920x1080_five_view_canvas": all(e["width"] == 1920 and e["height"] == 1080 for e in entries),
            "sparse_archive_disclosed_in_every_manifest": all(e["sparse_sensor_archive"] for e in entries),
        },
        "entries": entries,
    }
    report_path = root / "replay_validation.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"REPLAY_VALIDATION={report_path}")
    print(f"STATUS={report['status']} ({len(entries)}/36)")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

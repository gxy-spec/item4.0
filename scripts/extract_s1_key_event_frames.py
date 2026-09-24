#!/usr/bin/env python3
"""Copy the nearest retained synchronized sensor frame for each key S1 event."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
from pathlib import Path


KEY_EVENTS = ("MESSAGE_SENT", "TARGET_VERIFIED", "ARRIVED_SAFE", "COMPLETED")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()

    with (run_dir / "synchronization" / "frame_index.csv").open(
        encoding="utf-8-sig", newline=""
    ) as handle:
        rows = list(csv.DictReader(handle))
    retained = [row for row in rows if int(row.get("sensor_files_saved", "1")) == 1]
    if not retained:
        raise RuntimeError("No retained synchronized sensor frames")

    events = [
        json.loads(line)
        for line in (run_dir / "task" / "task_events.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    event_by_state = {event.get("new_state"): event for event in events}
    output_root = run_dir / "preview" / "key_events"
    output_root.mkdir(parents=True, exist_ok=True)
    manifest: list[dict] = []

    for state in KEY_EVENTS:
        event = event_by_state.get(state)
        if event is None:
            continue
        event_frame = int(event["frame"])
        nearest = min(retained, key=lambda row: abs(int(row["frame"]) - event_frame))
        retained_frame = int(nearest["frame"])
        event_dir = output_root / state.lower()
        event_dir.mkdir(parents=True, exist_ok=True)
        copied: dict[str, str] = {}
        paths = {
            "uav_rgb": run_dir / "sensors" / "uav" / "rgb" / f"{retained_frame:08d}.png",
            "uav_depth": run_dir / "sensors" / "uav" / "depth_colour" / f"{retained_frame:08d}.png",
            "ugv_rgb": run_dir / "sensors" / "ugv" / "rgb" / f"{retained_frame:08d}.png",
            "ugv_depth": run_dir / "sensors" / "ugv" / "depth_colour" / f"{retained_frame:08d}.png",
        }
        for name, source in paths.items():
            if not source.exists():
                raise FileNotFoundError(source)
            destination = event_dir / f"{name}.png"
            shutil.copy2(source, destination)
            copied[name] = str(destination)
        manifest.append(
            {
                "event": state,
                "event_frame": event_frame,
                "retained_frame": retained_frame,
                "frame_offset": retained_frame - event_frame,
                "files": copied,
            }
        )

    (output_root / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(output_root / "manifest.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

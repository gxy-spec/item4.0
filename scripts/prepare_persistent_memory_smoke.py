#!/usr/bin/env python3
"""Freeze three C-policy scenes for the packet-TTL/target-memory smoke test.

The previously completed 36-run v2/v3 batches remain untouched.  These three
scenes exercise stable observation (N1), intermittent UAV information (E1),
and a west-district route with distractor observations (W2).
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "configs" / "experiments" / "multiscene_abc_expiry_coast_20261010"
DEST = ROOT / "configs" / "experiments" / "persistent_memory_smoke_20261012"
DATA = Path(r"E:\CarlaAirData\CityInspection_GOC\persistent_memory_smoke_20261012")
CASES = ("N1", "E1", "W2")


def main() -> None:
    source_index = json.loads((SOURCE / "formal_36_run_index.json").read_text(encoding="utf-8"))
    DEST.mkdir(parents=True, exist_ok=True)
    selected = []
    for item in source_index["runs"]:
        if item["case"] not in CASES or item["group"] != "C":
            continue
        case = item["case"]
        config = yaml.safe_load((SOURCE / f"{case}_C_formal.yaml").read_text(encoding="utf-8"))
        channel = config["dynamic"]["persistent_target_communication"]
        channel["expiry_policy"] = "persistent_target_memory"
        channel["memory_association_radius_m"] = 10.0
        channel["challenger_window_s"] = 4.0
        channel["challenger_min_updates"] = 3
        channel["challenger_min_score_margin"] = 0.08
        channel["challenger_old_unseen_s"] = 2.0
        output_root = DATA / case / "C"
        config["output"]["root"] = str(output_root).replace("\\", "/")
        config["acceptance"]["minimum_target_routes_planned"] = 1
        config["acceptance"]["maximum_memory_commit_error_m"] = 12.0
        dest_config = DEST / f"{case}_C_memory.yaml"
        dest_config.write_text(
            yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8"
        )
        selected.append({**item, "config": str(dest_config), "output_root": str(output_root),
                         "policy": "dual_persistent_target_memory"})
    assert len(selected) == len(CASES)
    index = {
        "experiment": "Three-scene persistent target-memory smoke (not a 36-run paper comparison)",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_index": str(SOURCE / "formal_36_run_index.json"),
        "data_root": str(DATA),
        "cases": list(CASES),
        "runs": selected,
    }
    index_path = DEST / "three_scene_run_index.json"
    index_path.write_text(json.dumps(index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(index_path)


if __name__ == "__main__":
    main()

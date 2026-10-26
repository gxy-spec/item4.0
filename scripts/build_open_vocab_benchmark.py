#!/usr/bin/env python3
"""Build a scene-grouped image benchmark from frozen CARLA task replays."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from itertools import product
from pathlib import Path
from typing import Any

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.perception.candidate_pipeline import project_world_point, transform_matrix  # noqa: E402
from src.perception.task_spec import TaskSpec  # noqa: E402


SPLIT_BY_SCENE_NUMBER = {"1": "train", "2": "validation", "3": "test"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-root",
        type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\multiscene_abc_formal_20261010_v2"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\perception_benchmark_v1"),
    )
    parser.add_argument("--task", type=Path, default=ROOT / "configs/tasks/red_van_approach.yaml")
    parser.add_argument("--devices", nargs="+", choices=("uav", "ugv"), default=("uav", "ugv"))
    return parser.parse_args()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def projected_actor_box(
    actor: dict[str, Any], sensor_transform: dict[str, list[float]], width: int = 800,
    height: int = 600, fov_degrees: float = 100.0,
) -> tuple[list[float] | None, list[float], float]:
    """Project an actor's approximate 3-D bounding cuboid into the image.

    Frozen logs contain extents but not CARLA's bounding-box local offset.  The
    vehicle centre is therefore placed one vertical extent above actor origin.
    The resulting box is evaluation-only and its approximation is declared in
    the benchmark metadata.
    """
    ex, ey, ez = map(float, actor["bbox_extent_xyz"])
    actor_transform = {
        "location": [float(actor[key]) for key in ("x", "y", "z")],
        "rotation": [float(actor[key]) for key in ("pitch", "yaw", "roll")],
    }
    actor_to_world = transform_matrix(actor_transform)
    projected: list[tuple[float, float, float]] = []
    for sx, sy, sz in product((-1.0, 1.0), repeat=3):
        local = np.asarray([sx * ex, sy * ey, ez + sz * ez, 1.0])
        world = actor_to_world @ local
        projected.append(project_world_point(tuple(world[:3]), sensor_transform, width, height, fov_degrees))
    valid = [(u, v) for u, v, depth in projected if depth > 0.0]
    centre_xyz = (float(actor["x"]), float(actor["y"]), float(actor["z"]) + ez)
    centre_u, centre_v, centre_depth = project_world_point(
        centre_xyz, sensor_transform, width, height, fov_degrees
    )
    if not valid:
        return None, [centre_u, centre_v], centre_depth
    us, vs = zip(*valid)
    return [float(min(us)), float(min(vs)), float(max(us)), float(max(vs))], [centre_u, centre_v], centre_depth


def file_sha1(path: Path) -> str:
    digest = hashlib.sha1()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def depth_colour_visibility(
    path: Path | None, centre_uv: list[float], expected_depth_m: float,
    tolerance_m: float = 4.0,
) -> tuple[bool | None, float | None]:
    """Recover near-range depth from the archived visualization.

    ``colourise_depth`` stores B = 1.5 * depth / 100 for depths below 66.7 m.
    Saturated blue values are ambiguous and therefore remain ``unknown``.
    """
    if path is None or not path.exists() or not (0.0 < expected_depth_m < 65.0):
        return None, None
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        return None, None
    u, v = map(lambda value: int(round(value)), centre_uv)
    if not (0 <= u < image.shape[1] and 0 <= v < image.shape[0]):
        return False, None
    patch = image[max(0, v - 2):min(image.shape[0], v + 3), max(0, u - 2):min(image.shape[1], u + 3), 0]
    blue = float(np.median(patch))
    if blue >= 252.0:
        return None, None
    observed_depth_m = blue / 255.0 * (100.0 / 1.5)
    return abs(observed_depth_m - expected_depth_m) <= tolerance_m, observed_depth_m


def main() -> int:
    args = parse_args()
    import yaml

    task = yaml.safe_load(args.task.read_text(encoding="utf-8"))["task"]
    task_spec = TaskSpec.from_task_config(task)
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []
    skipped: Counter[str] = Counter()

    manifests = sorted(args.input_root.resolve().rglob("scene_manifest.json"))
    for manifest_path in manifests:
        run = manifest_path.parent
        scene = run.parent.parent.name
        policy = run.parent.name
        split = SPLIT_BY_SCENE_NUMBER.get(scene[-1:])
        if split is None:
            skipped["unknown_scene_split"] += 1
            continue
        actor_path = run / "actors/actor_states.jsonl"
        if not actor_path.exists():
            skipped["missing_actor_log"] += 1
            continue
        actors = load_jsonl(actor_path)
        target_by_frame = {
            int(item["frame"]): item for item in actors if item.get("role") == "target"
        }
        for device in args.devices:
            metadata_path = run / f"sensors/{device}/metadata/frames.jsonl"
            if not metadata_path.exists():
                skipped[f"missing_{device}_metadata"] += 1
                continue
            for metadata in load_jsonl(metadata_path):
                image_path = Path(str(metadata.get("rgb_path", "")))
                target = target_by_frame.get(int(metadata["frame"]))
                if target is None or not image_path.exists():
                    skipped["missing_image_or_target"] += 1
                    continue
                box, centre, depth = projected_actor_box(target, metadata["sensor_transform"])
                in_frame = bool(
                    depth > 0.0 and 0.0 <= centre[0] < 800.0 and 0.0 <= centre[1] < 600.0
                )
                depth_colour_path = (
                    None if not metadata.get("depth_colour_path")
                    else Path(str(metadata["depth_colour_path"]))
                )
                visible_proxy, observed_depth = depth_colour_visibility(
                    depth_colour_path, centre, depth
                ) if in_frame else (False, None)
                row_id = f"{scene}_{policy}_{run.name}_{device}_{int(metadata['frame']):08d}"
                rows.append({
                    "schema_version": "open_vocab_benchmark_v1",
                    "sample_id": row_id,
                    "split": split,
                    "scene": scene,
                    "policy": policy,
                    "run_id": run.name,
                    "device": device,
                    "frame": int(metadata["frame"]),
                    "timestamp_s": float(metadata["timestamp"]),
                    "image_path": str(image_path.resolve()),
                    "image_sha1": file_sha1(image_path),
                    "sensor_transform": metadata["sensor_transform"],
                    "target_actor": {
                        "type_id": target["type_id"],
                        "world_xyz": [float(target[key]) for key in ("x", "y", "z")],
                        "bbox_extent_xyz": [float(value) for value in target["bbox_extent_xyz"]],
                    },
                    "target_projected_bbox_xyxy": box,
                    "target_projected_center_uv": centre,
                    "target_camera_depth_m": depth,
                    "target_projected_in_frame": in_frame,
                    "target_visible_depth_proxy": visible_proxy,
                    "target_observed_depth_proxy_m": observed_depth,
                    "depth_colour_path": None if depth_colour_path is None else str(depth_colour_path.resolve()),
                    "task_spec": task_spec.to_dict(),
                })

    if not rows:
        raise RuntimeError(f"no benchmark samples found below {args.input_root}")
    rows.sort(key=lambda item: (item["split"], item["scene"], item["policy"], item["device"], item["frame"]))
    for split in ("train", "validation", "test"):
        split_rows = [item for item in rows if item["split"] == split]
        with (output / f"{split}.jsonl").open("w", encoding="utf-8") as handle:
            for item in split_rows:
                handle.write(json.dumps(item, ensure_ascii=False) + "\n")
    with (output / "all.jsonl").open("w", encoding="utf-8") as handle:
        for item in rows:
            handle.write(json.dumps(item, ensure_ascii=False) + "\n")

    unique_images = len({item["image_sha1"] for item in rows})
    summary = {
        "schema_version": "open_vocab_benchmark_summary_v1",
        "input_root": str(args.input_root.resolve()),
        "output_root": str(output),
        "task_spec": task_spec.to_dict(),
        "annotation_method": (
            "CARLA actor cuboid projection plus reversible near-range depth-colour visibility proxy; "
            "ambiguous saturated depth remains unknown"
        ),
        "sample_count": len(rows),
        "unique_image_count": unique_images,
        "exact_duplicate_count": len(rows) - unique_images,
        "counts_by_split": dict(Counter(item["split"] for item in rows)),
        "counts_by_device": dict(Counter(item["device"] for item in rows)),
        "counts_by_scene": dict(Counter(item["scene"] for item in rows)),
        "projected_in_frame_count": sum(item["target_projected_in_frame"] for item in rows),
        "visible_depth_proxy_count": sum(item["target_visible_depth_proxy"] is True for item in rows),
        "occluded_depth_proxy_count": sum(item["target_visible_depth_proxy"] is False for item in rows),
        "unknown_depth_proxy_count": sum(item["target_visible_depth_proxy"] is None for item in rows),
        "skipped": dict(skipped),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

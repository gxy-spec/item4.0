#!/usr/bin/env python3
"""Export synchronized CARLA RGB/instance frames to a scene-split YOLO dataset."""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import shutil
from typing import Any, Iterable

import cv2
import numpy as np
import yaml


CLASS_NAMES = ("van", "car", "truck", "bus")


def decode_actor_ids(encoded_bgr: np.ndarray, known_actor_ids: Iterable[int],
                     semantic_tags: Iterable[int] | None = None) -> tuple[np.ndarray, str]:
    """Decode CARLA's two ID bytes and self-check their byte order.

    CARLA releases and image readers expose the B/G bytes in different naming
    conventions.  The known actor IDs make the choice deterministic per frame.
    """
    first = encoded_bgr[:, :, 0].astype(np.uint16)
    second = encoded_bgr[:, :, 1].astype(np.uint16)
    options = {
        "B_plus_256G": first + 256 * second,
        "G_plus_256B": second + 256 * first,
    }
    known = np.asarray([int(value) % 65536 for value in known_actor_ids], dtype=np.uint16)
    if known.size == 0:
        return options["B_plus_256G"], "B_plus_256G"
    valid = np.ones(encoded_bgr.shape[:2],dtype=bool) if semantic_tags is None else np.isin(encoded_bgr[:,:,2],list(semantic_tags))
    scores = {name: int((np.isin(values, known) & valid).sum()) for name, values in options.items()}
    order = max(scores, key=scores.get)
    return options[order], order


def vehicle_actor_mask(actor_ids: np.ndarray, encoded_bgr: np.ndarray, actor_id: int,
                       semantic_tags: Iterable[int] = (14,15,16)) -> np.ndarray:
    """Disambiguate 16-bit actor IDs from pixels carrying non-vehicle semantics.

    This prevents semantic/id collisions; it does not certify RGB visibility
    under foliage or translucent materials. Such labels still require QA.
    """
    return (actor_ids == int(actor_id) % 65536) & np.isin(encoded_bgr[:,:,2],list(semantic_tags))


def category_from_actor(actor: dict[str, Any]) -> str | None:
    type_id = str(actor.get("type_id", "")).lower()
    if not type_id.startswith("vehicle."):
        return None
    base_type = str(dict(actor.get("attributes") or {}).get("base_type", "")).strip().lower()
    if base_type in CLASS_NAMES:
        return base_type
    if any(token in type_id for token in ("bus", "coach")):
        return "bus"
    if any(token in type_id for token in ("truck", "hgv", "carlacola", "firetruck", "cybertruck")):
        return "truck"
    if any(token in type_id for token in ("sprinter", "van", "ambulance")):
        return "van"
    return "car"


def canonical_actor_color(actor: dict[str, Any]) -> str:
    # Controlled six-paint renders establish that this asset keeps a white
    # primary body with blue trim. Its requested color attribute is not the
    # rendered body-paint truth. Apply the same correction to every variant.
    if str(actor.get('type_id','')).lower()=='vehicle.mitsubishi.fusorosa':
        return 'white'
    value = str(dict(actor.get("attributes") or {}).get("color", "")).strip()
    try:
        rgb = np.asarray([float(item) for item in value.split(",")], dtype=float)
    except (TypeError, ValueError):
        return "unknown"
    if rgb.shape != (3,):
        return "unknown"
    palette = {
        "red": (210, 35, 35), "blue": (35, 75, 190), "white": (225, 225, 225),
        "black": (25, 25, 25), "gray": (120, 120, 120), "yellow": (220, 190, 30),
    }
    return min(palette, key=lambda name: float(np.linalg.norm(rgb - np.asarray(palette[name]))))


def largest_polygon(mask: np.ndarray, width: int, height: int) -> list[float] | None:
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    epsilon = max(1.0, 0.002 * cv2.arcLength(contour, True))
    contour = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
    if len(contour) < 3:
        return None
    values: list[float] = []
    for x, y in contour:
        values.extend((float(x) / width, float(y) / height))
    return values


def load_actor_states(path: Path) -> dict[int, list[dict[str, Any]]]:
    by_frame: dict[int, list[dict[str, Any]]] = defaultdict(list)
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                item = json.loads(line)
                by_frame[int(item["frame"])].append(item)
    return by_frame


def scene_identity(run_dir: Path) -> str:
    config_path = run_dir / "resolved_config.yaml"
    if config_path.is_file():
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        map_name = str(dict(config.get("simulation") or {}).get("map", "unknown_map"))
        region_id = str(dict(config.get("region") or {}).get("region_id", run_dir.parent.name))
        return f"{map_name}:{region_id}"
    return run_dir.name


def scene_split(scene_key: str) -> str:
    bucket = int(hashlib.sha256(scene_key.encode("utf-8")).hexdigest()[:8], 16) % 100
    return "train" if bucket < 70 else "validation" if bucket < 85 else "test"


def discover_metadata(roots: list[Path]) -> list[Path]:
    found: list[Path] = []
    for root in roots:
        found.extend(root.rglob("sensors/*/metadata/frames.jsonl"))
    return sorted(set(found))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--minimum-visible-pixels", type=int, default=80)
    parser.add_argument("--every-n-frames", type=int, default=1)
    semantic_mode=parser.add_mutually_exclusive_group()
    semantic_mode.add_argument("--require-vehicle-semantic", dest="require_vehicle_semantic",action="store_true",default=True,
                        help="Require vehicle semantic labels for ID scoring and masks (default)")
    semantic_mode.add_argument("--legacy-unfiltered-actor-ids",dest="require_vehicle_semantic",action="store_false",
                        help="Historical reproduction only: allow the known ambiguous actor-ID-only export")
    parser.add_argument("--vehicle-semantic-tags",type=int,nargs='+',default=[14,15,16])
    args = parser.parse_args()

    for split in ("train", "validation", "test"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)

    records: list[dict[str, Any]] = []
    counters: dict[str, int] = defaultdict(int)
    actor_cache: dict[Path, dict[int, list[dict[str, Any]]]] = {}
    for metadata_path in discover_metadata(args.inputs):
        run_dir = metadata_path.parents[3]
        actor_path = run_dir / "actors" / "actor_states.jsonl"
        if not actor_path.exists():
            continue
        states = actor_cache.setdefault(actor_path, load_actor_states(actor_path))
        device = metadata_path.parents[1].name
        scene_key = scene_identity(run_dir)
        split = scene_split(scene_key)
        with metadata_path.open(encoding="utf-8") as handle:
            for index, line in enumerate(handle):
                if index % max(1, args.every_n_frames):
                    continue
                row = json.loads(line)
                counters["metadata_rows"] += 1
                rgb_value = row.get("rgb_path")
                instance_value = row.get("instance_segmentation_path")
                if not rgb_value or not instance_value:
                    counters["rows_not_persisted"] += 1
                    continue
                rgb_path = Path(str(rgb_value))
                instance_path = Path(str(instance_value))
                frame = int(row["frame"])
                actors = states.get(frame, [])
                if not rgb_path.is_file() or not instance_path.is_file():
                    counters["missing_sensor_file"] += 1
                    continue
                if not actors:
                    counters["missing_actor_state"] += 1
                    continue
                encoded = cv2.imread(str(instance_path), cv2.IMREAD_COLOR)
                if encoded is None:
                    counters["unreadable_instance_image"] += 1
                    continue
                actor_ids, byte_order = decode_actor_ids(encoded, (int(item["actor_id"]) for item in actors),
                    args.vehicle_semantic_tags if args.require_vehicle_semantic else None)
                height, width = encoded.shape[:2]
                labels: list[str] = []
                annotations: list[dict[str, Any]] = []
                for actor in actors:
                    category = category_from_actor(actor)
                    if category not in CLASS_NAMES:
                        continue
                    actor_id = int(actor["actor_id"])
                    mask = vehicle_actor_mask(actor_ids,encoded,actor_id,args.vehicle_semantic_tags) if args.require_vehicle_semantic else actor_ids == actor_id
                    visible_pixels = int(mask.sum())
                    if visible_pixels < args.minimum_visible_pixels:
                        continue
                    polygon = largest_polygon(mask, width, height)
                    if polygon is None:
                        continue
                    class_id = CLASS_NAMES.index(category)
                    labels.append(" ".join([str(class_id), *(f"{value:.6f}" for value in polygon)]))
                    semantic_role = str(
                        dict(actor.get("attributes") or {}).get("role_name")
                        or actor.get("role")
                        or "unknown"
                    )
                    annotations.append({
                        "actor_id": actor_id,
                        "role": semantic_role,
                        "category": category,
                        "color": canonical_actor_color(actor),
                        "visible_pixels": visible_pixels,
                        "type_id": actor.get("type_id"),
                    })
                if not labels:
                    counters["empty_frames"] += 1
                    continue
                stem = f"{hashlib.sha1(str(run_dir).encode()).hexdigest()[:10]}_{device}_{frame:08d}"
                image_out = args.output / "images" / split / f"{stem}{rgb_path.suffix.lower()}"
                label_out = args.output / "labels" / split / f"{stem}.txt"
                shutil.copy2(rgb_path, image_out)
                label_out.write_text("\n".join(labels) + "\n", encoding="utf-8")
                records.append({
                    "split": split, "scene": scene_key, "run": run_dir.name, "device": device, "frame": frame,
                    "image": str(image_out), "label": str(label_out), "instance_source": str(instance_path),
                    "instance_id_byte_order": byte_order, "annotations": annotations,
                })
                counters[f"{split}_frames"] += 1
                counters["instances"] += len(annotations)

    manifest = args.output / "manifest.jsonl"
    manifest.write_text("".join(json.dumps(item, ensure_ascii=False) + "\n" for item in records), encoding="utf-8")
    dataset = {
        "path": str(args.output.resolve()),
        "train": "images/train", "val": "images/validation", "test": "images/test",
        "names": {index: name for index, name in enumerate(CLASS_NAMES)},
    }
    (args.output / "dataset.yaml").write_text(yaml.safe_dump(dataset, sort_keys=False), encoding="utf-8")
    summary = {"schema_version": "carla_instance_dataset_v2" if args.require_vehicle_semantic else "carla_instance_dataset_v1",
        "vehicle_semantic_filter":bool(args.require_vehicle_semantic),
        "visibility_limitation":"instance masks alone do not certify RGB visibility under foliage",
        **dict(counters), "manifest": str(manifest)}
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

import json
from pathlib import Path

import jsonschema


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def valid_record():
    return {
        "schema_version": "target_candidate_v1",
        "device": "uav",
        "modality": "rgbd",
        "frame_id": 120,
        "timestamp_s": 6.0,
        "rgb_frame_id": 120,
        "depth_frame_id": 120,
        "rgb_depth_aligned": True,
        "coordinate_frame": "CARLA_WORLD_XYZ_M",
        "candidate_id": "120_00",
        "category": "car",
        "class_hypothesis": "vehicle",
        "color": "red",
        "bbox_xyxy": [10.0, 20.0, 80.0, 90.0],
        "detector_score": 0.8,
        "color_score": 0.9,
        "red_pixel_ratio": 0.6,
        "depth_m": 18.4,
        "depth_valid": True,
        "world_xyz_m": [-39.0, -40.0, 1.2],
        "proposal_source": "yolo26n_coco",
        "model": "yolo26n.pt",
        "temporal_hits": 2,
        "temporal_confirmed": True,
    }


def test_candidate_record_schema_accepts_aligned_rgbd_world_xyz():
    schema = json.loads((PROJECT_ROOT / "configs" / "schemas" / "target_candidate_v1.json").read_text(encoding="utf-8"))
    jsonschema.validate(valid_record(), schema)


def test_candidate_record_rejects_cross_device_frame_mismatch():
    schema = json.loads((PROJECT_ROOT / "configs" / "schemas" / "target_candidate_v1.json").read_text(encoding="utf-8"))
    record = valid_record()
    record["rgb_depth_aligned"] = False
    try:
        jsonschema.validate(record, schema)
    except jsonschema.ValidationError:
        return
    raise AssertionError("unaligned candidate record must be rejected")


def test_candidate_record_rejects_wrong_coordinate_frame_name():
    schema = json.loads((PROJECT_ROOT / "configs" / "schemas" / "target_candidate_v1.json").read_text(encoding="utf-8"))
    record = valid_record()
    record["coordinate_frame"] = "airsim_ned"
    try:
        jsonschema.validate(record, schema)
    except jsonschema.ValidationError:
        return
    raise AssertionError("candidate world coordinates must be CARLA world XYZ metres")

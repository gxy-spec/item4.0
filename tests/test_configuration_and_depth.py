from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from scenario.configuration import resolve_experiment, validate_schema
from sensors.depth import carla_depth_to_metres


def test_experiment_resolves_and_validates_after_region_freeze() -> None:
    config = resolve_experiment(PROJECT_ROOT / "configs" / "experiments" / "ci_e0_town10hd_zone_a.yaml")
    errors = validate_schema(config, PROJECT_ROOT / "configs" / "schemas" / "experiment_schema.json")
    assert errors == []


def test_carla_depth_decoder_endpoints() -> None:
    raw = np.asarray([[[0, 0, 0, 255], [255, 255, 255, 255]]], dtype=np.uint8).tobytes()
    decoded = carla_depth_to_metres(raw, width=2, height=1, maximum_m=1000.0)
    assert decoded.shape == (1, 2)
    assert float(decoded[0, 0]) == 0.0
    assert np.isclose(float(decoded[0, 1]), 1000.0)


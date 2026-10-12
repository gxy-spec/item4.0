from pathlib import Path

import pytest

from scripts.analyze_day3_survey_smoke import distance_to_route, first_correct_message
from src.scenario.configuration import resolve_experiment, validate_online_control_config, validate_schema
from scripts.run_stage1_dynamic import candidate_in_task_region


def test_finite_route_separation_does_not_use_infinite_line():
    assert distance_to_route([0, -30], [[0, 0], [0, -10]]) == pytest.approx(20)
    assert distance_to_route([3, -5], [[0, 0], [0, -10]]) == pytest.approx(3)


def test_no_effective_message_never_fabricates_discovery():
    assert first_correct_message([], "uav", [0, 0, 0], 5, 100) is None
    messages = [
        {"source": "uav", "world_position_xyz": [80, 0, 2], "source_timestamp_s": 101},
        {"source": "uav", "world_position_xyz": [2, 0, 2], "source_timestamp_s": 115},
    ]
    assert first_correct_message(messages, "uav", [0, 0, 0], 5, 100) == 15
    assert first_correct_message(messages, "ugv", [0, 0, 0], 5, 100) is None


def test_smoke_config_is_schema_valid_A_and_sensor_safety_only():
    root = Path(__file__).resolve().parents[1]
    config = resolve_experiment(root / "configs/experiments/ci_day3_survey_smoke.yaml")
    assert not validate_schema(config, root / "configs/schemas/dynamic_experiment_schema.json")
    assert not validate_online_control_config(config)
    dynamic = config["dynamic"]
    assert dynamic["persistent_target_communication"]["control_policy"] == "none"
    assert dynamic["ugv_patrol_route_target_spawn_point_id"] is None
    assert dynamic["sensor_warmup_ticks"] > 0
    assert dynamic["ugv_safety"]["hazard_detector"]["enabled"]
    assert config["output"]["save_depth_float_m"] is False
    assert dynamic["persistent_target_communication"]["enforce_task_region"] is True
    assert config["sensors"]["capture_every_fixed_tick"] is True
    assert config["sensors"]["frequency_hz"] * config["simulation"]["fixed_delta_seconds"] == 1


@pytest.mark.parametrize("xyz,expected", [([0, 0, 3], True), ([35, 0, 3], False), ([0, -90, 3], False), ([float("nan"), 0, 3], False), (None, False)])
def test_task_region_gate_uses_only_candidate_and_task_bounds(xyz, expected):
    bounds = {"x_min": -116, "x_max": 33, "y_min": -71, "y_max": 79}
    assert candidate_in_task_region({"world_position_xyz": xyz}, bounds) is expected

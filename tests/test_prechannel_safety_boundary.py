"""Regression tests for the non-Oracle online safety boundary."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from src.safety.rgbd_obstacle_guard import ObstacleGuardConfig, RGBDObstacleGuard
from src.scenario.configuration import resolve_experiment, validate_online_control_config


def guard() -> RGBDObstacleGuard:
    return RGBDObstacleGuard(
        ObstacleGuardConfig(
            pixel_stride=1,
            minimum_occupied_points=1,
            minimum_height_m=-100.0,
            maximum_height_m=100.0,
            corridor_half_width_m=100.0,
            recovery_clear_frames=2,
        )
    )


@pytest.mark.parametrize("depth", [np.full((16, 16), np.nan), np.zeros((16, 16))])
def test_invalid_depth_fails_closed_for_forward_and_pass_lane(depth: np.ndarray) -> None:
    observer = guard()
    result = observer.observe(depth, frame=1, timestamp=0.0, now_seconds=0.0, camera_to_vehicle=np.eye(4))
    assert result["state"] == "SENSOR_STALE"
    assert result["reason"] == "insufficient_valid_depth"
    assert observer.lane_occupancy(depth, np.eye(4), 3.5)["clear"] is False


def test_sparse_depth_fails_closed_but_valid_far_depth_is_clear() -> None:
    observer = guard()
    sparse = np.full((16, 16), np.nan, dtype=np.float32)
    sparse[0, 0] = 3.0
    assert observer.observe(sparse, 1, 0.0, 0.0, np.eye(4))["state"] == "SENSOR_STALE"
    assert observer.lane_occupancy(sparse, np.eye(4), 3.5)["clear"] is False
    far = np.full((16, 16), 30.0, dtype=np.float32)
    assert observer.observe(far, 2, 0.1, 0.1, np.eye(4))["state"] == "SENSOR_STALE"
    assert observer.observe(far, 3, 0.2, 0.2, np.eye(4))["state"] == "CLEAR"
    assert math.isnan(observer.observe(far, 4, 0.3, 0.3, np.eye(4))["nearest_obstacle_m"])


def test_same_frame_corruption_cannot_reuse_previous_clear_state() -> None:
    observer = guard()
    far = np.full((16, 16), 30.0, dtype=np.float32)
    assert observer.observe(far, 1, 0.0, 0.0, np.eye(4))["state"] == "CLEAR"
    invalid = np.full((16, 16), np.nan, dtype=np.float32)
    result = observer.observe(invalid, 1, 0.0, 0.0, np.eye(4))
    assert result["state"] == "SENSOR_STALE"
    assert result["reason"] == "insufficient_valid_depth"


def test_pass_lane_requires_depth_coverage_inside_that_lane() -> None:
    observer = RGBDObstacleGuard(ObstacleGuardConfig(pixel_stride=1))
    pose = np.eye(4)
    far = np.full((16, 16), 30.0, dtype=np.float32)
    assert observer.lane_occupancy(far, pose, 3.5)["clear"] is False
    lane_depth = np.full((16, 16), 10.0, dtype=np.float32)
    pose[2, 3] = -20.0  # A visible surface below the obstacle-height band.
    assert observer.lane_occupancy(lane_depth, pose, 3.5)["clear"] is True


def test_stop_and_stale_require_consecutive_clear_frames_to_recover() -> None:
    observer = guard()
    near = np.full((16, 16), 3.0, dtype=np.float32)
    far = np.full((16, 16), 30.0, dtype=np.float32)
    assert observer.observe(near, 1, 0.0, 0.0, np.eye(4))["state"] == "STOP"
    assert observer.observe(far, 2, 0.1, 0.1, np.eye(4))["state"] == "STOP"
    assert observer.observe(far, 3, 0.2, 0.2, np.eye(4))["state"] == "CLEAR"


def test_missing_sensor_brakes_without_reading_actor_truth(monkeypatch: pytest.MonkeyPatch) -> None:
    from scripts.run_stage1_dynamic import apply_safe_route_control

    class ForbiddenActor:
        def get_location(self):
            raise AssertionError("non-Oracle controller read actor truth")

    class Vehicle:
        def __init__(self) -> None:
            self.control = None

        def get_location(self):
            return SimpleNamespace(x=0.0, y=0.0, z=0.0)

        def apply_control(self, control):
            self.control = control

    monkeypatch.setitem(
        sys.modules,
        "carla",
        SimpleNamespace(VehicleControl=lambda **kwargs: SimpleNamespace(**kwargs)),
    )
    vehicle = Vehicle()
    decision = apply_safe_route_control(
        vehicle,
        [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
        2.0,
        ForbiddenActor(),
        [ForbiddenActor()],
        {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
        sensor_observation=None,
    )
    assert vehicle.control.brake == 1.0
    assert vehicle.control.throttle == 0.0
    assert decision["ugv_safety_mode"] == "sensor_stale_stop"


@pytest.mark.parametrize(
    "safety",
    [
        {},
        {"enabled": False, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
        {"enabled": True, "obstacle_source": "carla_actor_truth", "target_stop_enabled": False},
        {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": True},
    ],
)
def test_non_oracle_unsafe_control_config_is_rejected(safety: dict) -> None:
    config = {
        "experiment_type": "dual_device_persistent_communication",
        "dynamic": {"ugv_safety": safety, "uav_ignore_collision": False},
    }
    assert validate_online_control_config(config)


def test_safe_online_config_and_oracle_exception() -> None:
    safe = {
        "experiment_type": "dual_device_persistent_communication",
        "dynamic": {
            "ugv_safety": {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
            "uav_ignore_collision": False,
        },
    }
    assert validate_online_control_config(safe) == []
    assert validate_online_control_config({**safe, "experiment_type": "oracle_closed_loop_acceptance"}) == []
    assert validate_online_control_config({
        **safe,
        "dynamic": {**safe["dynamic"], "uav_ignore_collision": True},
    })


def test_target_message_expiry_must_return_to_rgbd_guarded_patrol() -> None:
    safe = {
        "experiment_type": "dual_device_persistent_communication",
        "dynamic": {
            "ugv_safety": {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
            "uav_ignore_collision": False,
            "persistent_target_communication": {
                "enabled": True,
                "expiry_policy": "return_to_patrol",
            },
        },
    }
    assert validate_online_control_config(safe) == []
    bounded_coast = {
        **safe,
        "dynamic": {
            **safe["dynamic"],
            "persistent_target_communication": {
                **safe["dynamic"]["persistent_target_communication"],
                "expiry_policy": "coast_then_patrol",
                "stale_grace_s": 5.0,
                "stale_speed_fraction": 0.5,
                "stale_max_distance_m": 8.0,
            },
        },
    }
    assert validate_online_control_config(bounded_coast) == []
    invalid_bounded_coast = {
        **bounded_coast,
        "dynamic": {
            **bounded_coast["dynamic"],
            "persistent_target_communication": {
                **bounded_coast["dynamic"]["persistent_target_communication"],
                "stale_grace_s": 0.0,
            },
        },
    }
    assert any(
        error["path"].endswith("stale_grace_s")
        for error in validate_online_control_config(invalid_bounded_coast)
    )
    stale_hold = {
        **safe,
        "dynamic": {
            **safe["dynamic"],
            "persistent_target_communication": {
                **safe["dynamic"]["persistent_target_communication"],
                "expiry_policy": "hold_after_grace",
                "stale_grace_s": 60.0,
            },
        },
    }
    errors = validate_online_control_config(stale_hold)
    assert any(error["path"].endswith("expiry_policy") for error in errors)


def test_persistent_target_memory_requires_bounded_hysteresis_parameters() -> None:
    safe = {
        "experiment_type": "dual_device_persistent_communication",
        "dynamic": {
            "ugv_safety": {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
            "uav_ignore_collision": False,
            "persistent_target_communication": {
                "enabled": True,
                "control_policy": "dual",
                "expiry_policy": "persistent_target_memory",
                "memory_association_radius_m": 10.0,
                "challenger_window_s": 4.0,
                "challenger_min_updates": 3,
                "challenger_min_score_margin": 0.08,
                "challenger_old_unseen_s": 2.0,
            },
        },
    }
    assert validate_online_control_config(safe) == []
    invalid = {
        **safe,
        "dynamic": {
            **safe["dynamic"],
            "persistent_target_communication": {
                **safe["dynamic"]["persistent_target_communication"],
                "challenger_min_updates": 0,
            },
        },
    }
    assert any(
        error["path"].endswith("challenger_min_updates")
        for error in validate_online_control_config(invalid)
    )


def test_all_runnable_non_oracle_configs_resolve_to_sensor_safety() -> None:
    config_dir = Path(__file__).resolve().parents[1] / "configs" / "experiments"
    excluded = {None, "scene_validation", "uav_target_candidate_baseline"}
    checked = 0
    for path in config_dir.glob("*.yaml"):
        resolved = resolve_experiment(path)
        if resolved.get("experiment_type") in excluded:
            continue
        assert validate_online_control_config(resolved) == [], path.name
        checked += 1
    assert checked >= 25

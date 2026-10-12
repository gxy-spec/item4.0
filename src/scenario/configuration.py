from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator


def _deep_merge(destination: dict[str, Any], source: dict[str, Any]) -> dict[str, Any]:
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            _deep_merge(destination[key], value)
        else:
            destination[key] = copy.deepcopy(value)
    return destination


def read_yaml(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return payload


def resolve_experiment(path: Path) -> dict[str, Any]:
    visited: set[Path] = set()
    included: list[Path] = []

    def resolve(current: Path, stack: tuple[Path, ...] = ()) -> dict[str, Any]:
        current = current.resolve()
        if current in stack:
            chain = " -> ".join(str(item) for item in (*stack, current))
            raise ValueError(f"Cyclic experiment include: {chain}")
        base = read_yaml(current)
        includes = base.pop("includes", [])
        resolved: dict[str, Any] = {}
        for item in includes:
            include_path = (current.parent / item).resolve()
            nested = resolve(include_path, (*stack, current))
            _deep_merge(resolved, nested)
            if include_path not in visited:
                included.append(include_path)
                visited.add(include_path)
        _deep_merge(resolved, base)
        return resolved

    source = path.resolve()
    resolved = resolve(source)
    resolved["_source_config"] = str(source)
    resolved["_included_configs"] = [str(item) for item in included]
    return resolved


def validate_schema(config: dict[str, Any], schema_path: Path) -> list[dict[str, str]]:
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    validator = Draft202012Validator(schema)
    errors = []
    for error in sorted(validator.iter_errors(config), key=lambda item: list(item.path)):
        errors.append({
            "path": ".".join(str(part) for part in error.absolute_path) or "$",
            "message": error.message,
        })
    return errors


def validate_online_control_config(config: dict[str, Any]) -> list[dict[str, str]]:
    """Reject unsafe controls and ambiguous stale-target recovery in non-Oracle runs."""
    if config.get("experiment_type") == "oracle_closed_loop_acceptance":
        return []
    dynamic = config.get("dynamic", {})
    safety = dynamic.get("ugv_safety", {})
    errors: list[dict[str, str]] = []
    required = {
        "enabled": True,
        "obstacle_source": "ugv_rgbd",
        "target_stop_enabled": False,
    }
    for key, expected in required.items():
        if safety.get(key) != expected:
            errors.append({
                "path": f"dynamic.ugv_safety.{key}",
                "message": f"non-Oracle online control requires {expected!r}",
            })
    if dynamic.get("uav_ignore_collision") is not False:
        errors.append({
            "path": "dynamic.uav_ignore_collision",
            "message": "non-Oracle UAV flight must not ignore collisions",
        })
    target_communication = dynamic.get("persistent_target_communication", {})
    target_control_policy = target_communication.get("control_policy", "dual")
    if target_communication.get("enabled", False) and target_control_policy != "none":
        expiry_policy = target_communication.get("expiry_policy", "return_to_patrol")
        if expiry_policy not in {"return_to_patrol", "coast_then_patrol", "persistent_target_memory"}:
            errors.append({
                "path": "dynamic.persistent_target_communication.expiry_policy",
                "message": (
                    "expiry_policy must be return_to_patrol, bounded coast_then_patrol, "
                    "or persistent_target_memory (with RGB-D obstacle guarding)"
                ),
            })
        elif expiry_policy == "persistent_target_memory" and not (
            dynamic.get("ugv_safety", {}).get("enabled")
            and dynamic.get("ugv_safety", {}).get("obstacle_source") == "ugv_rgbd"
        ):
            errors.append({
                "path": "dynamic.ugv_safety",
                "message": "persistent_target_memory requires enabled UGV RGB-D obstacle guarding",
            })
        elif expiry_policy == "persistent_target_memory":
            for key, cast, predicate, expectation in (
                ("memory_association_radius_m", float, lambda value: 0.0 < value <= 50.0,
                 "must be in (0, 50] meters"),
                ("challenger_window_s", float, lambda value: 0.0 < value <= 30.0,
                 "must be in (0, 30] seconds"),
                ("challenger_min_updates", int, lambda value: 1 <= value <= 100,
                 "must be an integer in [1, 100]"),
                ("challenger_min_score_margin", float, lambda value: 0.0 <= value <= 1.0,
                 "must be in [0, 1]"),
                ("challenger_old_unseen_s", float, lambda value: 0.0 <= value <= 60.0,
                 "must be in [0, 60] seconds"),
            ):
                try:
                    raw_value = target_communication.get(key)
                    valid = raw_value is not None and predicate(cast(raw_value))
                    if key == "challenger_min_updates":
                        valid = valid and float(raw_value).is_integer()
                except (TypeError, ValueError):
                    valid = False
                if not valid:
                    errors.append({
                        "path": f"dynamic.persistent_target_communication.{key}",
                        "message": expectation,
                    })
        elif expiry_policy == "coast_then_patrol":
            for key, predicate, expectation in (
                ("stale_grace_s", lambda value: 0.0 < value <= 30.0, "must be in (0, 30] seconds"),
                ("stale_speed_fraction", lambda value: 0.0 < value <= 1.0, "must be in (0, 1]"),
                ("stale_max_distance_m", lambda value: 0.0 < value <= 50.0, "must be in (0, 50] meters"),
            ):
                try:
                    valid = predicate(float(target_communication.get(key, 0.0)))
                except (TypeError, ValueError):
                    valid = False
                if not valid:
                    errors.append({
                        "path": f"dynamic.persistent_target_communication.{key}",
                        "message": expectation,
                    })
    return errors


def write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(payload, allow_unicode=True, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )

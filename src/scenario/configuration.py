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
    base = read_yaml(path)
    includes = base.pop("includes", [])
    resolved: dict[str, Any] = {}
    for item in includes:
        include_path = (path.parent / item).resolve()
        _deep_merge(resolved, read_yaml(include_path))
    _deep_merge(resolved, base)
    resolved["_source_config"] = str(path.resolve())
    resolved["_included_configs"] = [str((path.parent / item).resolve()) for item in includes]
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


def write_yaml(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(payload, allow_unicode=True, sort_keys=False, default_flow_style=False),
        encoding="utf-8",
    )


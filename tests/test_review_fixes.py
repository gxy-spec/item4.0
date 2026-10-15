"""Regression checks for batch report ownership and message-route standoff."""

from __future__ import annotations

import json
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import run_multiscene_abc_formal as batch
from src.navigation.ugv_oracle_planner import plan_route_from_message
from src.scenario.configuration import validate_online_control_config


def test_failed_batch_rerun_does_not_reuse_old_pass(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    output = tmp_path / "runs"
    old = output / "old"
    old.mkdir(parents=True)
    (old / "validation_report.json").write_text(json.dumps({
        "status": "PASS", "started_at": (datetime.now(timezone.utc) - timedelta(hours=1)).isoformat(),
        "run_directory": str(old),
    }), encoding="utf-8")
    monkeypatch.setattr(batch, "LOG_ROOT", tmp_path / "logs")
    batch.LOG_ROOT.mkdir()
    monkeypatch.setattr(batch, "stop_simulator", lambda: None)
    monkeypatch.setitem(sys.modules, "run_e7_comparison_batch", SimpleNamespace(restart_simulator=lambda *args: None))
    monkeypatch.setattr(batch.subprocess, "Popen", lambda *args, **kwargs: SimpleNamespace(poll=lambda: 1, wait=lambda: 1))
    (tmp_path / "test.yaml").write_text("experiment_id: TEST_E1_C\n", encoding="utf-8")
    result = batch.launch_run({"case": "E1", "group": "C", "seed": 2101,
                               "horizon_seconds": 1, "config": str(tmp_path / "test.yaml"),
                               "output_root": str(output)})
    assert result["status"] == "MISSING_REPORT"
    assert result["runner_exit_code"] == 1
    assert "run_dir" not in result


def test_batch_accepts_current_pass_report_after_cleanup_exit(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    output = tmp_path / "runs"
    output.mkdir()
    monkeypatch.setattr(batch, "LOG_ROOT", tmp_path / "logs")
    batch.LOG_ROOT.mkdir()
    monkeypatch.setattr(batch, "stop_simulator", lambda: None)
    monkeypatch.setitem(sys.modules, "run_e7_comparison_batch", SimpleNamespace(restart_simulator=lambda *args: None))

    def launch(*args, **kwargs):
        run = output / "current"
        run.mkdir()
        (run / "validation_report.json").write_text(json.dumps({
            "status": "PASS", "started_at": datetime.now(timezone.utc).isoformat(),
            "run_directory": str(run), "experiment_id": "TEST_E1_C", "summary": {}, "checks": [],
        }), encoding="utf-8")
        return SimpleNamespace(poll=lambda: 1, wait=lambda: 1)

    monkeypatch.setattr(batch.subprocess, "Popen", launch)
    (tmp_path / "test.yaml").write_text("experiment_id: TEST_E1_C\n", encoding="utf-8")
    result = batch.launch_run({"case": "E1", "scene": "E1", "group": "C", "policy": "dual",
                               "seed": 2101, "horizon_seconds": 1,
                               "config": str(tmp_path / "test.yaml"), "output_root": str(output)})
    assert result["status"] == "PASS"
    assert result["runner_exit_code"] == 1
    assert Path(result["run_dir"]).name == "current"


def test_message_route_stops_at_configured_distance(monkeypatch: pytest.MonkeyPatch) -> None:
    class Location:
        def __init__(self, x: float, y: float, z: float):
            self.x, self.y, self.z = x, y, z

        def distance(self, other: "Location") -> float:
            return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    class Map:
        def get_waypoint(self, location: Location, **kwargs):
            return SimpleNamespace(transform=SimpleNamespace(location=location))

    class Planner:
        def __init__(self, world_map, resolution):
            pass

        def trace_route(self, start, target):
            return [(SimpleNamespace(transform=SimpleNamespace(location=Location(x, 0, 0))), None)
                    for x in range(0, 101, 2)]

    monkeypatch.setitem(sys.modules, "carla", SimpleNamespace(
        Location=Location, LaneType=SimpleNamespace(Driving=1)
    ))
    monkeypatch.setitem(sys.modules, "agents", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "agents.navigation", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "agents.navigation.global_route_planner",
                        SimpleNamespace(GlobalRoutePlanner=Planner))
    route_6 = plan_route_from_message(Map(), Location(0, 0, 0), [100, 0, 0], standoff_m=6)
    route_20 = plan_route_from_message(Map(), Location(0, 0, 0), [100, 0, 0], standoff_m=20)
    assert route_6[-1][0] == pytest.approx(94)
    assert route_20[-1][0] == pytest.approx(80)


def test_cross_device_consensus_config_requires_dual_policy() -> None:
    config = {
        "experiment_type": "dual_device_persistent_communication",
        "dynamic": {
            "ugv_safety": {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
            "uav_ignore_collision": False,
            "persistent_target_communication": {
                "enabled": True, "control_policy": "ugv_local",
                "require_cross_device_consensus": True,
            },
        },
    }
    assert any(item["path"].endswith("require_cross_device_consensus")
               for item in validate_online_control_config(config))

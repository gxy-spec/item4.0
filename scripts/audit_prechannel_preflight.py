#!/usr/bin/env python3
"""Generate reproducible code-level evidence before E7-R simulator smoke runs."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.run_stage1_dynamic import apply_safe_route_control  # noqa: E402
from src.communication.persistent_target import PersistentTargetSelector  # noqa: E402
from src.safety.rgbd_obstacle_guard import ObstacleGuardConfig, RGBDObstacleGuard  # noqa: E402
from src.scenario.configuration import (  # noqa: E402
    resolve_experiment,
    validate_online_control_config,
    validate_schema,
)


def dump(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def candidate(x: float = 0.0) -> dict:
    return {
        "candidate_id": "audit-candidate",
        "color": "red",
        "color_score": 0.9,
        "detector_score": 0.8,
        "depth_m": 15.0,
        "depth_reliability": 0.8,
        "world_position_xyz": [x, 1.0, 0.5],
        "temporal_confirmed": True,
    }


def safety_evidence() -> dict:
    cfg = ObstacleGuardConfig(
        pixel_stride=1,
        minimum_occupied_points=1,
        minimum_height_m=-100.0,
        maximum_height_m=100.0,
        corridor_half_width_m=100.0,
        recovery_clear_frames=2,
    )
    guard = RGBDObstacleGuard(cfg)
    pose = np.eye(4)
    nan_depth = np.full((16, 16), np.nan, dtype=np.float32)
    sparse_depth = nan_depth.copy()
    sparse_depth[0, 0] = 3.0
    near = np.full((16, 16), 3.0, dtype=np.float32)
    far = np.full((16, 16), 30.0, dtype=np.float32)
    cases = []
    for name, depth, frame, expected in [
        ("all_nan", nan_depth, 1, "SENSOR_STALE"),
        ("single_valid_pixel", sparse_depth, 2, "SENSOR_STALE"),
        ("near_obstacle", near, 3, "STOP"),
        ("recovery_first_clear", far, 4, "STOP"),
        ("recovery_second_clear", far, 5, "CLEAR"),
    ]:
        actual = guard.observe(depth, frame, frame * 0.1, frame * 0.1, pose)
        cases.append({
            "case": name,
            "expected_state": expected,
            "observed_state": actual["state"],
            "reason": actual["reason"],
            "pass": actual["state"] == expected,
        })
    for name, depth, expected in [
        ("all_nan_pass_lane", nan_depth, False),
        ("sparse_pass_lane", sparse_depth, False),
        ("far_beyond_pass_lane_range", far, False),
    ]:
        actual = guard.lane_occupancy(depth, pose, 3.5)
        cases.append({
            "case": name,
            "expected_clear": expected,
            "observed_clear": actual["clear"],
            "pass": actual["clear"] is expected,
        })
    lane_guard = RGBDObstacleGuard(ObstacleGuardConfig(pixel_stride=1))
    visible_lane = np.full((16, 16), 10.0, dtype=np.float32)
    lane_pose = np.eye(4)
    lane_pose[2, 3] = -20.0
    actual = lane_guard.lane_occupancy(visible_lane, lane_pose, 3.5)
    cases.append({"case": "visible_unoccupied_pass_lane", "expected_clear": True,
                  "observed_clear": actual["clear"], "pass": actual["clear"] is True})
    return {"source": "synthetic metric-depth frames; no CARLA run", "cases": cases,
            "all_pass": all(row["pass"] for row in cases)}


def message_evidence() -> dict:
    cases = []
    selector = PersistentTargetSelector(delay_s=0.0, ttl_s=1.0)
    selector.send("uav", 1, 2.0, candidate())
    selector.advance(2.0)
    fresh_before = selector.select(2.999) is not None
    expired_at = selector.select(3.0) is None
    cases.append({"case": "per_message_ttl_boundary", "fresh_at_0_999_s": fresh_before,
                  "expired_at_1_000_s": expired_at, "pass": fresh_before and expired_at})

    selector = PersistentTargetSelector(delay_s=1.0, ttl_s=1.0)
    selector.send("uav", 1, 2.0, candidate())
    delivered = selector.advance(3.0)
    rejected = not delivered and any(row["event"] == "rejected_expired" for row in selector.events)
    cases.append({"case": "arrival_at_ttl_boundary", "delivered_count": len(delivered),
                  "pass": rejected})

    selector = PersistentTargetSelector(delay_s=0.0, ttl_s=1.0, require_cross_device_consensus=True)
    selector.send("uav", 1, 0.0, candidate())
    selector.advance(0.0)
    selector.send("ugv", 2, 0.9, candidate(0.2))
    selector.advance(0.9)
    selected = selector.select(1.0)
    correct = selected is None and set(selector.latest_by_source) == {"ugv"}
    cases.append({"case": "expired_uav_cannot_corroborate_fresh_ugv",
                  "selected_message_id": None if selected is None else selected["message_id"],
                  "retained_sources": sorted(selector.latest_by_source), "pass": correct})
    return {"source": "real PersistentTargetSelector with synthetic messages", "cases": cases,
            "all_pass": all(row["pass"] for row in cases)}


def truth_boundary_evidence() -> dict:
    class ForbiddenActor:
        reads = 0

        def get_location(self):
            self.reads += 1
            raise AssertionError("online control attempted an actor-truth read")

    class Vehicle:
        control = None

        def get_location(self):
            return SimpleNamespace(x=0.0, y=0.0, z=0.0)

        def apply_control(self, control):
            self.control = control

    original_carla = sys.modules.get("carla")
    sys.modules["carla"] = SimpleNamespace(VehicleControl=lambda **kw: SimpleNamespace(**kw))
    actor = ForbiddenActor()
    vehicle = Vehicle()
    try:
        decision = apply_safe_route_control(
            vehicle,
            [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            2.0,
            actor,
            [actor],
            {"enabled": True, "obstacle_source": "ugv_rgbd", "target_stop_enabled": False},
            sensor_observation=None,
        )
    finally:
        if original_carla is None:
            sys.modules.pop("carla", None)
        else:
            sys.modules["carla"] = original_carla
    passed = actor.reads == 0 and vehicle.control.brake == 1.0 and vehicle.control.throttle == 0.0
    return {
        "scope": "mocked non-Oracle route-control call, not a simulator-wide runtime trace",
        "forbidden_actor_truth_reads": actor.reads,
        "ugv_safety_mode": decision["ugv_safety_mode"],
        "brake": vehicle.control.brake,
        "throttle": vehicle.control.throttle,
        "pass": passed,
    }


def config_evidence() -> dict:
    kinds = {
        "oracle_closed_loop_acceptance": "oracle_experiment_schema.json",
        "perception_closed_loop_acceptance": "perception_experiment_schema.json",
    }
    excluded = {None, "scene_validation", "uav_target_candidate_baseline"}
    rows = []
    for path in sorted((PROJECT_ROOT / "configs" / "experiments").glob("*.yaml")):
        resolved = resolve_experiment(path)
        kind = resolved.get("experiment_type")
        if kind in excluded:
            continue
        schema = PROJECT_ROOT / "configs" / "schemas" / kinds.get(kind, "dynamic_experiment_schema.json")
        schema_errors = validate_schema(resolved, schema)
        control_errors = validate_online_control_config(resolved)
        rows.append({
            "config": path.name,
            "experiment_type": kind,
            "oracle_exception": kind == "oracle_closed_loop_acceptance",
            "ugv_obstacle_source": resolved.get("dynamic", {}).get("ugv_safety", {}).get("obstacle_source"),
            "target_stop_enabled": resolved.get("dynamic", {}).get("ugv_safety", {}).get("target_stop_enabled"),
            "uav_ignore_collision": resolved.get("dynamic", {}).get("uav_ignore_collision"),
            "schema_errors": schema_errors,
            "control_errors": control_errors,
            "pass": not schema_errors and not control_errors,
        })
    return {"checked": len(rows), "failed": sum(not row["pass"] for row in rows),
            "all_pass": all(row["pass"] for row in rows), "configs": rows,
            "note": "UAV collision flag check is static; route clearance needs a CARLA-Air smoke run."}


def junit_counts(path: Path) -> dict[str, int]:
    root = ElementTree.parse(path).getroot()
    suites = [root] if root.tag == "testsuite" else list(root.iter("testsuite"))
    return {key: sum(int(suite.attrib.get(key, 0)) for suite in suites)
            for key in ("tests", "failures", "errors", "skipped")}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--junit", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=True)
    safety = safety_evidence()
    messages = message_evidence()
    truth = truth_boundary_evidence()
    configs = config_evidence()
    tests = junit_counts(args.junit)
    dump(output / "safety_cases.json", safety)
    dump(output / "message_ttl_cases.json", messages)
    dump(output / "truth_boundary_audit.json", truth)
    dump(output / "config_audit.json", configs)
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True).strip()
    status = subprocess.check_output(["git", "status", "--short"], cwd=PROJECT_ROOT, text=True).strip()
    all_pass = (
        tests["tests"] > 0 and tests["failures"] == tests["errors"] == tests["skipped"] == 0
        and safety["all_pass"] and messages["all_pass"] and truth["pass"] and configs["all_pass"]
    )
    summary = {
        "status": "PASS_CODE_PREFLIGHT" if all_pass else "FAIL_CODE_PREFLIGHT",
        "scope": "automated preflight only; no simulator collision or trajectory claim",
        "git_head": head,
        "uncommitted_changes": status.splitlines(),
        "pytest": tests,
        "safety_cases": len(safety["cases"]),
        "message_cases": len(messages["cases"]),
        "configs_checked": configs["checked"],
        "configs_failed": configs["failed"],
        "mocked_actor_truth_reads": truth["forbidden_actor_truth_reads"],
    }
    dump(output / "preflight_summary.json", summary)
    markdown = [
        "# E7-R 前置信道实验：代码级验收",
        "",
        f"结论：**{summary['status']}**。本报告不代表 CARLA-Air 动态安全验收。",
        "",
        f"- 测试：{tests['tests']} 项；失败 {tests['failures']}，错误 {tests['errors']}，跳过 {tests['skipped']}。",
        f"- 深度安全用例：{len(safety['cases'])} 项，{'全部通过' if safety['all_pass'] else '存在失败'}。",
        f"- 消息时效用例：{len(messages['cases'])} 项，{'全部通过' if messages['all_pass'] else '存在失败'}。",
        f"- 受检配置：{configs['checked']} 份；不合格 {configs['failed']} 份。",
        f"- 模拟非 Oracle 控制调用中的 actor 真值读取：{truth['forbidden_actor_truth_reads']} 次。",
        "",
        "原始证据：`test_results.xml`、`safety_cases.json`、`message_ttl_cases.json`、",
        "`truth_boundary_audit.json`、`config_audit.json`、`preflight_summary.json`。",
        "",
        "限制：尚未检查 UAV 航线实际净空、UGV 在 CARLA-Air 中的停车距离与碰撞，",
        "也未运行同场景 A/B/C；这些属于下一步仿真 smoke。",
        "",
    ]
    (output / "test_summary.md").write_text("\n".join(markdown), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())

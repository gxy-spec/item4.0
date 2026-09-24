#!/usr/bin/env python3
"""Run the controlled 3-seed x 2-target S1 batch, stopping on first failure."""

from __future__ import annotations

import argparse
import copy
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from scenario.configuration import resolve_experiment, write_yaml  # noqa: E402


def now_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def newest_episode(episodes_dir: Path, experiment_id: str) -> Path | None:
    matches = list(episodes_dir.glob(f"{experiment_id}_*"))
    return max(matches, key=lambda path: path.stat().st_mtime) if matches else None


def write_progress(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def simulator_command(launcher: Path, package_root: Path) -> list[str]:
    return [
        "powershell.exe",
        "-NoProfile",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(launcher),
        "Town10HD",
        "--port",
        "2000",
        "--quality",
        "Low",
        "--res",
        "960x540",
        "--no-traffic",
        "--package-root",
        str(package_root / "WindowsNoEditor"),
    ]


def stop_simulator(launcher: Path) -> None:
    subprocess.run(
        [
            "powershell.exe",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(launcher),
            "--kill",
        ],
        check=False,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--design",
        type=Path,
        default=PROJECT_ROOT / "configs" / "experiments" / "s1_multiseed_3x2_batch.yaml",
    )
    parser.add_argument("--manage-simulator", action="store_true")
    parser.add_argument(
        "--launcher",
        type=Path,
        default=Path(r"D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64\CarlaAir.ps1"),
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=Path(r"D:\CarlaAir\CarlaAir-v0.1.7-Windows11-x86_64"),
    )
    parser.add_argument("--only-seed", type=int, default=None)
    parser.add_argument("--only-position", type=str, default=None)
    args = parser.parse_args()
    design_path = args.design.resolve()
    design = yaml.safe_load(design_path.read_text(encoding="utf-8"))
    base_path = (PROJECT_ROOT / design["base_config"]).resolve()
    base = resolve_experiment(base_path)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    batch_dir = Path(design["output_root"]) / f"{design['batch_id']}_{stamp}"
    episodes_dir = batch_dir / "episodes"
    configs_dir = batch_dir / "configs"
    analysis_dir = batch_dir / "analysis"
    for path in (episodes_dir, configs_dir, analysis_dir):
        path.mkdir(parents=True, exist_ok=True)
    (batch_dir / design_path.name).write_text(design_path.read_text(encoding="utf-8"), encoding="utf-8")

    jobs = [
        (int(seed), position)
        for seed in design["seeds"]
        for position in design["target_positions"]
        if (args.only_seed is None or int(seed) == args.only_seed)
        and (
            args.only_position is None
            or str(position["id"]).upper() == args.only_position.upper()
        )
    ]
    if not jobs:
        raise ValueError("Episode filter selected no seed/target-position combinations")
    progress = {
        "batch_id": design["batch_id"],
        "batch_directory": str(batch_dir),
        "started_at": now_utc(),
        "status": "RUNNING",
        "scientific_failure_automatic_retry": False,
        "engineering_startup_recovery": True,
        "stop_on_first_failure": True,
        "planned_episodes": len(jobs),
        "episodes": [],
    }
    progress_path = batch_dir / "batch_progress.json"
    write_progress(progress_path, progress)
    print(f"BATCH_DIRECTORY={batch_dir}", flush=True)

    runner = PROJECT_ROOT / "scripts" / "run_s1_perception_closed_loop.py"
    replay = PROJECT_ROOT / "scripts" / "build_synchronized_multiview_replay.py"
    summary_builder = PROJECT_ROOT / "scripts" / "build_s1_summary.py"
    key_frames = PROJECT_ROOT / "scripts" / "extract_s1_key_event_frames.py"
    aggregate = PROJECT_ROOT / "scripts" / "build_s1_batch_summary.py"

    for index, (seed, position) in enumerate(jobs, start=1):
        position_id = str(position["id"])
        experiment_id = f"S1_PERCEPTION_BATCH_T10_ZA_{position_id}_S{seed}"
        config = copy.deepcopy(base)
        config["experiment_id"] = experiment_id
        config["random_seed"] = seed
        config["region"]["target_transform"] = copy.deepcopy(position["target_transform"])
        config["region"]["target_lane_transform"] = copy.deepcopy(position["target_lane_transform"])
        config["dynamic"]["save_all_samples"] = False
        config["dynamic"]["sensor_save_every_n_frames"] = int(
            design["storage"]["sensor_save_every_n_frames"]
        )
        config["dynamic"]["maximum_initial_uav_pose_error_m"] = float(
            design["execution"].get("maximum_initial_uav_pose_error_m", 1.0)
        )
        config["output"].update(
            {
                "root": str(episodes_dir),
                "save_rgb": bool(design["storage"]["save_rgb"]),
                "save_depth_raw": bool(design["storage"]["save_depth_raw"]),
                "save_depth_float_m": bool(design["storage"]["save_depth_float_m"]),
                "save_depth_colour": bool(design["storage"]["save_depth_colour"]),
            }
        )
        config["acceptance"].update(design.get("acceptance_overrides", {}))
        config["batch"] = {
            "batch_id": design["batch_id"],
            "episode_index": index,
            "target_position_id": position_id,
            "target_position_description": position.get("description", ""),
            "storage_profile": design["storage"]["profile"],
        }
        config_path = configs_dir / f"{experiment_id}.yaml"
        write_yaml(config_path, config)
        episode_entry = {
            "episode_index": index,
            "experiment_id": experiment_id,
            "seed": seed,
            "target_position": position_id,
            "status": "RUNNING",
            "started_at": now_utc(),
            "config": str(config_path),
        }
        progress["episodes"].append(episode_entry)
        write_progress(progress_path, progress)
        print(f"[{index}/{len(jobs)}] START {experiment_id}", flush=True)

        maximum_engineering_attempts = int(
            design["execution"].get("maximum_engineering_attempts", 1)
        )
        cooldown_seconds = float(design["execution"].get("restart_cooldown_seconds", 0.0))
        settle_seconds = float(design["execution"].get("post_start_settle_seconds", 0.0))
        engineering_recoveries: list[dict] = []
        run_dir: Path | None = None
        report: dict = {}
        report_status = "MISSING_REPORT"
        result = subprocess.CompletedProcess([], 1)
        for engineering_attempt in range(1, maximum_engineering_attempts + 1):
            run_dir = None
            report = {}
            report_status = "MISSING_REPORT"
            simulator_ready = True
            if args.manage_simulator:
                stop_simulator(args.launcher)
                time.sleep(cooldown_seconds)
                simulator_start = subprocess.run(
                    simulator_command(args.launcher, args.package_root), cwd=PROJECT_ROOT
                )
                simulator_ready = simulator_start.returncode == 0
                if simulator_ready:
                    time.sleep(settle_seconds)
                else:
                    result = simulator_start
                    report_status = "SIMULATOR_START_ERROR"
            if simulator_ready:
                try:
                    result = subprocess.run(
                        [
                            sys.executable,
                            "-u",
                            str(runner),
                            "--config",
                            str(config_path),
                            "--output-root",
                            str(episodes_dir),
                        ],
                        cwd=PROJECT_ROOT,
                    )
                finally:
                    if args.manage_simulator:
                        stop_simulator(args.launcher)
                run_dir = newest_episode(episodes_dir, experiment_id)
                if run_dir is not None and (run_dir / "validation_report.json").exists():
                    report = json.loads(
                        (run_dir / "validation_report.json").read_text(encoding="utf-8")
                    )
                    report_status = str(report.get("status", "UNKNOWN"))

            # Only infrastructure/runtime ERRORs are eligible for automatic
            # recovery.  A completed scientific FAIL always leaves the loop.
            retryable_engineering_error = report_status in {
                "ERROR",
                "MISSING_REPORT",
                "SIMULATOR_START_ERROR",
            }
            if not retryable_engineering_error or engineering_attempt >= maximum_engineering_attempts:
                break
            diagnostic_dir = batch_dir / "engineering_diagnostics"
            diagnostic_dir.mkdir(parents=True, exist_ok=True)
            archived_run = None
            if run_dir is not None and run_dir.exists():
                archived_run = diagnostic_dir / f"attempt_{engineering_attempt}_{run_dir.name}"
                shutil.move(str(run_dir), str(archived_run))
            recovery = {
                "attempt": engineering_attempt,
                "status": report_status,
                "error": report.get("error"),
                "archived_run": None if archived_run is None else str(archived_run),
                "recovered_at": now_utc(),
            }
            engineering_recoveries.append(recovery)
            episode_entry["engineering_recoveries"] = engineering_recoveries
            write_progress(progress_path, progress)
            print(
                f"ENGINEERING_RECOVERY {experiment_id}: attempt {engineering_attempt}/"
                f"{maximum_engineering_attempts} ended with {report_status}; restarting cleanly.",
                flush=True,
            )
            time.sleep(cooldown_seconds)

        episode_entry["exit_code"] = result.returncode
        episode_entry["run_directory"] = None if run_dir is None else str(run_dir)
        episode_entry["completed_at"] = now_utc()
        if report.get("error"):
            episode_entry["error"] = report["error"]
        episode_entry["status"] = report_status
        write_progress(progress_path, progress)

        if result.returncode != 0 or report_status != "PASS" or run_dir is None:
            progress["status"] = "STOPPED_ON_FAILURE"
            progress["failed_episode"] = experiment_id
            progress["completed_at"] = now_utc()
            write_progress(progress_path, progress)
            subprocess.run([sys.executable, str(aggregate), "--batch-dir", str(batch_dir)], cwd=PROJECT_ROOT)
            print(
                f"BATCH_STOPPED_ON_FAILURE={experiment_id}; no automatic retry was attempted.",
                flush=True,
            )
            return result.returncode if result.returncode else 2

        post_steps = (
            [sys.executable, str(replay), "--run-dir", str(run_dir), "--fps", str(design["execution"]["replay_fps"])],
            [sys.executable, str(summary_builder), "--run-dir", str(run_dir)],
            [sys.executable, str(key_frames), "--run-dir", str(run_dir)],
        )
        for command in post_steps:
            post = subprocess.run(command, cwd=PROJECT_ROOT)
            if post.returncode != 0:
                episode_entry["status"] = "POSTPROCESS_ERROR"
                episode_entry["postprocess_exit_code"] = post.returncode
                progress["status"] = "STOPPED_ON_FAILURE"
                progress["failed_episode"] = experiment_id
                progress["completed_at"] = now_utc()
                write_progress(progress_path, progress)
                subprocess.run([sys.executable, str(aggregate), "--batch-dir", str(batch_dir)], cwd=PROJECT_ROOT)
                print(
                    f"BATCH_STOPPED_ON_FAILURE={experiment_id}; post-processing failed; no retry attempted.",
                    flush=True,
                )
                return post.returncode
        print(f"[{index}/{len(jobs)}] PASS {experiment_id}", flush=True)

    aggregate_result = subprocess.run(
        [sys.executable, str(aggregate), "--batch-dir", str(batch_dir)], cwd=PROJECT_ROOT
    )
    progress["status"] = "PASS" if aggregate_result.returncode == 0 else "POSTPROCESS_ERROR"
    progress["completed_at"] = now_utc()
    write_progress(progress_path, progress)
    if aggregate_result.returncode == 0:
        (Path(design["output_root"]) / "LATEST.txt").write_text(
            str(batch_dir), encoding="utf-8"
        )
    print(f"BATCH_RESULT={progress['status']}; output={batch_dir}", flush=True)
    return aggregate_result.returncode


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Independent analytic checks for the A2G propagation and transport chain."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from communication.a2g_spatial import (  # noqa: E402
    A2GSpatialConfig,
    C_MPS,
    K_BOLTZMANN,
    T0_K,
    CausalA2GSpatialSampler,
    evaluate_track,
    geometry,
)
from communication.a2g_transport import A2GTransportConfig, A2GTransportQueue  # noqa: E402
from communication.persistent_target import PersistentTargetSelector  # noqa: E402
from verify_a2g_spatial_run import verify as verify_saved_spatial_run  # noqa: E402


def observation(distance_m: float, *, los: int = 1, timestamp_s: float = 0.0) -> dict[str, Any]:
    height = min(30.0, distance_m * 0.5)
    horizontal = math.sqrt(distance_m * distance_m - height * height)
    return {
        "frame": round(timestamp_s * 10), "timestamp_s": timestamp_s,
        "uav_antenna_xyz_m": (horizontal, 0.0, height),
        "ugv_antenna_xyz_m": (0.0, 0.0, 0.0),
        "horizontal_m": horizontal, "height_m": height,
        "distance_m": distance_m,
        "elevation_deg": math.degrees(math.asin(height / distance_m)),
        "geo_los": int(los), "first_static_blocker": None if los else "Buildings",
        "ray_hit_count": 0 if los else 1,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--spatial-run", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    spatial_mapping = yaml.safe_load(
        (ROOT / "configs/communication/a2g_spatial_v1.yaml").read_text(encoding="utf-8")
    )["spatial_link"]
    transport_mapping = yaml.safe_load(
        (ROOT / "configs/communication/a2g_transport_v1.yaml").read_text(encoding="utf-8")
    )["transport_link"]
    spatial = A2GSpatialConfig.from_mapping(spatial_mapping, run_seed=1001)
    deterministic_spatial = replace(
        spatial, shadow_sigma_los_db=0.0, shadow_sigma_nlos_db=0.0
    )
    transport = A2GTransportConfig.from_mapping(
        transport_mapping, bandwidth_hz=spatial.bandwidth_hz
    )
    checks: list[dict[str, Any]] = []

    def numeric(name: str, theory: float, program: float, tolerance: float,
                unit: str, meaning: str) -> None:
        error = abs(program - theory)
        checks.append({"name": name, "passed": error <= tolerance,
                       "theory": theory, "program": program,
                       "absolute_error": error, "tolerance": tolerance,
                       "unit": unit, "meaning": meaning})

    def logical(name: str, passed: bool, observed: Any, expected: str,
                meaning: str) -> None:
        checks.append({"name": name, "passed": bool(passed),
                       "observed": observed, "expected": expected,
                       "meaning": meaning})

    horizontal, height, distance, elevation = geometry((3, 4, 12), (0, 0, 0))
    numeric("geometry_horizontal", 5.0, horizontal, 1e-12, "m", "3-4-5 horizontal geometry")
    numeric("geometry_distance", 13.0, distance, 1e-12, "m", "5-12-13 three-dimensional distance")
    numeric("geometry_elevation", math.degrees(math.asin(12 / 13)), elevation,
            1e-12, "degree", "elevation uses height divided by 3D distance")

    spatial_rows = evaluate_track(
        [observation(100.0, los=1), observation(200.0, los=1, timestamp_s=0.1),
         observation(100.0, los=0, timestamp_s=0.2)],
        deterministic_spatial,
    )
    fspl_100 = 20 * math.log10(4 * math.pi * 100.0 * spatial.carrier_hz / C_MPS)
    fspl_200 = 20 * math.log10(4 * math.pi * 200.0 * spatial.carrier_hz / C_MPS)
    numeric("fspl_at_100m", fspl_100, spatial_rows[0]["fspl_db"], 1e-10, "dB",
            "closed-form free-space path loss")
    numeric("doubling_distance_loss", 20 * math.log10(2.0),
            spatial_rows[1]["fspl_db"] - spatial_rows[0]["fspl_db"],
            1e-10, "dB", "doubling distance adds 6.02 dB loss")
    extra_difference = spatial.eta_nlos_db - spatial.eta_los_db
    numeric("nlos_extra_loss", extra_difference,
            spatial_rows[2]["geo_path_loss_db"] - spatial_rows[0]["geo_path_loss_db"],
            1e-10, "dB", "same position isolates the LoS/NLoS excess-loss difference")
    numeric("nlos_snr_drop", -extra_difference,
            spatial_rows[2]["geo_snr_db"] - spatial_rows[0]["geo_snr_db"],
            1e-10, "dB", "extra path loss produces an equal SNR reduction")
    noise_theory = 10 * math.log10(
        K_BOLTZMANN * T0_K * spatial.bandwidth_hz
        * 10 ** (spatial.noise_figure_db / 10) / 1e-3
    )
    numeric("thermal_noise", noise_theory, spatial_rows[0]["noise_dbm"],
            1e-10, "dBm", "kTB noise plus receiver noise figure")

    cap10 = replace(transport, rate_cap_bit_s=10_000_000.0)
    gap = 10 ** (cap10.snr_gap_db / 10)
    rate_at_6_theory = cap10.resource_fraction * cap10.bandwidth_hz * math.log2(
        1 + 10 ** (6 / 10) / gap
    )
    numeric("rate_at_6db", rate_at_6_theory, cap10.rate_bit_s(6.0),
            1e-6, "bit/s", "rate formula before the business cap")
    logical("below_5db_stops_service", cap10.rate_bit_s(4.999) == 0.0,
            cap10.rate_bit_s(4.999), "0 bit/s", "working threshold is strict below 5 dB")
    needed_spectral = cap10.rate_cap_bit_s / (cap10.resource_fraction * cap10.bandwidth_hz)
    cap_snr_db = 10 * math.log10(gap * (2 ** needed_spectral - 1))
    numeric("snr_for_10mbps_cap", 10_000_000.0, cap10.rate_bit_s(cap_snr_db),
            1e-5, "bit/s", "the analytically derived SNR reaches the 10 Mb/s cap")
    physical_max = (transport.resource_fraction * transport.bandwidth_hz
                    * transport.spectral_efficiency_cap_bit_s_hz)
    numeric("physical_rate_ceiling", physical_max,
            replace(transport, rate_cap_bit_s=100_000_000.0).rate_bit_s(100.0),
            1e-5, "bit/s", "spectral-efficiency ceiling limits the reference case")

    semantic_config = replace(transport, rate_cap_bit_s=500_000.0)
    semantic_queue = A2GTransportQueue(semantic_config, "analytic")
    semantic_queue.update_channel(0.0, snr_db=30.0, distance_m=120.0)
    semantic_queue.enqueue({"message_id": "semantic", "source_timestamp_s": 0.0},
                           enqueue_timestamp_s=0.0, payload_bytes_override=704)
    semantic_queue.advance_to(0.1)
    semantic = semantic_queue.records[0]
    numeric("semantic_service_at_500kbps", 704 * 8 / 500_000,
            float(semantic.t_finish), 1e-10, "s",
            "704-byte payload time equals bits divided by rate")
    numeric("propagation_delay_at_120m", 120 / transport.propagation_speed_m_s,
            float(semantic.t_arr - semantic.t_finish), 1e-10, "s",
            "arrival adds distance divided by propagation speed")

    slow = replace(transport, rate_cap_bit_s=800.0, ttl_s=2.0)
    pause_queue = A2GTransportQueue(slow, "analytic")
    pause_queue.update_channel(0.0, snr_db=30.0, distance_m=10.0)
    pause_queue.enqueue({"message_id": "pause", "source_timestamp_s": 0.0},
                        enqueue_timestamp_s=0.0, payload_bytes_override=80)
    pause_queue.update_channel(0.25, snr_db=0.0, distance_m=10.0)
    pause_queue.update_channel(0.75, snr_db=30.0, distance_m=10.0)
    pause_queue.advance_to(1.4)
    numeric("pause_resume_finish_time", 1.30,
            float(pause_queue.records[0].t_finish), 1e-10, "s",
            "200 bits sent, 0.5 s pause, then remaining 440 bits resume")
    logical("pause_resume_events",
            [item["event"] for item in pause_queue.events].count("service_paused") == 1
            and [item["event"] for item in pause_queue.events].count("service_resumed") == 1,
            [item["event"] for item in pause_queue.events],
            "one pause and one resume", "service retains progress across a zero-rate interval")

    boundary_queue = A2GTransportQueue(replace(slow, ttl_s=1.0), "analytic")
    boundary_queue.update_channel(0.0, snr_db=30.0, distance_m=10.0)
    boundary_queue.enqueue({"message_id": "boundary", "source_timestamp_s": 0.0},
                           enqueue_timestamp_s=0.0, payload_bytes_override=100)
    boundary_queue.advance_to(1.0)
    logical("ttl_equality_is_invalid", boundary_queue.records[0].status == "expired_in_service",
            boundary_queue.records[0].status, "expired_in_service",
            "information must arrive strictly before the one-second TTL")

    selector = PersistentTargetSelector(delay_s=0.05, ttl_s=1.0)
    candidate = {"color": "red", "detector_score": 0.8, "color_score": 0.9,
                 "depth_m": 10.0, "depth_reliability": 0.8,
                 "world_position_xyz": [1.0, 2.0, 0.0], "temporal_confirmed": True}
    external = selector.send("uav", 1, 0.0, candidate, external_delivery=True)
    received = selector.receive_external({**external, "received_at_s": 0.2})
    logical("external_delivery_uses_radio_arrival",
            not selector.pending and received is not None
            and received["receive_timestamp_s"] == 0.2,
            None if received is None else received["receive_timestamp_s"],
            "0.2 s with no added legacy 0.05 s", "old fixed delay is bypassed")

    prefix = [observation(100.0), observation(110.0, timestamp_s=0.1)]
    sampler_a = CausalA2GSpatialSampler(spatial)
    sampler_b = CausalA2GSpatialSampler(spatial)
    prefix_a = [sampler_a.sample(row) for row in prefix]
    prefix_b = [sampler_b.sample(row) for row in [*prefix, observation(120.0, timestamp_s=0.2)]]
    repeated = sampler_a.sample({**prefix[0], "timestamp_s": 0.3})
    logical("causal_sampling_is_repeatable",
            prefix_a == prefix_b[:2]
            and repeated["geo_shadow_db"] == prefix_a[0]["geo_shadow_db"],
            {"future_independent_prefix": prefix_a == prefix_b[:2],
             "repeated_position_reused": repeated["geo_shadow_db"] == prefix_a[0]["geo_shadow_db"]},
            "same prefix and repeated position give the same values",
            "online channel does not read future positions")

    saved = verify_saved_spatial_run(args.spatial_run.resolve())
    logical("saved_trace_exact_replay", saved["status"] == "PASS"
            and saved["maximum_numeric_replay_difference"] < 1e-8,
            saved, "PASS with maximum difference below 1e-8",
            "saved 200-sample spatial result is exactly reproducible")

    result = {
        "status": "PASS" if all(item["passed"] for item in checks) else "FAIL",
        "scope": "analytic and deterministic verification without CARLA task control",
        "passed": sum(item["passed"] for item in checks),
        "total": len(checks),
        "checks": checks,
    }
    (args.output / "independent_validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                         "font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False})
    distances = np.asarray([50.0, 75.0, 100.0, 150.0, 200.0, 300.0, 400.0])
    theory_fspl = 20 * np.log10(4 * math.pi * distances * spatial.carrier_hz / C_MPS)
    program_fspl = [evaluate_track([observation(float(value))], deterministic_spatial)[0]["fspl_db"]
                    for value in distances]
    snr_axis = np.linspace(-5, 30, 351)
    theory_rate = [0.0 if snr < cap10.snr_working_threshold_db else min(
        cap10.rate_cap_bit_s,
        cap10.resource_fraction * cap10.bandwidth_hz * min(
            math.log2(1 + 10 ** (snr / 10) / gap), cap10.spectral_efficiency_cap_bit_s_hz
        )) for snr in snr_axis]
    program_rate = [cap10.rate_bit_s(float(snr)) for snr in snr_axis]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    axes[0].plot(distances, theory_fspl, color="#225B91", linewidth=1.8, label="理论值")
    axes[0].scatter(distances, program_fspl, facecolors="none", edgecolors="#D0822C",
                    linewidth=1.4, s=42, label="程序值")
    axes[0].set(xlabel="三维距离（m）", ylabel="自由空间路径损耗（dB）",
                title="距离增大，路径损耗按理论上升")
    axes[0].legend(frameon=False)
    axes[1].plot(snr_axis, np.asarray(theory_rate) / 1e6, color="#225B91",
                 linewidth=1.8, label="理论值")
    axes[1].plot(snr_axis, np.asarray(program_rate) / 1e6, color="#D0822C",
                 linestyle="--", linewidth=1.4, label="程序值")
    axes[1].axvline(5.0, color="#B33A3A", linewidth=1.0, label="5 dB停传门限")
    axes[1].axvline(cap_snr_db, color="#555555", linestyle=":", linewidth=1.0,
                    label="约7.8 dB达到10 Mb/s")
    axes[1].set(xlabel="SNR（dB）", ylabel="可用传输速率（Mb/s）",
                title="SNR升高，速率增加并达到业务上限")
    axes[1].legend(frameon=False, fontsize=8)
    for axis in axes:
        axis.grid(alpha=0.2)
    fig.suptitle(f"空地信道独立验证：理论值与程序值一致（{result['passed']}/{result['total']}项通过）")
    fig.tight_layout()
    for extension in ("png", "pdf"):
        fig.savefig(args.output / f"independent_validation.{extension}", dpi=300,
                    bbox_inches="tight")
    plt.close(fig)

    print(json.dumps({"status": result["status"], "passed": result["passed"],
                      "total": result["total"], "output": str(args.output)},
                     ensure_ascii=False, indent=2))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())

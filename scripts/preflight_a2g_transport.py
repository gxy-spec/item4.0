"""Read-only rate and byte-budget audit before implementing A2G transport.

This script neither simulates a queue nor changes CARLA control. It evaluates
the frozen spatial trace at declared service parameters and measures the
sender payload already emitted by the E7 semantic-message path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_spatial_trace(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("Spatial trace has no samples")
    result = []
    previous = -math.inf
    for row in rows:
        sample = {key: float(row[key]) for key in
                  ("timestamp_s", "geo_snr_db", "prob_snr_db")}
        if not all(math.isfinite(value) for value in sample.values()):
            raise ValueError("Spatial trace contains nonfinite time or SNR")
        if sample["timestamp_s"] <= previous:
            raise ValueError("Spatial trace timestamps must increase")
        previous = sample["timestamp_s"]
        result.append(sample)
    return result


def _sender_bytes(path: Path, fields: list[str]) -> list[int]:
    sizes: list[int] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            message = json.loads(line)
            if message.get("event") != "sent" or message.get("source") != "uav":
                continue
            missing = [field for field in fields if field not in message]
            if missing:
                raise ValueError(f"E7 UAV message lacks sender fields: {missing}")
            payload = {key: message[key] for key in fields}
            wire = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                              ensure_ascii=False, allow_nan=False).encode("utf-8")
            sizes.append(len(wire))
    if not sizes:
        raise ValueError("No E7 UAV sender messages found")
    return sizes


def effective_rate(snr_db: float, *, threshold_db: float, gap_db: float,
                   resource_fraction: float, bandwidth_hz: float,
                   efficiency_cap: float, rate_cap: float) -> tuple[float, bool]:
    if snr_db < threshold_db:
        return 0.0, False
    gamma_linear = 10.0 ** (snr_db / 10.0)
    gamma_gap_linear = 10.0 ** (gap_db / 10.0)
    uncapped = resource_fraction * bandwidth_hz * min(
        math.log2(1.0 + gamma_linear / gamma_gap_linear), efficiency_cap
    )
    return min(rate_cap, uncapped), rate_cap <= uncapped


def _scenario(rows: list[dict[str, float]], *, group: str, attenuation_db: float,
              cap_bit_s: float, service: dict[str, Any], bandwidth_hz: float,
              payloads: dict[str, int], ttl_s: float,
              processing_s: float) -> dict[str, Any]:
    values = [effective_rate(
        row[f"{group}_snr_db"] - attenuation_db,
        threshold_db=float(service["snr_working_threshold_db"]),
        gap_db=float(service["snr_gap_db"]),
        resource_fraction=float(service["resource_fraction"]),
        bandwidth_hz=bandwidth_hz,
        efficiency_cap=float(service["spectral_efficiency_cap_bit_s_hz"]),
        rate_cap=cap_bit_s,
    ) for row in rows]
    rates = [entry[0] for entry in values]
    nonzero = [rate for rate in rates if rate > 0.0]
    return {
        "group": group,
        "extra_attenuation_db": attenuation_db,
        "rate_cap_bit_s": cap_bit_s,
        "rate_min_mbit_s": min(rates) / 1e6,
        "rate_median_mbit_s": statistics.median(rates) / 1e6,
        "rate_max_mbit_s": max(rates) / 1e6,
        "zero_service_sample_fraction": (len(rates) - len(nonzero)) / len(rates),
        "business_cap_binding_sample_fraction": sum(binding for _, binding in values) / len(rates),
        "one_shot_feasible_sample_fraction_excludes_queue": {
            name: sum(rate > 0 and (8 * size / rate + processing_s) < ttl_s
                      for rate in rates) / len(rates)
            for name, size in payloads.items()
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--messages", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/communication/a2g_transport_v1.yaml")
    parser.add_argument("--spatial-config", type=Path,
                        default=ROOT / "configs/communication/a2g_spatial_v1.yaml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))["transport_link"]
    spatial = yaml.safe_load(args.spatial_config.read_text(encoding="utf-8"))["spatial_link"]
    service = cfg["service"]
    bandwidth_hz = float(spatial["bandwidth_hz"])
    if bandwidth_hz <= 0 or not 0 < float(service["resource_fraction"]) <= 1:
        raise ValueError("Bandwidth and resource fraction are invalid")
    if float(service["snr_gap_db"]) < 0 or float(service["spectral_efficiency_cap_bit_s_hz"]) <= 0:
        raise ValueError("SNR gap or spectral-efficiency ceiling is invalid")
    caps = [float(value) for value in service["throttle_cases_bit_s"]]
    if any(value <= 0 or not math.isfinite(value) for value in caps):
        raise ValueError("Rate caps must be finite and positive")
    if float(service["rate_cap_bit_s"]) != caps[0]:
        raise ValueError("First diagnostic cap must be the unthrottled reference")

    rows = _load_spatial_trace(args.trace)
    deltas = [right["timestamp_s"] - left["timestamp_s"]
              for left, right in zip(rows, rows[1:])]
    sample_period = float(cfg["clock"]["rate_sample_period_s"])
    if any(abs(delta - sample_period) > 1e-4 for delta in deltas):
        raise ValueError("Spatial trace interval differs from configured service sample period")
    fields = list(cfg["wire"]["sender_fields"])
    sizes = _sender_bytes(args.messages, fields)
    header_bytes = int(cfg["wire"]["explicit_header_bytes"])
    if header_bytes < 0:
        raise ValueError("Explicit header bytes cannot be negative")
    payloads = {
        "e7_semantic_max_observed": max(sizes) + header_bytes,
        "feature_placeholder": int(cfg["probes"]["feature_bytes_placeholder"]) + header_bytes,
        "raw_rgb_placeholder": int(cfg["probes"]["raw_rgb_bytes_placeholder"]) + header_bytes,
    }
    ttl_s = float(cfg["queue"]["ttl_s"])
    processing_s = float(cfg["receiver"]["semantic_processing_s"])
    if ttl_s <= 0 or processing_s < 0:
        raise ValueError("TTL or processing time is invalid")
    scenarios = [
        _scenario(rows, group=group, attenuation_db=float(attenuation),
                  cap_bit_s=cap, service=service, bandwidth_hz=bandwidth_hz,
                  payloads=payloads, ttl_s=ttl_s, processing_s=processing_s)
        for group in ("geo", "prob")
        for attenuation in service["sensitivity"]["extra_attenuation_db"]
        for cap in caps
    ]
    output = {
        "report_type": "preflight_math_only_no_queue_no_online_delivery",
        "source_files": {
            "trace": str(args.trace), "trace_sha256": _sha256(args.trace),
            "message_log": str(args.messages), "message_log_sha256": _sha256(args.messages),
            "transport_config": str(args.config), "transport_config_sha256": _sha256(args.config),
            "spatial_config": str(args.spatial_config), "spatial_config_sha256": _sha256(args.spatial_config),
        },
        "spatial_sample_count": len(rows),
        "sample_period_s": sample_period,
        "e7_uav_semantic_message_count": len(sizes),
        "e7_uav_semantic_sender_bytes": {
            "min": min(sizes), "median": statistics.median(sizes), "max": max(sizes),
        },
        "probe_wire_bytes": payloads,
        "note": "Feasibility uses one frozen instantaneous rate and excludes queue, source processing, and future SNR changes.",
        "scenarios": scenarios,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {args.output}: {len(rows)} spatial samples, {len(sizes)} E7 UAV messages, {len(scenarios)} scenarios")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

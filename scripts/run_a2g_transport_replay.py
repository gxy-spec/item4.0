#!/usr/bin/env python3
"""Run paired, fixed-trajectory A2G transport experiments without CARLA control."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from communication.a2g_transport import A2GTransportConfig, A2GTransportQueue  # noqa: E402


def load_trace(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    required = ("timestamp_s", "distance_m", "geo_snr_db", "prob_snr_db")
    if not rows:
        raise ValueError("Spatial trace is empty")
    data = [{key: float(row[key]) for key in required} for row in rows]
    if not all(all(math.isfinite(value) for value in row.values()) and row["distance_m"] > 0
               for row in data):
        raise ValueError("Spatial trace contains invalid physical samples")
    if any(right["timestamp_s"] <= left["timestamp_s"] for left, right in zip(data, data[1:])):
        raise ValueError("Spatial sample times must increase")
    return data


def offers(start_s: float, end_s: float, *, semantic_bytes: int,
           feature_bytes: int, raw_bytes: int) -> list[tuple[float, str, int]]:
    result = []
    index = 0
    while start_s + index * 0.5 < end_s - 0.25:
        result.append((start_s + index * 0.5, f"semantic_{index:03d}", semantic_bytes))
        index += 1
    for offset, kind, size in ((6.0, "feature_0", feature_bytes),
                               (8.0, "raw_0", raw_bytes),
                               (12.0, "feature_1", feature_bytes)):
        if start_s + offset < end_s:
            result.append((start_s + offset, kind, size))
    return result


def average_aoi(queue: A2GTransportQueue, end_s: float) -> float | None:
    valid = sorted((item for item in queue.records if item.status == "available_valid"),
                   key=lambda item: (item.t_arr, item.message_id))
    if not valid:
        return None
    last_time = valid[0].t_arr
    latest_observation = valid[0].t_obs
    total = 0.0
    for item in valid[1:]:
        duration = item.t_arr - last_time
        total += duration * (last_time - latest_observation) + duration * duration / 2
        last_time = item.t_arr
        latest_observation = max(latest_observation, item.t_obs)
    duration = end_s - last_time
    total += duration * (last_time - latest_observation) + duration * duration / 2
    observed_duration = end_s - valid[0].t_arr
    return total / observed_duration if observed_duration > 0 else None


def summarize(queue: A2GTransportQueue, *, cap: float, attenuation: float,
              end_s: float) -> dict[str, Any]:
    valid = [item for item in queue.records if item.status == "available_valid"]
    sample = queue.summary()
    sample.update({
        "rate_cap_bit_s": cap,
        "extra_attenuation_db": attenuation,
        "median_valid_age_s": (statistics.median(item.t_arr - item.t_obs for item in valid)
                               if valid else None),
        "mean_valid_queue_wait_s": (statistics.mean(item.t_start - item.t_enq for item in valid)
                                    if valid else None),
        "mean_aoi_after_first_delivery_s": average_aoi(queue, end_s),
        "status_by_probe": {
            kind: {status: sum(item.message_id.startswith(kind)
                                 and item.status == status for item in queue.records)
                   for status in sorted({item.status for item in queue.records})}
            for kind in ("semantic", "feature", "raw")
        },
    })
    return sample


def plot_rate_grid(rows: list[dict[str, float]], config: A2GTransportConfig,
                   output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.family": ["Microsoft YaHei", "SimHei", "DejaVu Sans"], "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.dpi": 300})
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.4), sharex=True)
    times = [row["timestamp_s"] - rows[0]["timestamp_s"] for row in rows]
    for row_index, attenuation in enumerate((0.0, 15.0)):
        for col_index, cap in enumerate((100e6, 10e6)):
            axis = axes[row_index][col_index]
            chosen = replace(config, rate_cap_bit_s=cap)
            for group, color, style in (("geo", "#225B91", "-"), ("prob", "#D0822C", "--")):
                rates = [chosen.rate_bit_s(row[f"{group}_snr_db"] - attenuation) / 1e6
                         for row in rows]
                axis.step(times, rates, where="post", color=color, linestyle=style,
                          linewidth=1.8, label="几何射线" if group == "geo" else "仰角概率")
            attenuation_label = "无额外衰减" if attenuation == 0 else "附加 15 dB 衰减"
            cap_label = "100 Mb/s 业务上限" if cap == 100e6 else "10 Mb/s 业务上限"
            ceiling_note = "（本模型最高约 33.9）" if cap == 100e6 else "（最高 10）"
            axis.set_title(f"{attenuation_label}｜{cap_label}{ceiling_note}", fontsize=10, pad=9)
            axis.set_ylabel("当前可用传输速率（Mb/s）")
            axis.set_xlim(0, 20)
            axis.set_ylim(bottom=-0.7, top=36 if cap == 100e6 else 11)
            axis.grid(alpha=0.2)
            if row_index == 1:
                axis.set_xlabel("轨迹经过时间（秒）")
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.suptitle("固定轨迹上每一时刻的可用传输速度", fontsize=14, y=0.985)
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.94),
               ncol=2, frameon=False)
    fig.subplots_adjust(top=0.86, bottom=0.12, left=0.09, right=0.98,
                        hspace=0.34, wspace=0.14)
    fig.text(0.5, 0.025,
             "水平线＝受上限限制　｜　曲线下探＝信道变差　｜　降到 0＝暂时无法传输",
             ha="center", fontsize=10)
    for extension in ("png", "pdf"):
        fig.savefig(output.with_suffix(f".{extension}"), dpi=300)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transport-config", type=Path,
                        default=ROOT / "configs/communication/a2g_transport_v1.yaml")
    parser.add_argument("--spatial-config", type=Path,
                        default=ROOT / "configs/communication/a2g_spatial_v1.yaml")
    parser.add_argument("--semantic-bytes", type=int, default=704)
    args = parser.parse_args()
    trace = load_trace(args.trace)
    transport = yaml.safe_load(args.transport_config.read_text(encoding="utf-8"))["transport_link"]
    spatial = yaml.safe_load(args.spatial_config.read_text(encoding="utf-8"))["spatial_link"]
    base = A2GTransportConfig.from_mapping(transport, bandwidth_hz=float(spatial["bandwidth_hz"]))
    period_s = float(transport["clock"]["rate_sample_period_s"])
    if any(abs(b["timestamp_s"] - a["timestamp_s"] - period_s) > 1e-4
           for a, b in zip(trace, trace[1:])):
        raise ValueError("Trace sample interval conflicts with transport configuration")
    start_s = trace[0]["timestamp_s"]
    end_s = trace[-1]["timestamp_s"] + period_s
    offered = offers(start_s, end_s, semantic_bytes=args.semantic_bytes,
                     feature_bytes=int(transport["probes"]["feature_bytes_placeholder"]),
                     raw_bytes=int(transport["probes"]["raw_rgb_bytes_placeholder"]))
    timeline = [(row["timestamp_s"], 0, row) for row in trace]
    timeline.extend((when, 1, (message_id, size)) for when, message_id, size in offered)
    timeline.sort(key=lambda item: (item[0], item[1]))
    args.output.mkdir(parents=True, exist_ok=True)
    summaries: list[dict[str, Any]] = []
    for attenuation in (0.0, 15.0):
        for cap in (100e6, 10e6, 2e6, 0.5e6):
            for group in ("geo", "prob"):
                queue = A2GTransportQueue(replace(base, rate_cap_bit_s=cap), group)
                for when, kind, content in timeline:
                    if kind == 0:
                        row = content
                        queue.update_channel(when,
                                             snr_db=row[f"{group}_snr_db"] - attenuation,
                                             distance_m=row["distance_m"])
                    else:
                        message_id, size = content
                        queue.enqueue({"message_id": message_id, "source_timestamp_s": when},
                                      enqueue_timestamp_s=when,
                                      payload_bytes_override=size)
                queue.end_episode(end_s)
                key = f"{group}_cap{cap/1e6:g}Mbps_atten{attenuation:g}dB"
                path = args.output / f"events_{key}.jsonl"
                path.write_text("".join(json.dumps(event, ensure_ascii=False) + "\n"
                                        for event in queue.events), encoding="utf-8")
                summaries.append(summarize(queue, cap=cap, attenuation=attenuation, end_s=end_s))
    output = {
        "experiment_type": "fixed_trajectory_transport_replay",
        "interpretation": "Synthetic 2 Hz semantic-size and placeholder feature/raw probes; no task-control or physical PER claim",
        "source": {
            "trace": str(args.trace), "trace_sha256": hashlib.sha256(args.trace.read_bytes()).hexdigest(),
            "transport_config": str(args.transport_config),
            "transport_config_sha256": hashlib.sha256(args.transport_config.read_bytes()).hexdigest(),
            "spatial_config": str(args.spatial_config),
            "spatial_config_sha256": hashlib.sha256(args.spatial_config.read_bytes()).hexdigest(),
        },
        "trajectory_samples": len(trace),
        "duration_s": end_s - start_s,
        "synthetic_offers": len(offered),
        "synthetic_offer_sizes_byte": {"semantic": args.semantic_bytes,
                                       "feature": int(transport["probes"]["feature_bytes_placeholder"]),
                                       "raw": int(transport["probes"]["raw_rgb_bytes_placeholder"])},
        "cases": summaries,
    }
    (args.output / "replay_summary.json").write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    plot_rate_grid(trace, base, args.output / "effective_rate_fixed_trajectory")
    print(f"Wrote {args.output}: {len(summaries)} paired transport cases, {len(offered)} synthetic offers each")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

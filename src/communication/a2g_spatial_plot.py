"""Reproducible, publication-sized figures for one A2G spatial trace."""

from __future__ import annotations

import ast
import csv
from pathlib import Path
from typing import Any, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch


BLUE = "#176B9A"
ORANGE = "#C45A23"
DARK = "#253440"
GRID = "#DCE2E5"


def read_trace(path: str | Path) -> list[dict[str, Any]]:
    """Read the saved diagnostic CSV without requiring CARLA."""
    coordinates = {"uav_antenna_xyz_m", "ugv_antenna_xyz_m"}
    strings = {"first_static_blocker"}
    integers = {"frame", "geo_los", "prob_los", "ray_hit_count"}
    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.DictReader(handle))
    for row in rows:
        for key, value in row.items():
            if key in coordinates:
                row[key] = tuple(float(x) for x in ast.literal_eval(value))
            elif key in strings:
                continue
            elif key in integers:
                row[key] = int(value)
            else:
                row[key] = float(value)
    return rows


def _style() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 8.5,
        "axes.labelsize": 9,
        "axes.titlesize": 9.5,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.75,
        "xtick.direction": "out",
        "ytick.direction": "out",
        "pdf.fonttype": 42,
        "savefig.facecolor": "white",
    })


def _finish_axis(ax: Any, letter: str) -> None:
    ax.grid(color=GRID, lw=0.55, zorder=0)
    ax.text(0.0, 1.04, letter, transform=ax.transAxes,
            fontsize=11, fontweight="bold", color=DARK)


def _save(fig: Any, output_stem: Path) -> None:
    output_stem.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_stem.with_suffix(".png"), dpi=300, bbox_inches="tight", pad_inches=0.08)
    fig.savefig(output_stem.with_suffix(".pdf"), bbox_inches="tight", pad_inches=0.08)
    plt.close(fig)


def plot_publication_figures(rows: Sequence[dict[str, Any]], output_dir: str | Path) -> list[Path]:
    """Separate shared geometry from propagation-state comparisons.

    Each PDF uses vector line work and embedded TrueType fonts. The PNG is
    a 300 dpi review copy. Neither figure claims population-level inference.
    """
    if not rows:
        raise ValueError("A2G spatial trace is empty")
    _style()
    output_dir = Path(output_dir)
    t = np.asarray([float(row["timestamp_s"]) - float(rows[0]["timestamp_s"])
                    for row in rows])
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("A2G spatial timestamps must increase")

    uav = np.asarray([row["uav_antenna_xyz_m"] for row in rows], dtype=float)
    ugv = np.asarray([row["ugv_antenna_xyz_m"] for row in rows], dtype=float)
    horizontal = np.asarray([row["horizontal_m"] for row in rows], dtype=float)
    distance = np.asarray([row["distance_m"] for row in rows], dtype=float)
    height = np.asarray([row["height_m"] for row in rows], dtype=float)
    elevation = np.asarray([row["elevation_deg"] for row in rows], dtype=float)
    if not np.allclose(distance, np.hypot(horizontal, height), rtol=1e-8, atol=1e-8):
        raise ValueError("Distance is inconsistent with horizontal and height components")

    fig, axes = plt.subplots(2, 2, figsize=(7.2, 5.7), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(uav[:, 0], uav[:, 1], color=BLUE, lw=1.8, label="UAV")
    ax.plot(ugv[:, 0], ugv[:, 1], color=ORANGE, lw=1.8, ls="--", label="UGV")
    for points, color in ((uav, BLUE), (ugv, ORANGE)):
        ax.scatter(*points[0, :2], marker="o", s=28, facecolor="white",
                   edgecolor=color, linewidth=1.2, zorder=5)
        ax.scatter(*points[-1, :2], marker="s", s=24, color=color, zorder=5)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("World x (m)")
    ax.set_ylabel("World y (m)")
    ax.legend(frameon=False, loc="best", ncol=2)
    _finish_axis(ax, "a")

    ax = axes[0, 1]
    ax.plot(t, horizontal, color=BLUE, lw=1.8, label="Horizontal separation")
    ax.plot(t, distance, color=ORANGE, lw=1.8, ls="--", label="3D distance")
    ax.set_ylabel("Separation (m)")
    ax.legend(frameon=False, loc="best")
    _finish_axis(ax, "b")

    ax = axes[1, 0]
    ax.plot(t, height, color=BLUE, lw=1.8)
    ax.set_xlabel("Time from first sample (s)")
    ax.set_ylabel("Absolute antenna height difference (m)")
    ax.set_ylim(0, max(45.0, 1.1 * float(height.max())))
    _finish_axis(ax, "c")

    ax = axes[1, 1]
    ax.plot(t, elevation, color=ORANGE, lw=1.8)
    ax.set_xlabel("Time from first sample (s)")
    ax.set_ylabel("Elevation angle (degrees)")
    _finish_axis(ax, "d")
    geometry_stem = output_dir / "spatial_geometry_v2"
    _save(fig, geometry_stem)

    geo_state = np.asarray([int(row["geo_los"]) for row in rows])
    prob_state = np.asarray([int(row["prob_los"]) for row in rows])
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 5.9), sharex=True,
                             gridspec_kw={"height_ratios": [0.85, 1.5, 1.5]},
                             constrained_layout=True)
    ax = axes[0]
    states = np.vstack((geo_state, prob_state))
    dt = np.median(np.diff(t))
    ax.imshow(states, interpolation="none", aspect="auto", origin="upper",
              cmap=ListedColormap(["#D3D8DA", "#176B9A"]), vmin=0, vmax=1,
              extent=(t[0] - dt / 2, t[-1] + dt / 2, 1.5, -0.5))
    ax.set_yticks([0, 1], labels=["Geometry", "Probability"])
    ax.tick_params(axis="y", length=0)
    ax.grid(False)
    ax.legend(handles=[Patch(facecolor="#176B9A", label="LoS"),
                       Patch(facecolor="#D3D8DA", label="NLoS")],
              loc="upper right", frameon=False, ncol=2,
              bbox_to_anchor=(1, 1.45))
    ax.text(-0.13, 1.14, "a", transform=ax.transAxes,
            fontsize=11, fontweight="bold", color=DARK)

    for ax, key, ylabel, letter in (
        (axes[1], "path_loss_db", "Path loss (dB)", "b"),
        (axes[2], "snr_db", "Received SNR (dB)", "c"),
    ):
        ax.plot(t, [row[f"geo_{key}"] for row in rows],
                color=BLUE, lw=1.8, label="Geometry", zorder=3)
        ax.plot(t, [row[f"prob_{key}"] for row in rows],
                color=ORANGE, lw=1.5, ls=(0, (4, 2)), label="Probability", zorder=4)
        ax.set_ylabel(ylabel)
        _finish_axis(ax, letter)
    axes[1].legend(loc="upper right", frameon=False, ncol=2)
    axes[-1].set_xlabel("Time from first sample (s)")
    axes[-1].set_xlim(0, t[-1])
    propagation_stem = output_dir / "spatial_propagation_v2"
    _save(fig, propagation_stem)
    return [geometry_stem.with_suffix(suffix) for suffix in (".png", ".pdf")] + [
        propagation_stem.with_suffix(suffix) for suffix in (".png", ".pdf")]

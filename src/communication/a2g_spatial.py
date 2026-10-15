"""Observation-only A2G spatial channel for a fixed CARLA trajectory.

This module never decides whether an application message is delivered.  The
Gaussian fields are sampled jointly on the recorded trajectory, so their
finite-dimensional covariance is the exponential kernel in the theory note.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Sequence

import numpy as np


C_MPS = 299_792_458.0
K_BOLTZMANN = 1.380649e-23
T0_K = 290.0


@dataclass(frozen=True)
class A2GSpatialConfig:
    carrier_hz: float
    transmit_power_dbm: float
    bandwidth_hz: float
    noise_figure_db: float
    tx_gain_dbi: float
    rx_gain_dbi: float
    system_loss_db: float
    eta_los_db: float
    eta_nlos_db: float
    shadow_sigma_los_db: float
    shadow_sigma_nlos_db: float
    shadow_corr_los_m: float
    shadow_corr_nlos_m: float
    los_a: float
    los_b: float
    latent_corr_m: float
    seed: int
    sample_every_n_ticks: int
    uav_antenna_offset_xyz_m: tuple[float, float, float]
    ugv_antenna_offset_xyz_m: tuple[float, float, float]
    static_labels: tuple[str, ...]
    max_exact_samples: int = 2500
    field_realization_mode: str = "causal_exact_v1"
    shared_field_features: int = 1024

    @classmethod
    def from_mapping(cls, mapping: dict[str, Any], *, run_seed: int) -> "A2GSpatialConfig":
        values = dict(mapping)
        values.pop("enabled", None)
        values.pop("parameter_status", None)
        values.pop("parameter_sources", None)
        values.pop("sensitivity_ranges", None)
        values["seed"] = int(values.pop("seed", run_seed))
        for key in ("uav_antenna_offset_xyz_m", "ugv_antenna_offset_xyz_m"):
            values[key] = tuple(float(v) for v in values[key])
            if len(values[key]) != 3:
                raise ValueError(f"{key} must contain exactly three coordinates")
        values["static_labels"] = tuple(str(v) for v in values["static_labels"])
        config = cls(**values)
        for key in ("carrier_hz", "bandwidth_hz", "shadow_corr_los_m",
                    "shadow_corr_nlos_m", "los_a", "los_b", "latent_corr_m"):
            if not math.isfinite(getattr(config, key)) or getattr(config, key) <= 0:
                raise ValueError(f"{key} must be finite and positive")
        for key in ("shadow_sigma_los_db", "shadow_sigma_nlos_db", "system_loss_db"):
            if not math.isfinite(getattr(config, key)) or getattr(config, key) < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        if config.eta_nlos_db <= config.eta_los_db or config.eta_los_db < 0:
            raise ValueError("Require eta_nlos_db > eta_los_db >= 0")
        if config.sample_every_n_ticks < 1 or config.max_exact_samples < 1:
            raise ValueError("Sampling intervals and sample limit must be positive")
        if config.field_realization_mode not in {"causal_exact_v1", "shared_spatial_v2"}:
            raise ValueError("field_realization_mode must be causal_exact_v1 or shared_spatial_v2")
        if not 64 <= config.shared_field_features <= 4096:
            raise ValueError("shared_field_features must be between 64 and 4096")
        if not config.static_labels:
            raise ValueError("The static obstruction label set cannot be empty")
        for key, value in asdict(config).items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{key} must be finite")
        return config


def antenna_xyz(actor: Any, offset_xyz_m: Sequence[float], carla: Any) -> tuple[float, float, float]:
    """Transform a local antenna mount into CARLA world coordinates."""
    local = carla.Location(*[float(v) for v in offset_xyz_m])
    world_location = actor.get_transform().transform(local)
    return float(world_location.x), float(world_location.y), float(world_location.z)


def geometry(uav_xyz: Sequence[float], ugv_xyz: Sequence[float]) -> tuple[float, float, float, float]:
    delta = np.asarray(uav_xyz, dtype=float) - np.asarray(ugv_xyz, dtype=float)
    if delta.shape != (3,) or not np.isfinite(delta).all():
        raise ValueError("Antenna coordinates must be finite world XYZ triples")
    horizontal_m = math.hypot(float(delta[0]), float(delta[1]))
    height_m = float(delta[2])
    distance_m = math.hypot(horizontal_m, height_m)
    if distance_m <= 0 or height_m < 0:
        raise ValueError("A2G geometry requires d > 0 and UAV antenna at or above UGV antenna")
    elevation_deg = math.degrees(math.asin(min(1.0, height_m / distance_m)))
    return horizontal_m, height_m, distance_m, elevation_deg


def static_los(world: Any, carla: Any, uav_xyz: Sequence[float], ugv_xyz: Sequence[float],
               labels: Sequence[str]) -> tuple[bool, str | None, int]:
    """Classify the centre ray with a frozen set of static CARLA labels."""
    start = carla.Location(*[float(v) for v in ugv_xyz])
    end = carla.Location(*[float(v) for v in uav_xyz])
    hits = world.cast_ray(start, end)
    selected = set(labels)
    for hit in hits:
        label = str(getattr(hit.label, "name", str(hit.label).split(".")[-1]))
        if label in selected:
            return False, label, len(hits)
    return True, None, len(hits)


def observe_world(world: Any, carla: Any, uav: Any, ugv: Any, config: A2GSpatialConfig,
                  *, frame: int, timestamp_s: float) -> dict[str, Any]:
    pu = antenna_xyz(uav, config.uav_antenna_offset_xyz_m, carla)
    pg = antenna_xyz(ugv, config.ugv_antenna_offset_xyz_m, carla)
    horizontal, height, distance, elevation = geometry(pu, pg)
    los, blocker, hit_count = static_los(world, carla, pu, pg, config.static_labels)
    return {
        "frame": int(frame), "timestamp_s": float(timestamp_s),
        "uav_antenna_xyz_m": pu, "ugv_antenna_xyz_m": pg,
        "horizontal_m": horizontal, "height_m": height,
        "distance_m": distance, "elevation_deg": elevation,
        "geo_los": int(los), "first_static_blocker": blocker,
        "ray_hit_count": hit_count,
    }


def _gaussian_field(positions: np.ndarray, *, sigma: float, corr_m: float,
                    seed: int) -> np.ndarray:
    """Exact exponential covariance on the finite set of sampled positions."""
    if sigma == 0:
        return np.zeros(len(positions), dtype=float)
    unique, inverse = np.unique(positions, axis=0, return_inverse=True)
    centred = unique - unique[0]
    sq = np.sum(centred * centred, axis=1)
    distance_sq = np.maximum(0.0, sq[:, None] + sq[None, :] - 2.0 * centred @ centred.T)
    covariance = np.exp(-np.sqrt(distance_sq) / corr_m)
    # Duplicate positions have already been removed. The tiny diagonal term is
    # numerical jitter, not an independently resampled channel component.
    factor = np.linalg.cholesky(covariance + 1e-10 * np.eye(len(unique)))
    rng = np.random.default_rng(seed)
    return (sigma * (factor @ rng.standard_normal(len(unique))))[inverse]


class CausalGaussianField:
    """Incremental exact exponential Gaussian field; no future track is read."""

    def __init__(self, sigma: float, corr_m: float, seed: int):
        self.sigma = float(sigma)
        self.corr_m = float(corr_m)
        self.rng = np.random.default_rng(seed)
        self.positions: list[np.ndarray] = []
        self.values: list[float] = []
        self.innovations: list[float] = []
        self.factor = np.empty((0, 0), dtype=float)

    def sample(self, position: Sequence[float]) -> float:
        point = np.asarray(position, dtype=float)
        if point.shape != (6,) or not np.isfinite(point).all():
            raise ValueError("Gaussian field position must be a finite 6D antenna pair")
        for index, previous in enumerate(self.positions):
            if np.array_equal(point, previous):
                return self.values[index]
        if self.sigma == 0:
            self.positions.append(point)
            self.values.append(0.0)
            return 0.0
        n = len(self.positions)
        if n:
            kernel = np.exp(-np.asarray([np.linalg.norm(point - old) for old in self.positions]) / self.corr_m)
            lower = np.empty(n, dtype=float)
            for index in range(n):
                lower[index] = (kernel[index] - float(self.factor[index, :index] @ lower[:index])) / self.factor[index, index]
            variance = max(1e-10, 1.0 + 1e-10 - float(lower @ lower))
        else:
            lower = np.empty(0, dtype=float)
            variance = 1.0 + 1e-10
        innovation = float(self.rng.standard_normal())
        standardized = float(lower @ np.asarray(self.innovations) + math.sqrt(variance) * innovation)
        updated = np.zeros((n + 1, n + 1), dtype=float)
        updated[:n, :n] = self.factor
        updated[n, :n] = lower
        updated[n, n] = math.sqrt(variance)
        self.factor = updated
        self.positions.append(point)
        self.innovations.append(innovation)
        value = self.sigma * standardized
        self.values.append(value)
        return value


class SharedSpatialGaussianField:
    """Path-order independent, seeded approximation to an exponential Gaussian field.

    A multivariate Cauchy spectral draw has characteristic function
    exp(-||delta||/corr_m). Fixed Fourier coefficients therefore give the
    desired covariance in expectation over seeds, with finite-feature error.
    """

    def __init__(self, sigma: float, corr_m: float, seed: int, features: int = 1024):
        self.sigma = float(sigma)
        self.corr_m = float(corr_m)
        self.features = int(features)
        rng = np.random.default_rng(seed)
        normal = rng.standard_normal((self.features, 6))
        denominator = np.maximum(np.abs(rng.standard_normal((self.features, 1))), 1e-12)
        self.frequencies = normal / denominator / self.corr_m
        self.cos_weights = rng.standard_normal(self.features)
        self.sin_weights = rng.standard_normal(self.features)

    def sample(self, position: Sequence[float]) -> float:
        point = np.asarray(position, dtype=float)
        if point.shape != (6,) or not np.isfinite(point).all():
            raise ValueError("Gaussian field position must be a finite 6D antenna pair")
        if self.sigma == 0:
            return 0.0
        phase = self.frequencies @ point
        return float(self.sigma * (
            self.cos_weights @ np.cos(phase) + self.sin_weights @ np.sin(phase)
        ) / math.sqrt(self.features))


class CausalA2GSpatialSampler:
    """Online counterpart to evaluate_track with temporal sampling order."""

    def __init__(self, config: A2GSpatialConfig):
        self.config = config
        children = np.random.SeedSequence(config.seed).spawn(3)
        seeds = [int(child.generate_state(1)[0]) for child in children]
        if config.field_realization_mode == "shared_spatial_v2":
            field = lambda sigma, corr, seed: SharedSpatialGaussianField(
                sigma, corr, seed, config.shared_field_features
            )
        else:
            field = CausalGaussianField
        self.latent = field(1.0, config.latent_corr_m, seeds[0])
        self.shadow_los = field(config.shadow_sigma_los_db, config.shadow_corr_los_m, seeds[1])
        self.shadow_nlos = field(config.shadow_sigma_nlos_db, config.shadow_corr_nlos_m, seeds[2])

    def sample(self, observation: dict[str, Any]) -> dict[str, Any]:
        config = self.config
        position = [*observation["uav_antenna_xyz_m"], *observation["ugv_antenna_xyz_m"]]
        latent = self.latent.sample(position)
        shadow_los = self.shadow_los.sample(position)
        shadow_nlos = self.shadow_nlos.sample(position)
        distance = float(observation["distance_m"])
        elevation = float(observation["elevation_deg"])
        if distance <= 0 or not 0 <= elevation <= 90:
            raise ValueError("Invalid A2G distance or elevation")
        p_los = 1.0 / (1.0 + config.los_a * math.exp(-config.los_b * (elevation - config.los_a)))
        prob_los = int(0.5 * (1.0 + math.erf(latent / math.sqrt(2.0))) <= p_los)
        fspl_db = 20.0 * math.log10(4.0 * math.pi * distance * config.carrier_hz / C_MPS)
        noise_w = K_BOLTZMANN * T0_K * config.bandwidth_hz * 10 ** (config.noise_figure_db / 10.0)
        noise_dbm = 10.0 * math.log10(noise_w / 1e-3)
        row = {**observation, "p_los": p_los, "latent_y": latent,
               "prob_los": prob_los, "fspl_db": fspl_db, "noise_dbm": noise_dbm}
        for group, los in (("geo", int(observation["geo_los"])), ("prob", prob_los)):
            extra = config.eta_los_db if los else config.eta_nlos_db
            shadow = shadow_los if los else shadow_nlos
            loss = fspl_db + extra + shadow
            received = (config.transmit_power_dbm + config.tx_gain_dbi + config.rx_gain_dbi
                        - config.system_loss_db - loss)
            snr = received - noise_dbm
            row.update({f"{group}_extra_loss_db": extra, f"{group}_shadow_db": shadow,
                        f"{group}_path_loss_db": loss, f"{group}_received_dbm": received,
                        f"{group}_snr_db": snr, f"{group}_snr_linear": 10 ** (snr / 10.0)})
        return row


def evaluate_track(observations: Sequence[dict[str, Any]],
                   config: A2GSpatialConfig) -> list[dict[str, Any]]:
    """Compute geometry, propagation and link budget for one fixed trajectory."""
    if not observations:
        return []
    if len(observations) > config.max_exact_samples:
        raise ValueError("Track exceeds max_exact_samples; use a coarser declared sampling interval")
    positions = np.asarray([
        [*row["uav_antenna_xyz_m"], *row["ugv_antenna_xyz_m"]]
        for row in observations
    ], dtype=float)
    if positions.shape != (len(observations), 6) or not np.isfinite(positions).all():
        raise ValueError("Track contains invalid antenna coordinates")
    seed_sequence = np.random.SeedSequence(config.seed)
    seeds = [int(child.generate_state(1)[0]) for child in seed_sequence.spawn(3)]
    if config.field_realization_mode == "shared_spatial_v2":
        fields = [
            SharedSpatialGaussianField(1.0, config.latent_corr_m, seeds[0], config.shared_field_features),
            SharedSpatialGaussianField(config.shadow_sigma_los_db, config.shadow_corr_los_m,
                                       seeds[1], config.shared_field_features),
            SharedSpatialGaussianField(config.shadow_sigma_nlos_db, config.shadow_corr_nlos_m,
                                       seeds[2], config.shared_field_features),
        ]
        latent_y, shadow_los, shadow_nlos = (
            np.asarray([field.sample(position) for position in positions], dtype=float)
            for field in fields
        )
    else:
        latent_y = _gaussian_field(positions, sigma=1.0, corr_m=config.latent_corr_m, seed=seeds[0])
        shadow_los = _gaussian_field(positions, sigma=config.shadow_sigma_los_db,
                                     corr_m=config.shadow_corr_los_m, seed=seeds[1])
        shadow_nlos = _gaussian_field(positions, sigma=config.shadow_sigma_nlos_db,
                                      corr_m=config.shadow_corr_nlos_m, seed=seeds[2])
    noise_w = K_BOLTZMANN * T0_K * config.bandwidth_hz * 10 ** (config.noise_figure_db / 10.0)
    noise_dbm = 10.0 * math.log10(noise_w / 1e-3)
    rows: list[dict[str, Any]] = []
    for index, observation in enumerate(observations):
        distance = float(observation["distance_m"])
        elevation = float(observation["elevation_deg"])
        if distance <= 0 or not 0 <= elevation <= 90:
            raise ValueError("Invalid A2G distance or elevation")
        p_los = 1.0 / (1.0 + config.los_a * math.exp(-config.los_b * (elevation - config.los_a)))
        u = 0.5 * (1.0 + math.erf(float(latent_y[index]) / math.sqrt(2.0)))
        prob_los = int(u <= p_los)
        fspl_db = 20.0 * math.log10(4.0 * math.pi * distance * config.carrier_hz / C_MPS)
        row = {**observation, "p_los": p_los, "latent_y": float(latent_y[index]),
               "prob_los": prob_los, "fspl_db": fspl_db, "noise_dbm": noise_dbm}
        for group, los in (("geo", int(observation["geo_los"])), ("prob", prob_los)):
            extra_db = config.eta_los_db if los else config.eta_nlos_db
            shadow_db = float(shadow_los[index] if los else shadow_nlos[index])
            loss_db = fspl_db + extra_db + shadow_db
            received_dbm = (config.transmit_power_dbm + config.tx_gain_dbi
                            + config.rx_gain_dbi - config.system_loss_db - loss_db)
            snr_db = received_dbm - noise_dbm
            row.update({f"{group}_extra_loss_db": extra_db,
                        f"{group}_shadow_db": shadow_db,
                        f"{group}_path_loss_db": loss_db,
                        f"{group}_received_dbm": received_dbm,
                        f"{group}_snr_db": snr_db,
                        f"{group}_snr_linear": 10 ** (snr_db / 10.0)})
        rows.append(row)
    return rows


def summarize_track(rows: Sequence[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"sample_count": 0}
    def extent(key: str) -> dict[str, float]:
        values = [float(row[key]) for row in rows]
        return {"min": min(values), "max": max(values), "mean": sum(values) / len(values)}
    return {
        "sample_count": len(rows),
        "time_start_s": rows[0]["timestamp_s"],
        "time_end_s": rows[-1]["timestamp_s"],
        "distance_m": extent("distance_m"),
        "elevation_deg": extent("elevation_deg"),
        "geo_los_fraction": sum(row["geo_los"] for row in rows) / len(rows),
        "prob_los_fraction": sum(row["prob_los"] for row in rows) / len(rows),
        "geo_snr_db": extent("geo_snr_db"),
        "prob_snr_db": extent("prob_snr_db"),
        "static_blocker_counts": {
            str(label): sum(row["first_static_blocker"] == label for row in rows)
            for label in sorted({row["first_static_blocker"] for row in rows if row["first_static_blocker"]})
        },
    }


def plot_track(rows: Sequence[dict[str, Any]], output_path: Any) -> None:
    """Save a compact diagnostic figure from the raw fixed-trajectory trace."""
    if not rows:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t0 = float(rows[0]["timestamp_s"])
    t = [float(row["timestamp_s"]) - t0 for row in rows]
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), dpi=150, sharex=True)
    axes[0].plot(t, [row["distance_m"] for row in rows], label="3D distance (m)")
    angle_axis = axes[0].twinx()
    angle_axis.plot(t, [row["elevation_deg"] for row in rows], color="#d88720", label="Elevation (deg)")
    axes[0].set_ylabel("Distance (m)")
    angle_axis.set_ylabel("Elevation (deg)")
    axes[1].step(t, [row["geo_los"] for row in rows], where="post", label="Town10HD geometry")
    axes[1].step(t, [row["prob_los"] for row in rows], where="post", label="Probability control", alpha=0.7)
    axes[1].set_ylabel("LoS state")
    axes[1].set_yticks([0, 1])
    axes[1].legend(loc="upper right")
    for axis, key, ylabel in ((axes[2], "path_loss_db", "Path loss (dB)"),
                              (axes[3], "snr_db", "SNR (dB)")):
        for group, label in (("geo", "Geometry"), ("prob", "Probability")):
            axis.plot(t, [row[f"{group}_{key}"] for row in rows], label=label)
        axis.set_ylabel(ylabel)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("Time since first spatial sample (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle("A2G spatial link: observation only, fixed realized trajectory")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)

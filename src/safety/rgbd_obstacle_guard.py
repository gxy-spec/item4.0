"""UGV obstacle guard driven by its forward metric-depth camera.

The guard does not consume simulator obstacle actors. It projects sampled depth
pixels through the calibrated camera mount into the vehicle frame and reports
the nearest occupied point in the forward corridor.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ObstacleGuardConfig:
    horizontal_fov_degrees: float = 100.0
    maximum_range_m: float = 18.0
    corridor_half_width_m: float = 1.6
    minimum_height_m: float = 0.12
    maximum_height_m: float = 2.6
    warning_distance_m: float = 8.0
    stop_distance_m: float = 4.0
    warning_ttc_s: float = 3.0
    stop_ttc_s: float = 1.5
    stale_after_s: float = 0.30
    pixel_stride: int = 4
    recovery_clear_frames: int = 5
    minimum_occupied_points: int = 3


class RGBDObstacleGuard:
    """Depth occupancy and conservative distance/TTC state machine."""

    def __init__(self, config: ObstacleGuardConfig | None = None) -> None:
        self.config = config or ObstacleGuardConfig()
        self._last_frame: int | None = None
        self._last_timestamp: float | None = None
        self._last_distance_m: float | None = None
        self._state = "CLEAR"
        self._clear_frames = 0
        self.last_obstacle_pixels_uv: np.ndarray = np.empty((0, 2), dtype=np.int32)

    def observe(
        self,
        depth_m: np.ndarray | None,
        frame: int | None,
        timestamp: float | None,
        now_seconds: float,
        camera_to_vehicle: np.ndarray,
    ) -> dict[str, Any]:
        cfg = self.config
        new_frame = frame is not None and frame != self._last_frame
        age_s = math.inf if timestamp is None else max(0.0, float(now_seconds) - float(timestamp))
        if depth_m is None or timestamp is None or age_s > cfg.stale_after_s:
            self._state = "SENSOR_STALE"
            self._clear_frames = 0
            self.last_obstacle_pixels_uv = np.empty((0, 2), dtype=np.int32)
            return {
                "state": self._state,
                "nearest_obstacle_m": math.nan,
                "closing_speed_mps": math.nan,
                "ttc_s": math.nan,
                "occupied_points": 0,
                "sensor_age_s": age_s,
                "source_frame": frame,
                "reason": "missing_or_stale_ugv_depth",
            }

        if not new_frame:
            return {
                "state": self._state,
                "nearest_obstacle_m": (
                    math.nan if self._last_distance_m is None else self._last_distance_m
                ),
                "closing_speed_mps": math.nan,
                "ttc_s": math.nan,
                "occupied_points": int(len(self.last_obstacle_pixels_uv)),
                "sensor_age_s": age_s,
                "source_frame": frame,
                "reason": "reused_latest_sensor_frame",
            }

        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2 or depth.size == 0:
            self._state = "SENSOR_STALE"
            self._clear_frames = 0
            return {
                "state": self._state,
                "nearest_obstacle_m": math.nan,
                "closing_speed_mps": math.nan,
                "ttc_s": math.nan,
                "occupied_points": 0,
                "sensor_age_s": age_s,
                "source_frame": frame,
                "reason": "invalid_depth_shape",
            }

        height, width = depth.shape
        fx = width / (2.0 * math.tan(math.radians(cfg.horizontal_fov_degrees) / 2.0))
        fy = fx
        cx, cy = (width - 1.0) / 2.0, (height - 1.0) / 2.0
        stride = max(1, int(cfg.pixel_stride))
        vv, uu = np.mgrid[0:height:stride, 0:width:stride]
        z_forward = depth[::stride, ::stride].reshape(-1)
        u = uu.reshape(-1).astype(np.float32)
        v = vv.reshape(-1).astype(np.float32)
        valid = (
            np.isfinite(z_forward)
            & (z_forward > 0.2)
            & (z_forward <= cfg.maximum_range_m)
        )
        u, v, z_forward = u[valid], v[valid], z_forward[valid]
        if z_forward.size:
            points_camera = np.column_stack(
                (
                    z_forward,
                    (u - cx) * z_forward / fx,
                    -(v - cy) * z_forward / fy,
                    np.ones_like(z_forward),
                )
            )
            points_vehicle = (np.asarray(camera_to_vehicle, dtype=np.float64) @ points_camera.T).T[:, :3]
            forward = points_vehicle[:, 0]
            lateral = points_vehicle[:, 1]
            vertical = points_vehicle[:, 2]
            occupied = (
                (forward > 0.3)
                & (forward <= cfg.maximum_range_m)
                & (np.abs(lateral) <= cfg.corridor_half_width_m)
                & (vertical >= cfg.minimum_height_m)
                & (vertical <= cfg.maximum_height_m)
            )
            obstacle_forward = forward[occupied]
            obstacle_u, obstacle_v = u[occupied], v[occupied]
        else:
            obstacle_forward = np.empty(0, dtype=np.float32)
            obstacle_u = obstacle_v = np.empty(0, dtype=np.float32)

        enough_points = obstacle_forward.size >= cfg.minimum_occupied_points
        distance = float(np.percentile(obstacle_forward, 5)) if enough_points else math.inf
        closing_speed = 0.0
        ttc = math.inf
        if (
            self._last_distance_m is not None
            and self._last_timestamp is not None
            and math.isfinite(distance)
        ):
            dt = float(timestamp) - self._last_timestamp
            if dt > 1e-4:
                closing_speed = max(0.0, (self._last_distance_m - distance) / dt)
                if closing_speed > 0.25:
                    ttc = distance / closing_speed

        danger = distance <= cfg.stop_distance_m or ttc <= cfg.stop_ttc_s
        warning = distance <= cfg.warning_distance_m or ttc <= cfg.warning_ttc_s
        if danger:
            self._state = "STOP"
            self._clear_frames = 0
        elif warning:
            self._state = "CAUTION"
            self._clear_frames = 0
        else:
            self._clear_frames += 1
            if self._state in {"STOP", "CAUTION"} and self._clear_frames < cfg.recovery_clear_frames:
                pass
            else:
                self._state = "CLEAR"

        self._last_frame = int(frame) if frame is not None else None
        self._last_timestamp = float(timestamp)
        self._last_distance_m = distance if math.isfinite(distance) else None
        self.last_obstacle_pixels_uv = (
            np.column_stack((obstacle_u, obstacle_v)).astype(np.int32)
            if enough_points
            else np.empty((0, 2), dtype=np.int32)
        )
        reason = {
            "STOP": "depth_obstacle_or_ttc_stop_threshold",
            "CAUTION": "depth_obstacle_or_ttc_warning_threshold",
            "CLEAR": "no_obstacle_in_forward_corridor",
        }.get(self._state, "waiting_for_clear_depth_frames")
        return {
            "state": self._state,
            "nearest_obstacle_m": distance if math.isfinite(distance) else math.nan,
            "closing_speed_mps": closing_speed,
            "ttc_s": ttc if math.isfinite(ttc) else math.nan,
            "occupied_points": int(obstacle_forward.size),
            "sensor_age_s": age_s,
            "source_frame": frame,
            "reason": reason,
        }

    def lane_occupancy(
        self,
        depth_m: np.ndarray,
        camera_to_vehicle: np.ndarray,
        lateral_center_m: float,
        half_width_m: float = 1.15,
        minimum_forward_m: float = 2.5,
        maximum_forward_m: float = 20.0,
    ) -> dict[str, Any]:
        """Check an adjacent forward corridor using depth only, not actor states."""
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2 or depth.size == 0:
            return {"clear": False, "occupied_points": -1, "nearest_m": math.nan}
        height, width = depth.shape
        fx = width / (2.0 * math.tan(math.radians(self.config.horizontal_fov_degrees) / 2.0))
        fy = fx
        cx, cy = (width - 1.0) / 2.0, (height - 1.0) / 2.0
        stride = max(1, int(self.config.pixel_stride))
        vv, uu = np.mgrid[0:height:stride, 0:width:stride]
        forward_camera = depth[::stride, ::stride].reshape(-1)
        u, v = uu.reshape(-1).astype(np.float32), vv.reshape(-1).astype(np.float32)
        valid = np.isfinite(forward_camera) & (forward_camera > 0.2) & (forward_camera <= self.config.maximum_range_m)
        u, v, forward_camera = u[valid], v[valid], forward_camera[valid]
        if not forward_camera.size:
            return {"clear": True, "occupied_points": 0, "nearest_m": math.inf}
        points_camera = np.column_stack(
            (forward_camera, (u - cx) * forward_camera / fx, -(v - cy) * forward_camera / fy, np.ones_like(forward_camera))
        )
        points_vehicle = (np.asarray(camera_to_vehicle, dtype=np.float64) @ points_camera.T).T[:, :3]
        forward, lateral, vertical = points_vehicle[:, 0], points_vehicle[:, 1], points_vehicle[:, 2]
        occupied = (
            (forward >= minimum_forward_m)
            & (forward <= maximum_forward_m)
            & (np.abs(lateral - float(lateral_center_m)) <= half_width_m)
            & (vertical >= self.config.minimum_height_m)
            & (vertical <= self.config.maximum_height_m)
        )
        count = int(np.count_nonzero(occupied))
        nearest = float(np.percentile(forward[occupied], 5)) if count else math.inf
        # Require more than a single noisy depth pixel to declare the pass lane blocked.
        threshold = max(3, int(self.config.minimum_occupied_points))
        return {"clear": count < threshold, "occupied_points": count, "nearest_m": nearest}


def camera_mount_matrix(location_xyz: list[float], rotation_pyr: list[float]) -> np.ndarray:
    """Return camera-to-vehicle homogeneous matrix using CARLA transform rules."""
    import carla

    transform = carla.Transform(
        carla.Location(*[float(value) for value in location_xyz]),
        carla.Rotation(
            pitch=float(rotation_pyr[0]),
            yaw=float(rotation_pyr[1]),
            roll=float(rotation_pyr[2]),
        ),
    )
    return np.asarray(transform.get_matrix(), dtype=np.float64)


def save_obstacle_preview(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    obstacle_pixels_uv: np.ndarray,
    observation: dict[str, Any],
    destination: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    destination.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), dpi=150)
    axes[0].imshow(rgb)
    if len(obstacle_pixels_uv):
        axes[0].scatter(
            obstacle_pixels_uv[:, 0], obstacle_pixels_uv[:, 1], s=2, c="#ff3030", alpha=0.45
        )
    axes[0].set_title("UGV RGB with RGB-D obstacle points")
    axes[0].axis("off")
    clipped = np.clip(depth_m, 0.0, 30.0)
    image = axes[1].imshow(clipped, cmap="turbo", vmin=0.0, vmax=30.0)
    if len(obstacle_pixels_uv):
        axes[1].scatter(
            obstacle_pixels_uv[:, 0], obstacle_pixels_uv[:, 1], s=2, c="white", alpha=0.55
        )
    fig.colorbar(image, ax=axes[1], label="Depth (m)")
    axes[1].set_title(
        f"Safety: {observation['state']}; nearest obstacle: {observation['nearest_obstacle_m']:.2f} m"
        if math.isfinite(float(observation.get("nearest_obstacle_m", math.nan)))
        else f"Safety: {observation['state']}; no obstacle in corridor"
    )
    axes[1].axis("off")
    fig.tight_layout()
    fig.savefig(destination, bbox_inches="tight")
    plt.close(fig)

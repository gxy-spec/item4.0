from __future__ import annotations

import numpy as np
from PIL import Image


def bgra_to_rgb(raw_data: bytes, width: int, height: int) -> np.ndarray:
    bgra = np.frombuffer(raw_data, dtype=np.uint8).reshape(height, width, 4)
    return bgra[:, :, :3][:, :, ::-1].copy()


def carla_depth_to_metres(raw_data: bytes, width: int, height: int, maximum_m: float = 1000.0) -> np.ndarray:
    bgra = np.frombuffer(raw_data, dtype=np.uint8).reshape(height, width, 4)
    red = bgra[:, :, 2].astype(np.float32)
    green = bgra[:, :, 1].astype(np.float32)
    blue = bgra[:, :, 0].astype(np.float32)
    normalized = (red + green * 256.0 + blue * 65536.0) / 16777215.0
    return normalized * float(maximum_m)


def colourise_depth(depth_m: np.ndarray, display_max_m: float = 100.0) -> Image.Image:
    clipped = np.clip(depth_m, 0.0, display_max_m) / display_max_m
    near = 1.0 - clipped
    red = np.clip(1.5 * near, 0.0, 1.0)
    green = np.clip(1.5 - np.abs(near - 0.5) * 3.0, 0.0, 1.0)
    blue = np.clip(1.5 * (1.0 - near), 0.0, 1.0)
    rgb = np.stack([red, green, blue], axis=-1)
    return Image.fromarray((rgb * 255.0).astype(np.uint8), mode="RGB")


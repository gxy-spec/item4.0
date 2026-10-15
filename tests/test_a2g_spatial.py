from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from communication.a2g_spatial import (  # noqa: E402
    A2GSpatialConfig,
    CausalA2GSpatialSampler,
    SharedSpatialGaussianField,
    evaluate_track,
    geometry,
    static_los,
)


def config(**overrides: object) -> A2GSpatialConfig:
    payload = yaml.safe_load((ROOT / "configs" / "communication" / "a2g_spatial_v1.yaml").read_text(
        encoding="utf-8"
    ))["spatial_link"]
    payload.update(overrides)
    return A2GSpatialConfig.from_mapping(payload, run_seed=1001)


def observation(distance_m: float, *, geo_los: int = 1, timestamp_s: float = 0.0) -> dict:
    return {
        "frame": int(timestamp_s * 10), "timestamp_s": timestamp_s,
        "uav_antenna_xyz_m": (distance_m, 0.0, 60.0),
        "ugv_antenna_xyz_m": (0.0, 0.0, 0.0),
        "distance_m": math.hypot(distance_m, 60.0),
        "elevation_deg": math.degrees(math.atan2(60.0, distance_m)),
        "geo_los": geo_los, "first_static_blocker": None,
        "ray_hit_count": 0,
    }


def test_geometry_uses_3d_distance_and_correct_elevation() -> None:
    horizontal, height, distance, angle = geometry((3, 4, 12), (0, 0, 0))
    assert (horizontal, height, distance) == (5.0, 12.0, 13.0)
    assert angle == pytest.approx(math.degrees(math.asin(12 / 13)))
    assert geometry((0, 0, 20), (0, 0, 1))[3] == 90.0
    with pytest.raises(ValueError):
        geometry((0, 0, 0), (0, 0, 0))


def test_carla_ray_uses_only_frozen_static_obstructions() -> None:
    fake_carla = SimpleNamespace(Location=lambda *xyz: xyz)
    hits = [SimpleNamespace(label=SimpleNamespace(name="Car")),
            SimpleNamespace(label=SimpleNamespace(name="Buildings"))]
    fake_world = SimpleNamespace(cast_ray=lambda start, end: hits)
    assert static_los(fake_world, fake_carla, (0, 0, 50), (0, 0, 1),
                      ("Buildings", "Walls")) == (False, "Buildings", 2)
    assert static_los(fake_world, fake_carla, (0, 0, 50), (0, 0, 1),
                      ("Walls",)) == (True, None, 2)


def test_loss_and_snr_follow_link_budget_without_shadow() -> None:
    model = config(shadow_sigma_los_db=0.0, shadow_sigma_nlos_db=0.0)
    near = observation(80.0)
    far = observation(math.sqrt(4.0 * near["distance_m"] ** 2 - 60.0 ** 2))
    rows = evaluate_track([near, far, {**near, "geo_los": 0}], model)
    assert rows[1]["fspl_db"] - rows[0]["fspl_db"] == pytest.approx(20 * math.log10(2))
    assert rows[2]["geo_path_loss_db"] - rows[0]["geo_path_loss_db"] == pytest.approx(19.0)
    assert rows[2]["geo_snr_db"] - rows[0]["geo_snr_db"] == pytest.approx(-19.0)
    for row in rows:
        assert row["geo_snr_linear"] == pytest.approx(10 ** (row["geo_snr_db"] / 10))


def test_seeded_fields_are_reproducible_and_repeated_positions_match() -> None:
    observations = [observation(80.0), observation(81.0, timestamp_s=0.1),
                    observation(80.0, timestamp_s=0.2)]
    first = evaluate_track(observations, config())
    second = evaluate_track(observations, config())
    assert first == second
    assert first[0]["latent_y"] == first[2]["latent_y"]
    assert first[0]["geo_shadow_db"] == first[2]["geo_shadow_db"]
    assert first[0]["prob_los"] == first[2]["prob_los"]


def test_causal_sampler_repeats_positions_and_does_not_read_future_track() -> None:
    prefix = [observation(80.0), observation(81.0, timestamp_s=0.1)]
    first = CausalA2GSpatialSampler(config())
    second = CausalA2GSpatialSampler(config())
    first_rows = [first.sample(item) for item in prefix]
    second_rows = [second.sample(item) for item in [*prefix, observation(90.0, timestamp_s=0.2)]]
    assert first_rows == second_rows[:2]
    repeated = first.sample({**prefix[0], "timestamp_s": 0.3})
    assert repeated["latent_y"] == first_rows[0]["latent_y"]
    assert repeated["geo_shadow_db"] == first_rows[0]["geo_shadow_db"]


def test_shared_field_same_position_is_independent_of_path_order() -> None:
    x = (0.0, 0.0, 60.0, 0.0, 0.0, 1.6)
    y = (50.0, 0.0, 60.0, 0.0, 0.0, 1.6)
    first = SharedSpatialGaussianField(4.0, 50.0, 1001)
    second = SharedSpatialGaussianField(4.0, 50.0, 1001)
    value = first.sample(x)
    second.sample(y)
    assert second.sample(x) == value
    assert first.sample(x) == value


def test_shared_field_online_and_offline_same_realization() -> None:
    model = config(field_realization_mode="shared_spatial_v2")
    track = [observation(80.0), observation(82.0, timestamp_s=0.1)]
    offline = evaluate_track(track, model)
    sampler = CausalA2GSpatialSampler(model)
    online = [sampler.sample(item) for item in track]
    for a, b in zip(online, offline):
        assert a["latent_y"] == b["latent_y"]
        assert a["geo_shadow_db"] == b["geo_shadow_db"]

    alternate = CausalA2GSpatialSampler(model)
    alternate.sample(track[1])
    revisited = alternate.sample(track[0])
    assert revisited["latent_y"] == online[0]["latent_y"]
    assert revisited["geo_shadow_db"] == online[0]["geo_shadow_db"]

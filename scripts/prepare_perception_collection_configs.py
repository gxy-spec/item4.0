#!/usr/bin/env python3
"""Generate 36 Town10HD instance-labelled perception collection runs."""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PROJECT_ROOT / "configs" / "experiments" / "multiscene_abc_formal_20261010_v2"
DEFAULT_OUTPUT = PROJECT_ROOT / "configs" / "experiments" / "perception_collection_town10hd_v1"

CATEGORIES = (
    ("van", "厢式货车", "vehicle.mercedes.sprinter"),
    ("car", "轿车", "vehicle.tesla.model3"),
    ("truck", "卡车", "vehicle.carlamotors.carlacola"),
    ("bus", "公交车", "vehicle.mitsubishi.fusorosa"),
)
COLORS = (
    ("red", "红色", "255,0,0"), ("blue", "蓝色", "0,0,255"),
    ("white", "白色", "255,255,255"), ("black", "黑色", "10,10,10"),
    ("gray", "灰色", "100,100,100"), ("yellow", "黄色", "255,200,0"),
)
ALTITUDES = (30.0, 45.0, 60.0)
PITCHES = (-90.0, -75.0, -60.0)


def set_uav_altitude(config: dict, altitude: float) -> None:
    uav = config["region"]["uav"]
    uav["altitude_m"] = altitude
    uav["initial_position_xyz"][2] = altitude
    for waypoint in uav["waypoints_xyz"]:
        waypoint[2] = altitude
    config["actors"]["uav"]["altitude_m"] = altitude
    config["dynamic"]["uav_altitude_m"] = altitude


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--data-root", type=Path,
        default=Path(r"E:\CarlaAirData\CityInspection_GOC\perception_collection_town10hd_v1"),
    )
    args = parser.parse_args()
    source_files = sorted(args.source.glob("*_A_formal.yaml"))
    if len(source_files) != 12:
        raise RuntimeError(f"Expected 12 independent scene configs, found {len(source_files)}")
    args.output.mkdir(parents=True, exist_ok=True)
    generated: list[Path] = []
    for scene_index, source in enumerate(source_files):
        base = yaml.safe_load(source.read_text(encoding="utf-8"))
        scene = source.stem.split("_")[0]
        for variant in range(3):
            config = copy.deepcopy(base)
            category, category_zh, blueprint = CATEGORIES[(scene_index * 3 + variant) % len(CATEGORIES)]
            color_index = (scene_index * 3 + variant) % len(COLORS)
            color, color_zh, color_rgb = COLORS[color_index]
            wrong_color_rgb = COLORS[(color_index + 1) % len(COLORS)][2]
            altitude = ALTITUDES[variant]
            pitch = PITCHES[(scene_index + variant) % len(PITCHES)]
            label = f"{scene}_V{variant + 1}_{category}_{color}"
            config["experiment_id"] = f"CI_E7_PERCEPTION_COLLECTION_{label.upper()}"
            config["random_seed"] = 31000 + scene_index * 10 + variant
            config["simulation"]["duration_seconds"] = 30.0
            set_uav_altitude(config, altitude)

            task = config["task"]
            task["task_id"] = f"find_{color}_{category}_{scene.lower()}_{variant + 1}"
            task["instruction"] = f"寻找一辆{color_zh}{category_zh}，并前往其所在位置。"
            task["parsed_goal"] = {
                "target": {"category": category, "attributes": {"color": color}},
                "action": "approach",
            }
            target = config["actors"]["target_vehicle"]
            target["preferred_blueprints"] = [blueprint]
            target["resolved_blueprint"] = blueprint
            target["color_rgb"] = color_rgb
            config["actors"]["distractors"].update({
                "same_category_wrong_color": 1,
                "same_color_wrong_category": 1,
                "random_vehicles": 3,
            })

            sensors = config["sensors"]
            sensors["frequency_hz"] = 10.0
            uav_rgb = sensors["streams"]["uav_rgb"]
            ugv_rgb = sensors["streams"]["ugv_rgb"]
            uav_rgb["pitch_degrees"] = pitch
            sensors["streams"]["uav_instance"] = {
                **copy.deepcopy(uav_rgb), "type": "sensor.camera.instance_segmentation"
            }
            sensors["streams"]["ugv_instance"] = {
                **copy.deepcopy(ugv_rgb), "type": "sensor.camera.instance_segmentation"
            }
            dynamic = config["dynamic"]
            dynamic["save_all_samples"] = False
            dynamic["sensor_save_every_n_frames"] = 3
            dynamic["local_target_search"]["enabled"] = False
            dynamic["persistent_target_communication"]["enabled"] = False
            dynamic["ugv_safety"]["hazard_detector"]["enabled"] = False
            dynamic["sensor_warmup_ticks"] = 20
            dynamic["comparison_distractor_color_rgb"] = None
            dynamic["same_category_wrong_color_rgb"] = wrong_color_rgb
            dynamic["same_color_wrong_category_rgb"] = color_rgb
            dynamic["random_distractor_color_rgb"] = None

            acceptance = config["acceptance"]
            for key in list(acceptance):
                if key.startswith("minimum_"):
                    acceptance[key] = 0
                elif key.startswith("require_"):
                    acceptance[key] = False
            output = config["output"]
            output["root"] = str((args.data_root / scene / f"V{variant + 1}").as_posix())
            output["save_rgb"] = True
            output["save_depth_raw"] = False
            output["save_depth_float_m"] = False
            output["save_depth_colour"] = False
            output["save_instance_segmentation"] = True

            destination = args.output / f"{label}.yaml"
            destination.write_text(
                yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8"
            )
            generated.append(destination)
    manifest = args.output / "manifest.txt"
    manifest.write_text("\n".join(str(path) for path in generated) + "\n", encoding="utf-8")
    print(f"Generated {len(generated)} configs in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

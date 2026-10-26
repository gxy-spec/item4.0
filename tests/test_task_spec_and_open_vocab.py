from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np

from src.perception.attributes import analyze_color
from src.perception.candidate_pipeline import CandidatePipeline, PipelineConfig
from src.perception.instruction_parser import InstructionParser
from src.perception.target_ranker import TargetRanker, TransmissionGateConfig
from src.perception.task_spec import TaskSpec


def task_config(color: str = "red", category: str = "van") -> dict:
    return {
        "task_id": "test",
        "language": "zh-CN",
        "instruction": "寻找目标",
        "parsed_goal": {
            "subcategory": category,
            "attributes": {"color": color},
            "action": "approach",
        },
    }


def test_task_spec_accepts_existing_yaml_and_relation_schema() -> None:
    payload = task_config()
    payload["parsed_goal"]["relations"] = [{
        "type": "beside",
        "reference": {"subcategory": "car", "attributes": {"color": "blue"}},
    }]
    spec = TaskSpec.from_task_config(payload)
    assert spec.target.category == "van"
    assert spec.target_color == "red"
    assert spec.detector_prompts == ("van", "car")
    assert spec.relations[0].reference.color == "blue"


def test_mask_color_measurement_excludes_box_background() -> None:
    image = np.full((80, 120, 3), (255, 0, 0), dtype=np.uint8)  # blue background
    image[20:60, 45:75] = (0, 0, 255)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    mask[20:60, 45:75] = 1
    evidence = analyze_color(image, [10, 10, 110, 70], mask)
    assert evidence.dominant_color == "red"
    assert evidence.ratios["red"] > 0.95
    assert evidence.ratios["blue"] < 0.01


class FakeOpenVocabularyModel:
    def __init__(self) -> None:
        self.classes = []
        self.predict_kwargs = None

    def set_classes(self, classes) -> None:
        self.classes = list(classes)

    def predict(self, source, **kwargs):
        self.predict_kwargs = kwargs
        return []


def test_open_vocabulary_pipeline_sets_task_prompts_without_coco_class_filter() -> None:
    model = FakeOpenVocabularyModel()
    pipeline = CandidatePipeline(
        PipelineConfig(
            model_path=Path("models/yoloe-26s-seg.pt"),
            task_spec=TaskSpec.from_task_config(task_config("blue", "bus")),
            detector_backend="open_vocabulary",
        ),
        model=model,
    )
    pipeline.infer_images([np.zeros((10, 10, 3), dtype=np.uint8)])
    assert model.classes == ["bus"]
    assert "classes" not in model.predict_kwargs
    assert pipeline.detector_proposal_source == "yoloe_26s_seg_open_vocab"


def test_custom_vehicle_classes_do_not_use_coco_class_ids() -> None:
    model = FakeOpenVocabularyModel()
    model.names = {0: "van", 1: "car", 2: "truck", 3: "bus"}
    pipeline = CandidatePipeline(
        PipelineConfig(
            model_path=Path("models/city_vehicles.pt"),
            detector_backend="custom",
            task_spec=TaskSpec.from_task_config(task_config("blue", "bus")),
        ), model=model,
    )
    pipeline.infer_images([np.zeros((10, 10, 3), dtype=np.uint8)])
    assert "classes" not in model.predict_kwargs
    assert model.classes == []
    assert pipeline.detector_proposal_source == "city_vehicles_custom"


def test_pipeline_uses_instance_mask_for_requested_color() -> None:
    image = np.full((100, 100, 3), (255, 0, 0), dtype=np.uint8)
    image[30:70, 40:70] = (0, 0, 255)
    mask = np.zeros((1, 100, 100), dtype=np.uint8)
    mask[0, 30:70, 40:70] = 1
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            xyxy=np.asarray([[10.0, 10.0, 90.0, 90.0]]),
            conf=np.asarray([0.8]),
            cls=np.asarray([0]),
        ),
        masks=SimpleNamespace(data=mask),
        names={0: "van"},
    )
    pipeline = CandidatePipeline(
        PipelineConfig(
            model_path=Path("models/yoloe-26s-seg.pt"),
            image_width=100,
            image_height=100,
            fov_degrees=90,
            task_spec=TaskSpec.from_task_config(task_config()),
            detector_backend="open_vocabulary",
            enable_attribute_proposals=False,
        ),
        model=FakeOpenVocabularyModel(),
    )
    candidates = pipeline.process(
        1,
        0.1,
        image,
        np.full((100, 100), 10.0),
        {"location": [0.0, 0.0, 20.0], "rotation": [-90.0, 0.0, 0.0]},
        result,
    )
    assert len(candidates) == 1
    assert candidates[0]["target_color_match"] is True
    assert candidates[0]["target_color_ratio"] > 0.95
    assert candidates[0]["mask_area_px"] == 1200


def test_ranker_uses_task_color_and_category_instead_of_red_van_constants() -> None:
    spec = TaskSpec.from_task_config(task_config("blue", "bus"))
    ranker = TargetRanker(
        TransmissionGateConfig(allowed_sources=("yoloe_26s_seg_open_vocab",)),
        task_spec=spec,
    )
    candidate = {
        "candidate_id": "1_00",
        "bbox_xyxy": [1.0, 1.0, 20.0, 20.0],
        "category": "bus",
        "proposal_source": "yoloe_26s_seg_open_vocab",
        "color": "blue",
        "color_ratios": {"blue": 0.82, "red": 0.0},
        "detector_score": 0.70,
        "temporal_hits": 4,
        "world_position_xyz": [5.0, 6.0, 1.5],
    }
    selected = ranker.select([candidate])
    assert selected is not None
    assert selected["target_attribute_score"] == 0.82
    assert selected["category_match_score"] == 1.0


def test_instruction_parser_extracts_relational_chinese_target() -> None:
    spec = InstructionParser().parse("寻找停在蓝色轿车旁边的红色厢式货车")
    assert spec.target.category == "van"
    assert spec.target_color == "red"
    assert len(spec.relations) == 1
    assert spec.relations[0].relation == "beside"
    assert spec.relations[0].reference.category == "car"
    assert spec.relations[0].reference.color == "blue"


def test_instruction_parser_extracts_relational_english_target() -> None:
    spec = InstructionParser().parse("Locate the red cargo van beside a blue sedan and approach it.", language="en")
    assert spec.target.category == "van"
    assert spec.target_color == "red"
    assert spec.relations[0].reference.category == "car"
    assert spec.relations[0].reference.color == "blue"


def test_ranker_requires_requested_reference_relation() -> None:
    spec = InstructionParser().parse("寻找蓝色轿车旁边的红色厢式货车")
    ranker = TargetRanker(
        TransmissionGateConfig(
            allowed_sources=("yoloe_26s_seg_open_vocab",),
            minimum_temporal_hits=1,
            minimum_detector_score=0.01,
        ),
        task_spec=spec,
    )
    target = {
        "candidate_id": "target", "bbox_xyxy": [10, 10, 30, 30], "category": "van",
        "proposal_source": "yoloe_26s_seg_open_vocab", "color_ratios": {"red": 0.90},
        "detector_score": 0.80, "temporal_hits": 2, "depth_m": 20.0,
        "world_position_xyz": [10.0, 10.0, 1.5],
    }
    reference = {
        "candidate_id": "reference", "bbox_xyxy": [35, 10, 55, 30], "category": "car",
        "proposal_source": "yoloe_26s_seg_open_vocab", "color_ratios": {"blue": 0.85},
        "detector_score": 0.75, "temporal_hits": 1, "depth_m": 20.5,
        "world_position_xyz": [14.0, 10.0, 1.5],
    }
    selected = ranker.select([target, reference])
    assert selected is not None
    assert selected["candidate_id"] == "target"
    assert selected["relation_match_score"] > 0.5
    assert selected["relation_evidence"][0]["reference_candidate_id"] == "reference"
    assert ranker.select([target]) is None


class FakeJSONBackend:
    def generate_json(self, prompt, images=None):
        return {"target": {"category": "bus", "attributes": {"color": "yellow"}}, "action": "approach"}


def test_instruction_parser_uses_validated_vlm_fallback_for_unseen_noun() -> None:
    spec = InstructionParser(FakeJSONBackend()).parse("找到那个特殊接驳目标")
    assert spec.target.category == "bus"
    assert spec.target_color == "yellow"

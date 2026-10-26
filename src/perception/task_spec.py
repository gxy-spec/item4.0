"""Structured, model-independent representation of an inspection instruction.

The runtime consumes :class:`TaskSpec` instead of reading task-specific strings
such as ``red`` or ``van`` from perception code.  A VLM can populate the same
schema later; deterministic YAML remains the reproducible default.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


_COLOR_ALIASES = {
    "红": "red", "红色": "red", "red": "red",
    "蓝": "blue", "蓝色": "blue", "blue": "blue",
    "绿": "green", "绿色": "green", "green": "green",
    "黄": "yellow", "黄色": "yellow", "yellow": "yellow",
    "橙": "orange", "橙色": "orange", "orange": "orange",
    "紫": "purple", "紫色": "purple", "purple": "purple",
    "白": "white", "白色": "white", "white": "white",
    "黑": "black", "黑色": "black", "black": "black",
    "灰": "gray", "灰色": "gray", "gray": "gray", "grey": "gray",
}

_CATEGORY_ALIASES = {
    "厢式车": "van", "厢式货车": "van", "货运面包车": "van", "van": "van",
    "delivery van": "van", "cargo van": "van", "minivan": "van",
    "轿车": "car", "小汽车": "car", "car": "car", "sedan": "car",
    "卡车": "truck", "货车": "truck", "truck": "truck",
    "公交车": "bus", "巴士": "bus", "bus": "bus",
    "摩托车": "motorcycle", "motorcycle": "motorcycle", "motorbike": "motorcycle",
    "自行车": "bicycle", "bicycle": "bicycle", "bike": "bicycle",
    "行人": "person", "人": "person", "person": "person", "pedestrian": "person",
    "车辆": "vehicle", "汽车": "vehicle", "vehicle": "vehicle",
}

_RELATION_ALIASES = {
    "旁边": "beside", "附近": "near", "靠近": "near",
    "beside": "beside", "near": "near",
    "左边": "left_of", "左侧": "left_of", "left": "left_of", "left_of": "left_of",
    "右边": "right_of", "右侧": "right_of", "right": "right_of", "right_of": "right_of",
    "前面": "in_front_of", "前方": "in_front_of", "front": "in_front_of",
    "in_front_of": "in_front_of",
    "后面": "behind", "后方": "behind", "behind": "behind",
}

_VEHICLE_CATEGORIES = frozenset({"vehicle", "van", "car", "truck", "bus", "motorcycle", "bicycle"})

_PROMPT_VARIANTS = {
    "van": ("van", "delivery van", "cargo van", "minivan", "box truck"),
    "car": ("car", "sedan", "passenger car"),
    "truck": ("truck", "cargo truck", "lorry"),
    "bus": ("bus", "city bus", "coach"),
    "motorcycle": ("motorcycle", "motorbike"),
    "bicycle": ("bicycle", "bike"),
    "person": ("person", "pedestrian"),
    "vehicle": ("vehicle", "car", "van", "truck", "bus"),
}


def _canonical(value: Any, aliases: Mapping[str, str]) -> str | None:
    if value is None:
        return None
    text = str(value).strip().lower()
    if not text:
        return None
    return aliases.get(text, text.replace(" ", "_"))


def canonical_color(value: Any) -> str | None:
    return _canonical(value, _COLOR_ALIASES)


def canonical_category(value: Any) -> str | None:
    return _canonical(value, _CATEGORY_ALIASES)


@dataclass(frozen=True)
class ObjectSpec:
    category: str
    attributes: dict[str, str] = field(default_factory=dict)

    @property
    def color(self) -> str | None:
        return canonical_color(self.attributes.get("color"))

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "ObjectSpec | None":
        if not payload:
            return None
        category = canonical_category(
            payload.get("subcategory") or payload.get("category") or payload.get("object")
        )
        if category is None:
            return None
        attributes = {
            str(key): str(value)
            for key, value in dict(payload.get("attributes") or {}).items()
            if value is not None
        }
        if payload.get("color") is not None and "color" not in attributes:
            attributes["color"] = str(payload["color"])
        color = canonical_color(attributes.get("color"))
        if color is not None:
            attributes["color"] = color
        return cls(category=category, attributes=attributes)


@dataclass(frozen=True)
class SpatialRelationSpec:
    relation: str
    reference: ObjectSpec
    maximum_distance_m: float | None = None

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any] | None) -> "SpatialRelationSpec | None":
        if not payload:
            return None
        relation = _canonical(payload.get("relation") or payload.get("type"), _RELATION_ALIASES)
        reference = ObjectSpec.from_mapping(payload.get("reference"))
        if relation is None or reference is None:
            return None
        distance = payload.get("maximum_distance_m")
        return cls(relation, reference, None if distance is None else float(distance))


@dataclass(frozen=True)
class TaskSpec:
    """Target semantics shared by perception, ranking, communication and VLM."""

    task_id: str
    instruction: str
    target: ObjectSpec
    action: str = "approach"
    relations: tuple[SpatialRelationSpec, ...] = ()
    language: str = "zh-CN"

    @classmethod
    def from_task_config(cls, task: Mapping[str, Any]) -> "TaskSpec":
        goal = dict(task.get("parsed_goal") or task)
        target_payload: dict[str, Any]
        if isinstance(goal.get("target"), Mapping):
            target_payload = dict(goal["target"])
        else:
            target_payload = goal
        target = ObjectSpec.from_mapping(target_payload)
        if target is None:
            raise ValueError("parsed_goal must define target object/category/subcategory")

        raw_relations = goal.get("relations") or ()
        if isinstance(raw_relations, Mapping):
            raw_relations = [raw_relations]
        if not raw_relations and (goal.get("relation") or goal.get("reference")):
            raw_relations = [{"relation": goal.get("relation"), "reference": goal.get("reference")}]
        relations = tuple(
            item for item in (SpatialRelationSpec.from_mapping(value) for value in raw_relations) if item
        )
        return cls(
            task_id=str(task.get("task_id", "inspection_task")),
            instruction=str(task.get("instruction", "")),
            target=target,
            action=str(goal.get("action", "approach")),
            relations=relations,
            language=str(task.get("language", "zh-CN")),
        )

    @classmethod
    def legacy_red_van(cls) -> "TaskSpec":
        return cls(
            task_id="legacy_red_van",
            instruction="find a red van",
            target=ObjectSpec("van", {"color": "red"}),
        )

    @property
    def target_color(self) -> str | None:
        return self.target.color

    @property
    def detector_prompts(self) -> tuple[str, ...]:
        prompts = [self.target.category]
        prompts.extend(relation.reference.category for relation in self.relations)
        return tuple(dict.fromkeys(prompts))

    def category_score(self, detected: str | None, target: ObjectSpec | None = None) -> float:
        expected = (target or self.target).category
        observed_text = "" if detected is None else str(detected).strip().lower().replace("_", " ")
        observed = canonical_category(detected)
        if observed is None:
            return 0.0
        if observed == expected:
            return 1.0
        if observed_text in _PROMPT_VARIANTS.get(expected, ()):
            return 0.95
        if expected in observed_text.split():
            return 0.90
        if expected == "vehicle" and observed in _VEHICLE_CATEGORIES:
            return 0.85
        # COCO has no van class; these priors preserve the old detector as a
        # measurable fallback while open-vocabulary YOLOE can return ``van``.
        if expected == "van" and observed in {"truck", "bus"}:
            return 0.72
        if expected == "van" and observed == "car":
            return 0.58
        return 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": "task_spec_v1",
            "task_id": self.task_id,
            "instruction": self.instruction,
            "language": self.language,
            "action": self.action,
            "target": {"category": self.target.category, "attributes": dict(self.target.attributes)},
            "relations": [
                {
                    "relation": item.relation,
                    "reference": {
                        "category": item.reference.category,
                        "attributes": dict(item.reference.attributes),
                    },
                    "maximum_distance_m": item.maximum_distance_m,
                }
                for item in self.relations
            ],
        }

"""Natural-language task parsing with a deterministic path and optional VLM.

The deterministic parser covers the project's common Chinese/English object,
colour and spatial-relation vocabulary.  A VLM may produce the same JSON
contract for less constrained instructions; its output is always validated by
``TaskSpec`` before it can affect perception or navigation.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import re
from typing import Any, Protocol

from .task_spec import ObjectSpec, SpatialRelationSpec, TaskSpec, canonical_category, canonical_color


class JSONInstructionBackend(Protocol):
    def generate_json(self, prompt: str, images: list[Any] | None = None) -> dict[str, Any]: ...


_COLORS = ("红色", "蓝色", "绿色", "黄色", "橙色", "紫色", "白色", "黑色", "灰色",
           "red", "blue", "green", "yellow", "orange", "purple", "white", "black", "gray", "grey")
_CATEGORIES = ("厢式货车", "货运面包车", "厢式车", "公交车", "摩托车", "自行车", "轿车", "卡车", "货车", "行人", "车辆",
               "delivery van", "cargo van", "minivan", "van", "sedan", "car", "truck", "bus", "motorcycle", "bicycle", "person", "vehicle")
_RELATIONS = {
    "旁边": "beside", "附近": "near", "靠近": "near", "左边": "left_of", "左侧": "left_of",
    "右边": "right_of", "右侧": "right_of", "前面": "in_front_of", "前方": "in_front_of",
    "后面": "behind", "后方": "behind", "beside": "beside", "near": "near", "left of": "left_of",
    "right of": "right_of", "in front of": "in_front_of", "behind": "behind",
}


def _objects_in_text(text: str) -> list[tuple[int, ObjectSpec]]:
    lowered = text.lower()
    category_hits: list[tuple[int, int, str]] = []
    for token in _CATEGORIES:
        start = 0
        while True:
            index = lowered.find(token, start)
            if index < 0:
                break
            category_hits.append((index, index + len(token), token))
            start = index + len(token)
    # Keep the longest category when aliases overlap at the same location.
    filtered: list[tuple[int, int, str]] = []
    for hit in sorted(category_hits, key=lambda item: (item[0], -(item[1] - item[0]))):
        if any(first <= hit[0] and hit[1] <= last for first, last, _ in filtered):
            continue
        filtered.append(hit)
    color_hits = [(lowered.find(token), token) for token in _COLORS if lowered.find(token) >= 0]
    objects: list[tuple[int, ObjectSpec]] = []
    for first, last, token in sorted(filtered):
        preceding = [(position, color) for position, color in color_hits if 0 <= first - position <= 12]
        color = canonical_color(max(preceding, default=(-1, ""), key=lambda item: item[0])[1]) if preceding else None
        attributes = {} if color is None else {"color": color}
        objects.append((first, ObjectSpec(canonical_category(token) or token, attributes)))
    return objects


@dataclass
class InstructionParser:
    backend: JSONInstructionBackend | None = None

    def parse(self, instruction: str, task_id: str = "inspection_task", language: str = "zh-CN") -> TaskSpec:
        if self.backend is not None:
            return self._parse_with_backend(instruction, task_id, language)
        objects = _objects_in_text(instruction)
        relation_hits = [
            (instruction.lower().find(token), canonical)
            for token, canonical in _RELATIONS.items()
            if instruction.lower().find(token) >= 0
        ]
        if objects:
            target_position, target = objects[-1]
            relations: tuple[SpatialRelationSpec, ...] = ()
            if relation_hits and len(objects) >= 2:
                relation_position, relation_name = min(relation_hits, key=lambda item: item[0])
                before = [item for item in objects if item[0] < relation_position]
                after = [item for item in objects if item[0] > relation_position]
                contains_chinese = bool(re.search(r"[\u4e00-\u9fff]", instruction))
                # Chinese attributive form: “蓝车旁边的红车” (reference then
                # target). English imperative form: “red van beside blue car”
                # (target then reference).
                if contains_chinese and before and after:
                    target_position, target = after[-1]
                    reference = before[-1][1]
                    relations = (SpatialRelationSpec(relation_name, reference),)
                elif not contains_chinese and before and after:
                    target_position, target = before[-1]
                    reference = after[0][1]
                    relations = (SpatialRelationSpec(relation_name, reference),)
            return TaskSpec(task_id, instruction, target, "approach", relations, language)
        raise ValueError(f"No supported target object found in instruction: {instruction!r}")

    def _parse_with_backend(self, instruction: str, task_id: str, language: str) -> TaskSpec:
        assert self.backend is not None
        contract = {
            "target": {"category": "van", "attributes": {"color": "red"}},
            "action": "approach",
            "relations": [{
                "relation": "beside",
                "reference": {"category": "car", "attributes": {"color": "blue"}},
            }],
        }
        prompt = (
            "Convert the inspection instruction into strict JSON. Use canonical English object and colour names. "
            "Allowed relations: near, beside, left_of, right_of, in_front_of, behind. Do not infer facts absent "
            f"from the instruction. JSON shape example: {json.dumps(contract, ensure_ascii=False)}\n"
            f"Instruction: {instruction}"
        )
        payload = dict(self.backend.generate_json(prompt))
        payload.update({"task_id": task_id, "instruction": instruction, "language": language})
        return TaskSpec.from_task_config(payload)


def extract_json_object(text: str) -> dict[str, Any]:
    """Extract one JSON object from model output without accepting prose as data."""
    match = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if match is None:
        raise ValueError("VLM response does not contain a JSON object")
    value = json.loads(match.group(0))
    if not isinstance(value, dict):
        raise ValueError("VLM response JSON must be an object")
    return value

"""Low-frequency VLM adapters for instruction parsing and candidate review.

The VLM is deliberately outside the per-frame control loop.  It may parse an
instruction once and review ambiguous crops, while YOLOE plus RGB-D remains the
real-time, reproducible perception path.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from PIL import Image

from .instruction_parser import extract_json_object


@dataclass
class QwenVLBackend:
    model_name: str = "Qwen/Qwen2.5-VL-3B-Instruct"
    device_map: str = "auto"
    max_new_tokens: int = 256

    def __post_init__(self) -> None:
        self._model: Any | None = None
        self._processor: Any | None = None

    def _load(self) -> None:
        if self._model is not None:
            return
        try:
            from transformers import AutoProcessor, Qwen2_5_VLForConditionalGeneration
        except ImportError as exc:
            raise RuntimeError("Install requirements/vlm.txt to enable the optional Qwen VLM backend") from exc
        self._model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
            self.model_name,
            dtype="auto",
            device_map=self.device_map,
        )
        self._processor = AutoProcessor.from_pretrained(self.model_name)

    def generate_json(self, prompt: str, images: list[Any] | None = None) -> dict[str, Any]:
        self._load()
        assert self._model is not None and self._processor is not None
        content: list[dict[str, Any]] = []
        normalized_images: list[Image.Image] = []
        for image in images or []:
            if isinstance(image, Image.Image):
                normalized = image.convert("RGB")
            elif isinstance(image, (str, Path)):
                normalized = Image.open(image).convert("RGB")
            else:
                normalized = Image.fromarray(image).convert("RGB")
            normalized_images.append(normalized)
            content.append({"type": "image", "image": normalized})
        content.append({"type": "text", "text": prompt})
        messages = [{"role": "user", "content": content}]
        rendered = self._processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        inputs = self._processor(
            text=[rendered], images=normalized_images or None, padding=True, return_tensors="pt"
        ).to(self._model.device)
        generated = self._model.generate(**inputs, max_new_tokens=self.max_new_tokens, do_sample=False)
        trimmed = [output[len(source):] for source, output in zip(inputs.input_ids, generated)]
        response = self._processor.batch_decode(trimmed, skip_special_tokens=True)[0]
        return extract_json_object(response)


@dataclass(frozen=True)
class CandidateReview:
    matches_instruction: bool
    confidence: float
    reason: str


class VLMCandidateVerifier:
    """Review an ambiguous detector crop; never supplies metric coordinates."""

    def __init__(self, backend: Any) -> None:
        self.backend = backend

    def verify(self, image: Any, instruction: str) -> CandidateReview:
        prompt = (
            "Judge whether the cropped object satisfies the inspection instruction. Return only JSON with "
            'keys matches_instruction (boolean), confidence (0..1), and reason (short string). '
            f"Instruction: {instruction}"
        )
        result = self.backend.generate_json(prompt, images=[image])
        confidence = min(1.0, max(0.0, float(result.get("confidence", 0.0))))
        return CandidateReview(bool(result.get("matches_instruction", False)), confidence, str(result.get("reason", "")))

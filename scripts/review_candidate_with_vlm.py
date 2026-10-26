#!/usr/bin/env python3
"""Run one auditable, low-frequency VLM review on a detector crop."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from perception.vlm_adapter import QwenVLBackend, VLMCandidateVerifier  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--bbox", nargs=4, type=float, metavar=("X1", "Y1", "X2", "Y2"))
    parser.add_argument("--padding", type=int, default=32)
    parser.add_argument("--model", default="Qwen/Qwen2.5-VL-3B-Instruct")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--crop-output", type=Path)
    args = parser.parse_args()

    image = Image.open(args.image).convert("RGB")
    crop_box = None
    if args.bbox:
        x1, y1, x2, y2 = args.bbox
        crop_box = (
            max(0, int(x1) - args.padding), max(0, int(y1) - args.padding),
            min(image.width, int(x2) + args.padding), min(image.height, int(y2) + args.padding),
        )
        image = image.crop(crop_box)
    if args.crop_output:
        args.crop_output.parent.mkdir(parents=True, exist_ok=True)
        image.save(args.crop_output)
    review = VLMCandidateVerifier(QwenVLBackend(args.model)).verify(image, args.instruction)
    result = {
        "schema_version": "vlm_candidate_review_v1",
        "model": args.model,
        "source_image": str(args.image),
        "crop_xyxy": crop_box,
        "instruction": args.instruction,
        "matches_instruction": review.matches_instruction,
        "confidence": review.confidence,
        "reason": review.reason,
        "usage": "low_frequency_review_only; never a metric localization source",
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

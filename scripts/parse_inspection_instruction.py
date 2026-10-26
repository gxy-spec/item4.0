#!/usr/bin/env python3
"""Compile a natural-language inspection instruction into TaskSpec JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from perception.instruction_parser import InstructionParser  # noqa: E402
from perception.vlm_adapter import QwenVLBackend  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("instruction")
    parser.add_argument("--task-id", default="inspection_task")
    parser.add_argument("--language", default="zh-CN")
    parser.add_argument("--backend", choices=("deterministic", "qwen"), default="deterministic")
    parser.add_argument("--model", default="Qwen/Qwen2.5-VL-3B-Instruct")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    backend = QwenVLBackend(args.model) if args.backend == "qwen" else None
    result = InstructionParser(backend).parse(args.instruction, args.task_id, args.language).to_dict()
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

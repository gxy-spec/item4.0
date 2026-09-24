from __future__ import annotations

import sys
from pathlib import Path


SCRIPTS_ROOT = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from build_s1_batch_summary import build_batch_summary, main


def row(status: str, storage_gb: float = 0.5) -> dict:
    return {"status": status, "storage_gb": storage_gb}


def test_batch_summary_passes_only_when_every_planned_episode_passes(tmp_path: Path) -> None:
    summary = build_batch_summary(
        [row("PASS"), row("PASS")], tmp_path / "analysis", episodes_planned=2
    )
    assert summary["batch_complete"] is True
    assert summary["all_passed"] is True
    assert summary["episodes_passed"] == 2


def test_batch_summary_rejects_missing_episode(tmp_path: Path) -> None:
    summary = build_batch_summary(
        [row("PASS")], tmp_path / "analysis", episodes_planned=2
    )
    assert summary["batch_complete"] is False
    assert summary["all_passed"] is False


def test_batch_summary_rejects_scientific_failure(tmp_path: Path) -> None:
    summary = build_batch_summary(
        [row("PASS"), row("FAIL")], tmp_path / "analysis", episodes_planned=2
    )
    assert summary["batch_complete"] is True
    assert summary["all_passed"] is False
    assert summary["episodes_failed"] == 1


def test_batch_summary_requires_frozen_planned_count(tmp_path: Path) -> None:
    summary = build_batch_summary(
        [row("PASS")], tmp_path / "analysis", episodes_planned=None
    )
    assert summary["batch_complete"] is False
    assert summary["all_passed"] is False


def test_batch_summary_cli_returns_failure_for_incomplete_batch(
    tmp_path: Path, monkeypatch
) -> None:
    (tmp_path / "episodes").mkdir()
    (tmp_path / "batch_progress.json").write_text(
        '{"planned_episodes": 2}', encoding="utf-8"
    )
    monkeypatch.setattr(
        sys, "argv", ["build_s1_batch_summary.py", "--batch-dir", str(tmp_path)]
    )
    assert main() == 2

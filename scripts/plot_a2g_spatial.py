#!/usr/bin/env python3
"""Regenerate publication-sized A2G spatial figures from a saved trace."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from communication.a2g_spatial_plot import plot_publication_figures, read_trace  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    files = plot_publication_figures(
        read_trace(args.run_dir / "communication" / "spatial_link_trace.csv"),
        args.run_dir / "preview",
    )
    for path in files:
        print(path)


if __name__ == "__main__":
    main()

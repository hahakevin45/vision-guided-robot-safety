#!/usr/bin/env python3
"""Generate the fixed centered occupancy map used by the Pi goal bench."""
from __future__ import annotations

import argparse
from pathlib import Path


WIDTH = 100
HEIGHT = 100
RESOLUTION = 0.05
OCCUPIED = 0
FREE = 254


def generate(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for y in range(HEIGHT):
        row = [
            OCCUPIED if x in (0, WIDTH - 1) or y in (0, HEIGHT - 1) else FREE
            for x in range(WIDTH)
        ]
        rows.append(" ".join(str(value) for value in row))
    pgm = "P2\n100 100\n255\n" + "\n".join(rows) + "\n"
    (output_dir / "vgr_5x5_center.pgm").write_text(pgm, encoding="ascii")
    metadata = """image: vgr_5x5_center.pgm
resolution: 0.05
origin: [-2.5, -2.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
mode: trinary
"""
    (output_dir / "vgr_5x5_center.yaml").write_text(metadata, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    generate(args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

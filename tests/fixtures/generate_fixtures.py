#!/usr/bin/env python3
"""Generate test PGM fixtures for CheckerboardDetector.

Run once; the outputs are checked into the repo. Python stdlib only, so this
does not add any runtime dependency to the test build.

Outputs next to this script:
  * checkerboard_8x11.pgm — a visible 12x9 square checkerboard (11x8 inner
    corners, matching the default CheckerboardDetector(11, 8)).
  * no_checkerboard.pgm   — uniform mid-gray, no pattern.
"""
from __future__ import annotations

import os
import pathlib


def write_pgm(path: pathlib.Path, pixels: bytearray, width: int, height: int) -> None:
    assert len(pixels) == width * height
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    path.write_bytes(header + bytes(pixels))


def make_checkerboard(cols_squares: int, rows_squares: int,
                       square_px: int, border_px: int) -> tuple[bytearray, int, int]:
    width = cols_squares * square_px + 2 * border_px
    height = rows_squares * square_px + 2 * border_px
    px = bytearray(b"\xFF" * (width * height))  # white background (incl. border)
    for row in range(rows_squares):
        for col in range(cols_squares):
            if (row + col) % 2 == 1:  # black squares
                y0 = border_px + row * square_px
                x0 = border_px + col * square_px
                for y in range(y0, y0 + square_px):
                    start = y * width + x0
                    px[start:start + square_px] = b"\x00" * square_px
    return px, width, height


def make_uniform(width: int, height: int, value: int = 128) -> bytearray:
    return bytearray([value & 0xFF]) * (width * height)


def main() -> None:
    here = pathlib.Path(__file__).resolve().parent

    # 12x9 squares -> 11x8 inner corners, matches default board_width=11,
    # board_height=8 in CheckerboardDetector.
    board, w, h = make_checkerboard(cols_squares=12, rows_squares=9,
                                    square_px=24, border_px=24)
    write_pgm(here / "checkerboard_8x11.pgm", board, w, h)

    uniform = make_uniform(w, h, 128)
    write_pgm(here / "no_checkerboard.pgm", uniform, w, h)

    print(f"Wrote {w}x{h} fixtures to {here}")


if __name__ == "__main__":
    main()

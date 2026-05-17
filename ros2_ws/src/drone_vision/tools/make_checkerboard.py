#!/usr/bin/env python3
"""
Generate a print-exact checkerboard PDF for camera calibration.

Pairs with calibrate_camera.py: the board here is 10x7 squares =
**9x6 inner corners**, so it is calibrated with `--cols 9 --rows 6`.

A4 landscape, square size fixed in millimetres. The PDF carries no
page scaling, so printing at 100% / "actual size" reproduces the
squares at exactly --square-mm. A 50 mm ruler check is printed on the
sheet so any printer scaling is caught before it corrupts the
calibration (use the *measured* square size for --square-mm if the
check is off).
"""
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

MM_PER_IN = 25.4
A4_W_MM, A4_H_MM = 297.0, 210.0   # landscape


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9,
                    help="inner corners across (board has cols+1 squares)")
    ap.add_argument("--rows", type=int, default=6,
                    help="inner corners down (board has rows+1 squares)")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="square edge length in millimetres")
    ap.add_argument("--out", default="calibration_target_A4.pdf")
    args = ap.parse_args()

    nx, ny = args.cols + 1, args.rows + 1          # squares
    s = args.square_mm
    bw, bh = nx * s, ny * s                         # board size mm
    if bw > A4_W_MM or bh > A4_H_MM:
        raise SystemExit(
            f"Board {bw:.0f}x{bh:.0f} mm exceeds A4 landscape "
            f"({A4_W_MM:.0f}x{A4_H_MM:.0f}). Lower --square-mm.")
    x0 = (A4_W_MM - bw) / 2.0
    y0 = (A4_H_MM - bh) / 2.0

    fig = plt.figure(figsize=(A4_W_MM / MM_PER_IN, A4_H_MM / MM_PER_IN))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, A4_W_MM)
    ax.set_ylim(0, A4_H_MM)
    ax.set_aspect("equal")
    ax.axis("off")

    # Checkerboard (top-left square black).
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle(
                    (x0 + i * s, y0 + j * s), s, s,
                    facecolor="black", edgecolor="none"))

    # Header — kept near the page edge, well clear of the quiet zone.
    ax.text(A4_W_MM / 2, A4_H_MM - 6,
            f"Camera calibration target  —  {args.cols}x{args.rows} inner "
            f"corners  ({nx}x{ny} squares)  —  square = {s:.1f} mm",
            ha="center", va="center", fontsize=8)
    ax.text(A4_W_MM / 2, A4_H_MM - 12,
            "PRINT AT 100% / ACTUAL SIZE  —  disable 'Fit to page' / "
            "'Shrink to fit' / scaling",
            ha="center", va="center", fontsize=8, weight="bold")

    # 50 mm scale bar (bottom-left) for a post-print ruler check.
    bar = 50.0
    bx, by = 12.0, 8.0
    ax.plot([bx, bx + bar], [by, by], color="black", lw=1.2)
    for xx in (bx, bx + bar):
        ax.plot([xx, xx], [by - 1.5, by + 1.5], color="black", lw=1.2)
    ax.text(bx + bar + 4, by,
            "50 mm — measure after printing; if not exactly 50 mm use the "
            "MEASURED square size for --square-mm",
            ha="left", va="center", fontsize=7)

    ax.text(A4_W_MM - 12, 8.0,
            f"calibrate_camera.py --cols {args.cols} --rows {args.rows} "
            f"--square-mm {s:.1f}",
            ha="right", va="center", fontsize=7, family="monospace")

    fig.savefig(args.out)
    # PNG preview alongside the PDF (not for printing — for screen check).
    fig.savefig(args.out.rsplit(".", 1)[0] + "_preview.png", dpi=150)
    print(f"wrote {args.out}  (board {bw:.1f}x{bh:.1f} mm, "
          f"{nx}x{ny} squares @ {s:.1f} mm, {args.cols}x{args.rows} corners)")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
make_charuco.py — print-ready ChArUco board for tools/build/intrinsics and
tools/build/homography.

    python3 make_charuco.py                      # A3, 10x7 squares of 36 mm
    python3 make_charuco.py --paper a4 --squares-x 8 --squares-y 6 --square-mm 25
    python3 make_charuco.py --cam-height-mm 1500 # check it is detectable from there

Writes charuco_board.pdf (print at 100% / "actual size"), charuco_board.png and
charuco_board.json (the geometry the C++ tools read) next to this script, or
into --out-dir.

Uses DICT_4X4_50 — the dictionary the tracker itself runs (aruco_dict 0). Few
cells per marker means each cell is large in the image, so the board stays
detectable from far away, which is the point of this board. Its ids 0..N-1 are
reused by the robots' markers, but the board only exists during calibration
and the tools below look for it in a separate step.

Sizes are snapped so every square and marker is a whole number of pixels
(marker a multiple of its 6 cells): fractional edges blur and bias the corner
positions the calibration is built on. The PDF's resolution is then set so the
printed size is exactly the nominal square size. After printing, measure the
squares with calipers over a long span and pass that to the tools with
--square-mm; the board is only as accurate as the print.

Dependencies:  pip install opencv-contrib-python numpy pillow
"""

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

DICTS = {
    "4x4_50":   cv2.aruco.DICT_4X4_50,
    "4x4_100":  cv2.aruco.DICT_4X4_100,
    "5x5_100":  cv2.aruco.DICT_5X5_100,
    "6x6_250":  cv2.aruco.DICT_6X6_250,
}
DICT_BITS = {"4x4_50": 4, "4x4_100": 4, "5x5_100": 5, "6x6_250": 6}
DICT_SIZE = {"4x4_50": 50, "4x4_100": 100, "5x5_100": 100, "6x6_250": 250}
PAPER_MM = {"a4": (297.0, 210.0), "a3": (420.0, 297.0)}   # landscape


def build_board(sx, sy, square_px, marker_px, dict_name):
    d = cv2.aruco.getPredefinedDictionary(DICTS[dict_name])
    # Lengths only set the ratio here; render in pixels, publish in mm later.
    return cv2.aruco.CharucoBoard((sx, sy), float(square_px), float(marker_px), d)


def detect_charuco(img, board):
    det = cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = det.detectBoard(img)
    return 0 if ids is None else len(ids)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--squares-x", type=int, default=10)
    ap.add_argument("--squares-y", type=int, default=7)
    ap.add_argument("--square-mm", type=float, default=36.0)
    ap.add_argument("--marker-ratio", type=float, default=0.75,
                    help="marker side / square side (default 0.75)")
    ap.add_argument("--dict", default="4x4_50", choices=sorted(DICTS))
    ap.add_argument("--paper", default="a3", choices=sorted(PAPER_MM))
    ap.add_argument("--dpi", type=int, default=300, help="render resolution (default 300)")
    ap.add_argument("--out-dir", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--name", default="charuco_board")
    # Detectability check: which camera is this going to be held up to?
    ap.add_argument("--focal-mm", type=float, default=6.0)
    ap.add_argument("--pixel-um", type=float, default=2.2)
    ap.add_argument("--cam-height-mm", type=float, default=1200.0,
                    help="working distance used for the detectability check")
    args = ap.parse_args()

    sx, sy = args.squares_x, args.squares_y
    n_markers = (sx * sy) // 2
    if n_markers > DICT_SIZE[args.dict]:
        sys.exit(f"{sx}x{sy} needs {n_markers} markers but {args.dict} only has "
                 f"{DICT_SIZE[args.dict]}; use fewer squares or a bigger dictionary.")

    # ── Snap to whole pixels ──────────────────────────────────────────────
    cells = DICT_BITS[args.dict] + 2                       # data bits + 1-cell border each side
    px_per_mm = args.dpi / 25.4
    square_px = int(round(args.square_mm * px_per_mm))
    square_px += square_px % 2                             # even, so the marker centres exactly
    marker_px = cells * max(1, int(round(square_px * args.marker_ratio / cells)))
    if (square_px - marker_px) % 2:
        marker_px -= cells                                 # keep (square-marker) even
    if marker_px <= 0 or marker_px >= square_px:
        sys.exit("marker ratio leaves no room for the marker or its quiet zone")
    dpi_eff = square_px * 25.4 / args.square_mm            # makes the print exactly square_mm
    marker_mm = marker_px / square_px * args.square_mm

    board_w_mm, board_h_mm = sx * args.square_mm, sy * args.square_mm
    page_w_mm, page_h_mm = PAPER_MM[args.paper]
    foot_mm = 22.0                                         # caption + scale bar
    if board_w_mm + 20 > page_w_mm or board_h_mm + 20 + foot_mm > page_h_mm:
        sys.exit(f"board {board_w_mm:.0f}x{board_h_mm:.0f} mm does not fit {args.paper.upper()} "
                 f"with margins; use fewer/smaller squares or a larger paper")

    board = build_board(sx, sy, square_px, marker_px, args.dict)
    board_img = board.generateImage((sx * square_px, sy * square_px), marginSize=0, borderBits=1)

    # ── Self-check: does OpenCV find every inner corner? ──────────────────
    expected = (sx - 1) * (sy - 1)
    found = detect_charuco(board_img, board)
    if found != expected:
        sys.exit(f"self-check failed: detected {found}/{expected} corners on the clean render")

    # Detectability at the working distance: shrink the render to the pixel
    # density the camera will see and blur slightly, like a real lens does.
    px_per_mm_cam = (args.focal_mm * 1000.0 / args.pixel_um) / args.cam_height_mm
    scale = px_per_mm_cam / (square_px / args.square_mm)
    small = cv2.resize(board_img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    small = cv2.GaussianBlur(small, (0, 0), 0.8)
    found_far = detect_charuco(small, board)
    cell_px = marker_px / cells * scale
    print(f"board {sx}x{sy}  square {args.square_mm:.2f} mm  marker {marker_mm:.2f} mm  "
          f"{args.dict}  ({n_markers} markers)")
    print(f"at {args.cam_height_mm:.0f} mm: {px_per_mm_cam:.2f} px/mm, "
          f"square {args.square_mm * px_per_mm_cam:.0f} px, marker cell {cell_px:.1f} px "
          f"-> {found_far}/{expected} corners detected on a simulated view")
    if found_far < expected or cell_px < 4.0:
        print("WARNING: marginal at that distance. Use bigger squares (--square-mm), "
              "or hold the board closer.", file=sys.stderr)

    # ── Compose the page at the exact physical size ───────────────────────
    page_w = int(round(page_w_mm / 25.4 * dpi_eff))
    page_h = int(round(page_h_mm / 25.4 * dpi_eff))
    page = Image.new("L", (page_w, page_h), 255)
    bw, bh = sx * square_px, sy * square_px
    top = int(round(10.0 / 25.4 * dpi_eff))
    page.paste(Image.fromarray(board_img), ((page_w - bw) // 2, top))

    d = ImageDraw.Draw(page)
    try:
        font = ImageFont.load_default(size=int(3.2 / 25.4 * dpi_eff))
    except TypeError:                                      # Pillow < 10.1
        font = ImageFont.load_default()
    y = top + bh + int(4.0 / 25.4 * dpi_eff)
    d.text(((page_w - bw) // 2, y),
           f"ChArUco {sx}x{sy}  {args.dict}  square {args.square_mm:g} mm  "
           f"marker {marker_mm:.2f} mm  -  print at 100% / actual size, mount FLAT",
           fill=0, font=font)
    # 100 mm scale bar: hold a ruler to it to prove the print was not rescaled.
    by = y + int(9.0 / 25.4 * dpi_eff)
    bx = (page_w - bw) // 2
    L = int(round(100.0 / 25.4 * dpi_eff))
    t = max(2, int(0.6 / 25.4 * dpi_eff))
    d.rectangle([bx, by, bx + L, by + t], fill=0)
    for k in range(11):
        xx = bx + int(round(k * L / 10))
        d.rectangle([xx - t // 2, by - (t * 2 if k % 5 == 0 else t), xx + t // 2, by], fill=0)
    d.text((bx + L + int(3.0 / 25.4 * dpi_eff), by - int(2.0 / 25.4 * dpi_eff)),
           "100 mm", fill=0, font=font)

    os.makedirs(args.out_dir, exist_ok=True)
    base = os.path.join(args.out_dir, args.name)
    page.save(base + ".pdf", resolution=dpi_eff)
    page.save(base + ".png", dpi=(dpi_eff, dpi_eff))
    with open(base + ".json", "w") as f:
        json.dump({
            "dict": args.dict,
            "dict_id": int(DICTS[args.dict]),
            "squares_x": sx,
            "squares_y": sy,
            "square_mm": args.square_mm,
            "marker_mm": round(marker_mm, 4),
        }, f, indent=2)
        f.write("\n")
    print(f"wrote {base}.pdf / .png / .json  (print {page_w_mm:.0f}x{page_h_mm:.0f} mm, "
          f"{dpi_eff:.2f} dpi)")
    print("Print at 100%, mount on something rigid and flat, then check the 100 mm bar with a ruler.")


if __name__ == "__main__":
    main()

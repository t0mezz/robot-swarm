"""
generate_markers.py — print ArUco or AprilTag marker sheets as a PDF.

Usage:
    # ArUco (default)
    python generate_markers.py --n 14 --size 5.0 --output markers.pdf
    python generate_markers.py --n 50 --size 3.5 --dict 6x6_250 --output aruco.pdf

    # AprilTag
    python generate_markers.py --n 10 --size 5.0 --family 36h11 --output april.pdf
    python generate_markers.py --n 5  --size 6.0 --family 25h9  --output april.pdf

Arguments:
    --n        Number of markers to generate (IDs 0 … n-1)
    --size     Marker side length in centimetres
    --margin   Page margin in centimetres (default: 1.0)
    --output   Output PDF path (default: markers/markers.pdf)

    ArUco options (mutually exclusive with --family):
    --dict     ArUco dictionary name: 4x4_50 (default), 4x4_100, 5x5_50, 6x6_250, 7x7_1000

    AprilTag options (mutually exclusive with --dict):
    --family   AprilTag family: 16h5, 25h9, 36h10, 36h11 (default for AprilTag mode)

    When neither --dict nor --family is given, ArUco DICT_4X4_50 is used.

Dependencies:
    pip install opencv-python numpy reportlab
"""

import argparse
import io
import sys

import cv2
import numpy as np
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas


MARKER_PX      = 400   # render size in pixels — high enough for clean print
LABEL_FONT_SIZE = 7    # pt

# ─── Dictionary tables ────────────────────────────────────────────────────────

ARUCO_DICTS = {
    "4x4_50":    (cv2.aruco.DICT_4X4_50,    50,   "ArUco 4×4_50"),
    "4x4_100":   (cv2.aruco.DICT_4X4_100,   100,  "ArUco 4×4_100"),
    "5x5_50":    (cv2.aruco.DICT_5X5_50,    50,   "ArUco 5×5_50"),
    "6x6_250":   (cv2.aruco.DICT_6X6_250,   250,  "ArUco 6×6_250"),
    "7x7_1000":  (cv2.aruco.DICT_7X7_1000,  1000, "ArUco 7×7_1000"),
}

APRILTAG_DICTS = {
    "16h5":  (cv2.aruco.DICT_APRILTAG_16h5,  30,   "AprilTag 16h5"),
    "25h9":  (cv2.aruco.DICT_APRILTAG_25h9,  35,   "AprilTag 25h9"),
    "36h10": (cv2.aruco.DICT_APRILTAG_36h10, 2320, "AprilTag 36h10"),
    "36h11": (cv2.aruco.DICT_APRILTAG_36h11, 587,  "AprilTag 36h11"),
}


def _resolve_dict(dict_name: str | None, family: str | None):
    """Return (cv2_dict_id, max_n, label_prefix) from CLI arguments."""
    if family is not None and dict_name is not None:
        raise ValueError("--dict and --family are mutually exclusive")

    if family is not None:
        key = family.lower()
        if key not in APRILTAG_DICTS:
            raise ValueError(
                f"Unknown AprilTag family '{family}'. "
                f"Choose from: {', '.join(APRILTAG_DICTS)}"
            )
        return APRILTAG_DICTS[key]

    # ArUco path (default or explicit --dict)
    key = (dict_name or "4x4_50").lower()
    if key not in ARUCO_DICTS:
        raise ValueError(
            f"Unknown ArUco dictionary '{dict_name}'. "
            f"Choose from: {', '.join(ARUCO_DICTS)}"
        )
    return ARUCO_DICTS[key]


def _marker_image(cv_dict_id: int, marker_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv_dict_id)
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    return img


def _ndarray_to_png_bytes(img: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise RuntimeError("Failed to encode marker image")
    return bytes(buf)


def generate_pdf(
    n: int,
    size_cm: float,
    margin_cm: float,
    output_path: str,
    cv_dict_id: int,
    max_n: int,
    label_prefix: str,
) -> None:
    if n < 1:
        raise ValueError("n must be >= 1")
    if n > max_n:
        raise ValueError(f"{label_prefix} only supports IDs 0–{max_n - 1}")
    if size_cm <= 0:
        raise ValueError("size must be > 0")

    page_w, page_h = A4
    margin   = margin_cm * cm
    size     = size_cm * cm
    label_gap = 0.35 * cm

    usable_w = page_w - 2 * margin
    usable_h = page_h - 2 * margin

    cols = max(1, int(usable_w // (size + label_gap)))
    row_h = size + label_gap + LABEL_FONT_SIZE + 2
    rows_per_page = max(1, int(usable_h // row_h))

    c = canvas.Canvas(output_path, pagesize=A4)
    c.setTitle(f"{label_prefix} — {n} markers @ {size_cm} cm")

    for idx in range(n):
        page_pos = idx % (cols * rows_per_page)
        col = page_pos % cols
        row = page_pos // cols

        if idx > 0 and page_pos == 0:
            c.showPage()

        x = margin + col * (usable_w / cols) + (usable_w / cols - size) / 2
        y = page_h - margin - (row + 1) * row_h + label_gap + LABEL_FONT_SIZE + 2

        img_bytes = _ndarray_to_png_bytes(_marker_image(cv_dict_id, idx))
        c.drawImage(
            ImageReader(io.BytesIO(img_bytes)),
            x, y, width=size, height=size,
            preserveAspectRatio=True,
        )

        c.setFont("Helvetica", LABEL_FONT_SIZE)
        label = f"ID {idx}"
        label_w = c.stringWidth(label, "Helvetica", LABEL_FONT_SIZE)
        c.drawString(x + (size - label_w) / 2, y - label_gap - LABEL_FONT_SIZE, label)

    c.save()
    pages = (n + cols * rows_per_page - 1) // (cols * rows_per_page)
    print(
        f"Saved {n} {label_prefix} markers (IDs 0–{n-1}) to '{output_path}'\n"
        f"  Layout : {cols} col × {rows_per_page} row per page, {pages} page(s)\n"
        f"  Marker : {size_cm} cm,  margin: {margin_cm} cm"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate ArUco or AprilTag marker PDF",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  ArUco (default):    python generate_markers.py --n 14 --size 5\n"
            "  AprilTag 36h11:     python generate_markers.py --n 10 --size 5 --family 36h11\n"
            "  AprilTag 16h5:      python generate_markers.py --n 5  --size 6 --family 16h5\n"
            "  ArUco 4x4_50:       python generate_markers.py --n 10 --size 5 --dict 4x4_50\n"
        ),
    )
    parser.add_argument("--n",      type=int,   required=True,         help="Number of markers (IDs 0…n-1)")
    parser.add_argument("--size",   type=float, required=True,         help="Marker side length in cm")
    parser.add_argument("--margin", type=float, default=1.0,           help="Page margin in cm (default: 1.0)")
    parser.add_argument("--output", type=str,   default="markers/markers.pdf", help="Output PDF filename")
    parser.add_argument("--dict",   type=str,   default=None,
                        metavar="DICT",
                        help="ArUco dict: 4x4_50 (default), 4x4_100, 5x5_50, 6x6_250, 7x7_1000")
    parser.add_argument("--family", type=str,   default=None,
                        metavar="FAMILY",
                        help="AprilTag family: 16h5, 25h9, 36h10, 36h11")
    args = parser.parse_args()

    try:
        cv_dict_id, max_n, label_prefix = _resolve_dict(args.dict, args.family)
        generate_pdf(args.n, args.size, args.margin, args.output,
                     cv_dict_id, max_n, label_prefix)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

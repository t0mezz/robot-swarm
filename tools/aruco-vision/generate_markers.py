"""
generate_markers.py — print ArUco marker sheets as a PDF.

Usage:
    python generate_markers.py --n 14 --size 5.0 --output markers.pdf
    python generate_markers.py --n 50 --size 3.5 --margin 1.0 --output markers.pdf

Arguments:
    --n        Number of markers to generate (IDs 0 … n-1)
    --size     Marker side length in centimetres
    --margin   Page margin in centimetres (default: 1.0)
    --output   Output PDF path (default: markers.pdf)

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


DICTIONARY = cv2.aruco.DICT_6X6_250
MARKER_PX = 400          # render size in pixels — high enough for clean print
LABEL_FONT_SIZE = 7      # pt


def _marker_image(marker_id: int) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    img = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    return img


def _ndarray_to_png_bytes(img: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise RuntimeError("Failed to encode marker image")
    return bytes(buf)


def generate_pdf(n: int, size_cm: float, margin_cm: float, output_path: str) -> None:
    if n < 1:
        raise ValueError("n must be >= 1")
    if n > 250:
        raise ValueError("DICT_6X6_250 only supports IDs 0–249")
    if size_cm <= 0:
        raise ValueError("size must be > 0")

    page_w, page_h = A4          # points (1 pt = 1/72 inch)
    margin = margin_cm * cm
    size = size_cm * cm
    label_gap = 0.35 * cm        # space between marker bottom and ID label

    usable_w = page_w - 2 * margin
    usable_h = page_h - 2 * margin

    cols = max(1, int(usable_w // (size + label_gap)))
    # row height = marker + label line
    row_h = size + label_gap + LABEL_FONT_SIZE + 2
    rows_per_page = max(1, int(usable_h // row_h))

    c = canvas.Canvas(output_path, pagesize=A4)
    c.setTitle(f"ArUco DICT_6X6_250 — {n} markers @ {size_cm} cm")

    for idx in range(n):
        page_pos = idx % (cols * rows_per_page)
        col = page_pos % cols
        row = page_pos // cols

        if idx > 0 and page_pos == 0:
            c.showPage()

        # ReportLab origin is bottom-left; we work from top-left.
        x = margin + col * (usable_w / cols) + (usable_w / cols - size) / 2
        y = page_h - margin - (row + 1) * row_h + label_gap + LABEL_FONT_SIZE + 2

        img_bytes = _ndarray_to_png_bytes(_marker_image(idx))
        c.drawImage(
            ImageReader(io.BytesIO(img_bytes)),
            x, y, width=size, height=size,
            preserveAspectRatio=True,
        )

        # ID label centred below the marker
        c.setFont("Helvetica", LABEL_FONT_SIZE)
        label = f"ID {idx}"
        label_w = c.stringWidth(label, "Helvetica", LABEL_FONT_SIZE)
        c.drawString(x + (size - label_w) / 2, y - label_gap - LABEL_FONT_SIZE, label)

    c.save()
    pages = (n + cols * rows_per_page - 1) // (cols * rows_per_page)
    print(
        f"Saved {n} markers (IDs 0–{n-1}) to '{output_path}'\n"
        f"  Layout : {cols} col × {rows_per_page} row per page, {pages} page(s)\n"
        f"  Marker : {size_cm} cm,  margin: {margin_cm} cm"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate ArUco marker PDF")
    parser.add_argument("--n", type=int, required=True, help="Number of markers (IDs 0…n-1)")
    parser.add_argument("--size", type=float, required=True, help="Marker side length in cm")
    parser.add_argument("--margin", type=float, default=1.0, help="Page margin in cm (default: 1.0)")
    parser.add_argument("--output", type=str, default="markers.pdf", help="Output PDF filename")
    args = parser.parse_args()

    try:
        generate_pdf(args.n, args.size, args.margin, args.output)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()

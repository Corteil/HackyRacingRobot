#!/usr/bin/env python3
"""
generate_aruco_tags.py — Generate ArUco tags as a PDF (one per A4 page).

Usage
-----
  python3 generate_aruco_tags.py                  # IDs 1–4 (default)
  python3 generate_aruco_tags.py 1 2 3 4 5 6      # specific IDs
  python3 generate_aruco_tags.py --dict 6X6_100 7 8

Options
-------
  IDs            Tag IDs to generate (positional, space-separated)
  --dict NAME    ArUco dictionary  [default: 4X4_50]
                 Short form: 4X4_50, 4X4_100, 5X5_100, 6X6_100, …
  --size MM      Printed tag size in mm  [default: 160]
  --out PATH     Output PDF path  [default: docs/aruco_tags_<ids>.pdf]

Output: docs/aruco_tags_<ids>.pdf
"""

import argparse
import os
import sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ── Known dictionaries ────────────────────────────────────────────────────────

_DICTS = {
    "4X4_50":   cv2.aruco.DICT_4X4_50,
    "4X4_100":  cv2.aruco.DICT_4X4_100,
    "4X4_250":  cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50":   cv2.aruco.DICT_5X5_50,
    "5X5_100":  cv2.aruco.DICT_5X5_100,
    "5X5_250":  cv2.aruco.DICT_5X5_250,
    "6X6_50":   cv2.aruco.DICT_6X6_50,
    "6X6_100":  cv2.aruco.DICT_6X6_100,
    "7X7_50":   cv2.aruco.DICT_7X7_50,
}

# ── Paper sizes (width × height in mm, portrait) ─────────────────────────────

_PAPERS = {
    "A3":     (297, 420),
    "A4":     (210, 297),
    "A5":     (148, 210),
    "LETTER": (216, 279),
    "LEGAL":  (216, 356),
    "HALF":   (216, 140),   # half-letter / index card
}

# ── Page constants ────────────────────────────────────────────────────────────

DPI       = 300
LABEL_GAP = 8    # mm between tag bottom and label


def _mm(mm: float) -> int:
    return int(round(mm / 25.4 * DPI))

_FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def _make_page(tag_id: int, dictionary, dict_name: str, tag_px: int,
               font, page_w: int, page_h: int) -> Image.Image:
    """Return a 300 DPI PIL Image containing one ArUco tag."""
    marker = np.zeros((tag_px, tag_px), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, tag_id, tag_px, marker, 1)

    page    = Image.fromarray(np.full((page_h, page_w), 255, dtype=np.uint8))
    tag_img = Image.fromarray(marker)

    x = (page_w - tag_px) // 2
    y = (page_h - tag_px) // 2 - _mm(LABEL_GAP + 5)
    page.paste(tag_img, (x, y))

    draw  = ImageDraw.Draw(page)
    label = f"ArUco  {dict_name}  ID {tag_id}"
    bbox  = draw.textbbox((0, 0), label, font=font)
    tw    = bbox[2] - bbox[0]
    draw.text(((page_w - tw) // 2, y + tag_px + _mm(LABEL_GAP)),
              label, fill=0, font=font)

    return page


def main():
    parser = argparse.ArgumentParser(
        description="Generate ArUco tags as a PDF (one per A4 page).")
    parser.add_argument("ids", nargs="*", type=int,
                        help="Tag IDs to generate (default: 1 2 3 4)")
    parser.add_argument("--dict", default="4X4_50", metavar="NAME",
                        help=f"ArUco dictionary (default: 4X4_50). "
                             f"Options: {', '.join(_DICTS)}")
    parser.add_argument("--paper", default="A4", metavar="SIZE",
                        help=f"Paper size (default: A4). "
                             f"Options: {', '.join(_PAPERS)}")
    parser.add_argument("--size", default=None, type=int, metavar="MM",
                        help="Printed tag size in mm (default: 80%% of page width)")
    parser.add_argument("--out", default=None, metavar="PATH",
                        help="Output PDF path (default: docs/aruco_tags_<ids>.pdf)")
    args = parser.parse_args()

    tag_ids   = args.ids if args.ids else [1, 2, 3, 4]
    dict_name = args.dict.upper()

    if dict_name not in _DICTS:
        sys.exit(f"Unknown dictionary {dict_name!r}. Options: {', '.join(_DICTS)}")

    paper_name = args.paper.upper()
    if paper_name not in _PAPERS:
        sys.exit(f"Unknown paper size {paper_name!r}. Options: {', '.join(_PAPERS)}")

    paper_w_mm, paper_h_mm = _PAPERS[paper_name]
    page_w = _mm(paper_w_mm)
    page_h = _mm(paper_h_mm)

    # Default tag size: 80% of the shorter page dimension
    tag_mm = args.size if args.size else int(paper_w_mm * 0.80)
    tag_px = _mm(tag_mm)

    if tag_px > min(page_w, page_h):
        sys.exit(f"Tag size {tag_mm} mm is too large for {paper_name} paper.")

    dictionary = cv2.aruco.getPredefinedDictionary(_DICTS[dict_name])

    try:
        font = ImageFont.truetype(_FONT_PATH, _mm(10))
    except OSError:
        font = ImageFont.load_default()

    pages = [_make_page(i, dictionary, dict_name, tag_px, font, page_w, page_h)
             for i in tag_ids]

    if args.out:
        out = args.out
    else:
        ids_str = "_".join(str(i) for i in tag_ids)
        out = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "docs", f"aruco_tags_{ids_str}.pdf")
        )

    pages[0].save(out, save_all=True, append_images=pages[1:], resolution=DPI)
    print(f"Saved: {out}")
    print(f"  {len(pages)} page(s) · {paper_name} ({paper_w_mm}×{paper_h_mm} mm) · "
          f"{DPI} DPI · {tag_mm} mm tag · DICT_{dict_name} · IDs {tag_ids}")


if __name__ == "__main__":
    main()

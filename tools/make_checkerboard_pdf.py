#!/usr/bin/env python3
"""
make_checkerboard_pdf.py — Generate a printable checkerboard calibration target.

Produces docs/checkerboard_9x6.pdf:
  • 9×6 inner corners  →  10×7 squares
  • 25 mm squares (matches SQUARE_MM in calibrate_camera.py)
  • Centred on A4 with a 10 mm white border all round
  • 300 DPI for crisp printing

Print at 100 % scale (no "fit to page") for accurate square sizes.
After printing, measure one square with a ruler.  If it is not exactly
25 mm, update SQUARE_MM in tools/calibrate_camera.py to match.
"""

import os
import struct
import zlib

import cv2
import numpy as np

# ── Board geometry (must match calibrate_camera.py) ──────────────────────────
COLS      = 9          # inner corners horizontally  →  COLS+1 squares wide
ROWS      = 6          # inner corners vertically    →  ROWS+1 squares tall
SQUARE_MM = 25         # physical square size in mm

# ── Page / render settings ────────────────────────────────────────────────────
DPI       = 300
MM_PER_IN = 25.4

A4_W_MM   = 210
A4_H_MM   = 297
BORDER_MM = 10         # white border around the board

# ── Output path ───────────────────────────────────────────────────────────────
_HERE     = os.path.dirname(os.path.abspath(__file__))
OUT_PDF   = os.path.join(_HERE, '..', 'docs', 'checkerboard_9x6.pdf')


def mm_to_px(mm):
    return int(round(mm / MM_PER_IN * DPI))


def _png_bytes(img_bgr):
    """Encode a BGR numpy array as a PNG byte string (no external libs)."""
    # cv2.imencode returns (ok, 1-D uint8 array)
    ok, buf = cv2.imencode('.png', img_bgr)
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return buf.tobytes()


def _minimal_pdf(png_data, img_w_px, img_h_px,
                 page_w_mm, page_h_mm,
                 img_x_mm, img_y_mm,
                 img_w_mm, img_h_mm):
    """
    Build a minimal single-page PDF embedding one raw PNG image.
    No external PDF library required — just struct + zlib (stdlib).

    PDF coordinate origin is bottom-left; y_mm is measured from bottom.
    """
    # PDF uses points (1 pt = 1/72 inch)
    def mm2pt(mm):
        return mm / MM_PER_IN * 72.0

    pw = mm2pt(page_w_mm)
    ph = mm2pt(page_h_mm)
    ix = mm2pt(img_x_mm)
    iy = mm2pt(img_y_mm)
    iw = mm2pt(img_w_mm)
    ih = mm2pt(img_h_mm)

    # Object 1: Catalog
    # Object 2: Pages
    # Object 3: Page
    # Object 4: Image XObject
    # Object 5: Content stream

    img_stream = png_data   # embed as PNG (PDF 1.4+ supports /FlateDecode + PNG predictor,
                             # but the simplest path is to embed via /DCTDecode if JPEG or
                             # use raw deflate.  Easiest: re-encode as raw RGB + /FlateDecode.)

    # Re-encode as raw RGB rows with PNG predictor for smaller size
    h, w = img_h_px, img_w_px
    # Convert BGR → RGB
    img_rgb = cv2.cvtColor(
        cv2.imdecode(np.frombuffer(png_data, dtype=np.uint8), cv2.IMREAD_COLOR),
        cv2.COLOR_BGR2RGB
    )
    # Build raw pixels with PNG sub-filter (filter byte 0x01 per row)
    raw_rows = bytearray()
    for row in range(h):
        raw_rows.append(1)          # Sub filter
        row_data = img_rgb[row].tobytes()
        prev = bytes(3)
        filtered = bytearray()
        for i in range(0, len(row_data), 3):
            r = row_data[i:i+3]
            filtered += bytes([
                (r[0] - prev[0]) & 0xFF,
                (r[1] - prev[1]) & 0xFF,
                (r[2] - prev[2]) & 0xFF,
            ])
            prev = r
        raw_rows += filtered
    compressed = zlib.compress(bytes(raw_rows), 9)
    img_len = len(compressed)

    content = (
        f"q\n"
        f"{iw:.4f} 0 0 {ih:.4f} {ix:.4f} {iy:.4f} cm\n"
        f"/Im1 Do\n"
        f"Q\n"
    ).encode()
    content_compressed = zlib.compress(content, 9)

    offsets = []
    parts = []

    def add_obj(obj_bytes):
        offsets.append(sum(len(p) for p in parts) + len(b"%PDF-1.4\n"))
        parts.append(obj_bytes)

    # Obj 1 — Catalog
    add_obj(b"1 0 obj\n<< /Type /Catalog /Pages 2 0 R >>\nendobj\n")

    # Obj 2 — Pages
    add_obj((
        f"2 0 obj\n"
        f"<< /Type /Pages /Kids [3 0 R] /Count 1 >>\n"
        f"endobj\n"
    ).encode())

    # Obj 3 — Page
    add_obj((
        f"3 0 obj\n"
        f"<< /Type /Page /Parent 2 0 R\n"
        f"   /MediaBox [0 0 {pw:.4f} {ph:.4f}]\n"
        f"   /Contents 5 0 R\n"
        f"   /Resources << /XObject << /Im1 4 0 R >> >>\n"
        f">>\n"
        f"endobj\n"
    ).encode())

    # Obj 4 — Image XObject (PNG predictor / FlateDecode)
    add_obj((
        f"4 0 obj\n"
        f"<< /Type /XObject /Subtype /Image\n"
        f"   /Width {w} /Height {h}\n"
        f"   /ColorSpace /DeviceRGB /BitsPerComponent 8\n"
        f"   /Filter /FlateDecode\n"
        f"   /DecodeParms << /Predictor 15 /Colors 3 /BitsPerComponent 8 /Columns {w} >>\n"
        f"   /Length {img_len}\n"
        f">>\n"
        f"stream\n"
    ).encode() + compressed + b"\nendstream\nendobj\n")

    # Obj 5 — Content stream
    clen = len(content_compressed)
    add_obj((
        f"5 0 obj\n"
        f"<< /Filter /FlateDecode /Length {clen} >>\n"
        f"stream\n"
    ).encode() + content_compressed + b"\nendstream\nendobj\n")

    # Cross-reference table
    header = b"%PDF-1.4\n"
    body = b"".join(parts)
    xref_offset = len(header) + len(body)
    xref = b"xref\n"
    xref += f"0 {len(offsets) + 1}\n".encode()
    xref += b"0000000000 65535 f \n"
    for off in offsets:
        xref += f"{off:010d} 00000 n \n".encode()

    trailer = (
        f"trailer\n"
        f"<< /Size {len(offsets) + 1} /Root 1 0 R >>\n"
        f"startxref\n"
        f"{xref_offset}\n"
        f"%%EOF\n"
    ).encode()

    return header + bytes(body) + xref + trailer


def main():
    n_cols = COLS + 1   # 10
    n_rows = ROWS + 1   # 7

    board_w_mm = n_cols * SQUARE_MM   # 250 mm
    board_h_mm = n_rows * SQUARE_MM   # 175 mm

    # Total image size includes border on all sides
    img_w_mm = board_w_mm + 2 * BORDER_MM
    img_h_mm = board_h_mm + 2 * BORDER_MM

    img_w_px = mm_to_px(img_w_mm)
    img_h_px = mm_to_px(img_h_mm)
    sq_px    = mm_to_px(SQUARE_MM)
    brd_px   = mm_to_px(BORDER_MM)

    # Draw board on white canvas (BGR)
    canvas = np.full((img_h_px, img_w_px, 3), 255, dtype=np.uint8)

    for r in range(n_rows):
        for c in range(n_cols):
            if (r + c) % 2 == 0:
                continue          # leave white
            x0 = brd_px + c * sq_px
            y0 = brd_px + r * sq_px
            canvas[y0:y0 + sq_px, x0:x0 + sq_px] = 0   # black

    # Add label below the board
    label = (f"Checkerboard  {n_cols}\u00d7{n_rows} squares  "
             f"({COLS}\u00d7{ROWS} inner corners)  |  "
             f"{SQUARE_MM} mm per square  |  Print at 100\u0025 scale")
    font_scale = img_w_px / 2200
    cv2.putText(canvas, label,
                (brd_px, img_h_px - mm_to_px(3)),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                (80, 80, 80), max(1, mm_to_px(0.4)), cv2.LINE_AA)

    png_data = _png_bytes(canvas)

    # Place image centred on A4 (landscape if board is wider than tall)
    if img_w_mm > img_h_mm:
        page_w_mm, page_h_mm = A4_H_MM, A4_W_MM   # A4 landscape
    else:
        page_w_mm, page_h_mm = A4_W_MM, A4_H_MM   # A4 portrait

    # Centre the board on the page
    place_x_mm = (page_w_mm - img_w_mm) / 2
    place_y_mm = (page_h_mm - img_h_mm) / 2   # PDF y from bottom

    os.makedirs(os.path.dirname(OUT_PDF), exist_ok=True)
    pdf = _minimal_pdf(
        png_data,
        img_w_px, img_h_px,
        page_w_mm, page_h_mm,
        place_x_mm, place_y_mm,
        img_w_mm, img_h_mm,
    )
    with open(OUT_PDF, 'wb') as f:
        f.write(pdf)

    print(f"Saved: {os.path.normpath(OUT_PDF)}")
    print(f"  Board:   {n_cols}×{n_rows} squares  ({COLS}×{ROWS} inner corners)")
    print(f"  Squares: {SQUARE_MM} mm  →  board {board_w_mm}×{board_h_mm} mm")
    print(f"  Page:    {'landscape' if img_w_mm > img_h_mm else 'portrait'} A4")
    print(f"  DPI:     {DPI}")
    print()
    print("Print at 100% scale (no 'fit to page').")
    print("Verify one square measures exactly 25 mm before calibrating.")


if __name__ == '__main__':
    main()

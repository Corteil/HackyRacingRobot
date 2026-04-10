#!/usr/bin/env python3
"""
tools/build_gs_html.py — Generate ground_station.html from robot_dashboard.py.

Run this once after cloning, and again whenever robot_dashboard.py changes.

Usage
-----
  python3 tools/build_gs_html.py
  python3 tools/build_gs_html.py --dashboard path/to/robot_dashboard.py
  python3 tools/build_gs_html.py --out path/to/ground_station.html
"""

import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def build(dashboard_path: str, out_path: str):
    print(f"Reading:  {dashboard_path}")
    src = open(dashboard_path, encoding='utf-8').read()

    # ── Extract the _HTML string ───────────────────────────────────────────────
    marker_start = '_HTML = r"""'
    marker_end   = '"""\n\n\n# ── Flask app'
    if marker_start not in src:
        sys.exit("ERROR: cannot find _HTML string in robot_dashboard.py")
    start = src.index(marker_start) + len(marker_start)
    end   = src.index(marker_end, start)
    html  = src[start:end]
    print(f"Extracted HTML: {len(html):,} chars")

    # ── Patch 1: Add FPV Camera to PANEL_DEFS ─────────────────────────────────
    old = "  { type:'camera:front_left',  label:'Camera FL'    },"
    new = "  { type:'camera:fpv',         label:'FPV Camera'   },\n" + old
    if old in html:
        html = html.replace(old, new, 1)
        print("✓  Added FPV Camera to PANEL_DEFS")
    else:
        print("⚠  PANEL_DEFS patch not applied (format may have changed)")

    # ── Patch 2: Add FPV to PRESET_MAP ────────────────────────────────────────
    old = "  front_left:'camera:front_left', front_right:'camera:front_right',"
    new = "  fpv:'camera:fpv', " + old.lstrip()
    if old in html:
        html = html.replace(old, new, 1)
        print("✓  Added fpv to PRESET_MAP")
    else:
        print("⚠  PRESET_MAP patch not applied")

    # ── Patch 3: Default Race preset uses FPV ─────────────────────────────────
    old = "Race:  ['camera:front_left','camera:front_right','camera:rear','lidar'],"
    new = "Race:  ['camera:fpv','camera:front_left','telemetry','lidar'],"
    if old in html:
        html = html.replace(old, new, 1)
        print("✓  Updated Race preset to include FPV")
    else:
        print("⚠  Race preset patch not applied")

    # ── Patch 4: updateCameraPanel — skip for 'fpv' (no robot-side status) ────
    old = ("  const okKey  = camKey==='front_left'?'cam_fl_ok' :"
           " camKey==='front_right'?'cam_fr_ok':'cam_re_ok';\n"
           "  const recKey = camKey==='front_left'?'cam_fl_rec':"
           " camKey==='front_right'?'cam_fr_rec':'cam_re_rec';")
    new = ("  if (camKey === 'fpv') return; // FPV is local — no robot-side status\n"
           + old)
    if old in html:
        html = html.replace(old, new, 1)
        print("✓  Patched updateCameraPanel to skip fpv")
    else:
        print("⚠  updateCameraPanel patch not applied")

    # ── Patch 5: Ground station status badges in status bar ───────────────────
    old = '<span class="sb-badge" id="sb-conn"'
    new = ('<span class="sb-badge" id="sb-sik" style="color:var(--gray)">📡 --</span>\n'
           '    <span class="sb-badge" id="sb-ntrip" style="color:var(--gray)">RTK --</span>\n'
           '    ' + old)
    if old in html and 'sb-sik' not in html:
        html = html.replace(old, new, 1)
        print("✓  Added SiK/NTRIP badges to status bar")
    else:
        print("⚠  Status badge patch not applied (may already exist)")

    # ── Patch 6: applyState — update ground station badges ────────────────────
    gs_js = (
        "\n  // Ground station: update link-quality and NTRIP badges\n"
        "  (function() {\n"
        "    const sik = document.getElementById('sb-sik');\n"
        "    if (sik) {\n"
        "      const age = d._link_age;\n"
        "      sik.textContent = (age !== undefined) ? '\U0001f4e1 ' + age.toFixed(1) + 's' : '\U0001f4e1 --';\n"
        "      sik.style.color = (age !== undefined && age < 2) ? C.green\n"
        "                      : (age < 5)                      ? C.yellow : C.red;\n"
        "    }\n"
        "    const nb = document.getElementById('sb-ntrip');\n"
        "    if (nb) {\n"
        "      const ns = d._ntrip_status || '';\n"
        "      nb.textContent = 'RTK ' + (ns || '--');\n"
        "      nb.style.color = ns === 'connected'       ? C.green\n"
        "                     : ns.startsWith('connect') ? C.yellow : C.gray;\n"
        "    }\n"
        "  })();\n"
    )
    anchor = 'function applyState(s) {\n  lastState = s;'
    if anchor in html and '_link_age' not in html:
        # Patch uses 's' (the parameter name in robot_dashboard.py)
        gs_js_patched = gs_js.replace("d._link_age", "s._link_age") \
                             .replace("d._ntrip_status", "s._ntrip_status")
        html = html.replace(anchor, anchor + gs_js_patched, 1)
        print("✓  Injected ground station badge updates into applyState()")
    else:
        print("⚠  applyState patch not applied (may already exist)")

    # ── Patch 7: Hide robot-only control buttons ───────────────────────────────
    # Remove ArUco toggle, rotate, FPV Camera power buttons (not useful remotely)
    for btn_text, reason in (
        ("onclick=\"sendCmd('bench_toggle')\"",    "bench power (robot-side)"),
    ):
        if btn_text in html:
            # Wrap button in a hidden span rather than deleting (safer)
            html = html.replace(
                f'<button {btn_text}',
                f'<button style="display:none" {btn_text}',
                1,
            )
            print(f"✓  Hidden: {reason}")

    # ── Write output ──────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)

    size_kb = len(html) / 1024
    print(f"\n✓  Written: {out_path}  ({size_kb:.0f} KB)")
    print("   Run ground_station.py to start the server.")


def main():
    ap = argparse.ArgumentParser(
        description="Build ground_station.html from robot_dashboard.py")
    ap.add_argument("--dashboard", default=os.path.join(ROOT, "robot_dashboard.py"),
                    help="Path to robot_dashboard.py")
    ap.add_argument("--out", default=os.path.join(ROOT, "tools", "ground_station.html"),
                    help="Output HTML path")
    args = ap.parse_args()

    if not os.path.exists(args.dashboard):
        sys.exit(f"Not found: {args.dashboard}\n"
                 f"Pass --dashboard path/to/robot_dashboard.py")

    build(args.dashboard, args.out)


if __name__ == "__main__":
    main()

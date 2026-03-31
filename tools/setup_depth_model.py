#!/usr/bin/env python3
"""
tools/setup_depth_model.py — Download and verify the scdepthv3 HEF model.

Downloads the pre-compiled scdepthv3 HEF for the Hailo-8L from the Hailo
Model Zoo S3 bucket and places it in models/scdepthv3.hef relative to the
project root.  Also verifies the Hailo device is reachable.

Usage
-----
  python3 tools/setup_depth_model.py

  Options:
    --model-dir DIR    Destination directory (default: <project_root>/models/)
    --force            Re-download even if the file already exists
    --no-verify        Skip Hailo device verification after download
"""

import argparse
import hashlib
import os
import sys
import urllib.request
from pathlib import Path

# ── Model registry ─────────────────────────────────────────────────────────────
#
# scdepthv3 — self-supervised monocular depth, Hailo Model Zoo v2.x
# Pre-compiled for Hailo-8L (HAILO8L arch).  145 fps at 256×320 input.
#
# Source: Hailo Model Zoo, depth estimation category
#   https://github.com/hailo-ai/hailo_model_zoo

MODELS = {
    'scdepthv3': {
        'filename': 'scdepthv3.hef',
        'url': (
            'https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/'
            'ModelZoo/Compiled/hailo8l/scdepthv3.hef'
        ),
        # SHA-256 of the official HEF — update this if Hailo releases a new version
        'sha256': None,   # set to None to skip integrity check until hash is known
        'description': 'scdepthv3 monocular depth — Hailo-8L, 145 fps, 256×320',
    },
}

_HERE    = Path(__file__).resolve().parent.parent   # project root
_DEFAULT = _HERE / 'models'


def _download(url: str, dest: Path, force: bool = False) -> bool:
    """Download *url* to *dest*.  Returns True on success."""
    if dest.exists() and not force:
        print(f"  Already exists: {dest}")
        return True
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix('.tmp')
    print(f"  Downloading {url}")
    print(f"  → {dest}")
    try:
        def _reporthook(count, block_size, total_size):
            if total_size > 0:
                pct = min(100, count * block_size * 100 // total_size)
                bar = '#' * (pct // 4)
                print(f"\r  [{bar:<25}] {pct:3d}%", end='', flush=True)
        urllib.request.urlretrieve(url, tmp, reporthook=_reporthook)
        print()   # newline after progress bar
        tmp.rename(dest)
        return True
    except Exception as e:
        print(f"\n  ERROR: download failed: {e}", file=sys.stderr)
        if tmp.exists():
            tmp.unlink()
        return False


def _verify_sha256(path: Path, expected: str) -> bool:
    """Return True if SHA-256 of *path* matches *expected*."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    actual = h.hexdigest()
    if actual != expected:
        print(f"  ERROR: SHA-256 mismatch!", file=sys.stderr)
        print(f"    expected: {expected}", file=sys.stderr)
        print(f"    got:      {actual}",   file=sys.stderr)
        return False
    print(f"  SHA-256 OK: {actual[:16]}…")
    return True


def _verify_hailo() -> bool:
    """Check the Hailo device is reachable via hailortcli."""
    import shutil, subprocess
    if not shutil.which('hailortcli'):
        print("  hailortcli not found — install the HailoRT package to verify hardware.")
        print("  (Model download is still valid; device check skipped.)")
        return True
    try:
        result = subprocess.run(
            ['hailortcli', 'fw-control', 'identify'],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if 'Device' in line or 'Firmware' in line or 'hailo' in line.lower():
                    print(f"  {line.strip()}")
            print("  Hailo device verified OK.")
            return True
        else:
            print(f"  WARNING: hailortcli returned {result.returncode}: {result.stderr.strip()}")
            return False
    except Exception as e:
        print(f"  WARNING: could not run hailortcli: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model-dir', default=str(_DEFAULT),
                        help='Directory for downloaded HEF files (default: models/)')
    parser.add_argument('--force',     action='store_true',
                        help='Re-download even if HEF already exists')
    parser.add_argument('--no-verify', action='store_true',
                        help='Skip Hailo device verification')
    args = parser.parse_args()

    model_dir = Path(args.model_dir)
    all_ok    = True

    print("=== HackyRacingRobot — depth model setup ===\n")

    for name, info in MODELS.items():
        dest = model_dir / info['filename']
        print(f"Model: {name}")
        print(f"  {info['description']}")

        ok = _download(info['url'], dest, force=args.force)
        if not ok:
            all_ok = False
            continue

        if info['sha256']:
            ok = _verify_sha256(dest, info['sha256'])
            if not ok:
                all_ok = False
                continue
        else:
            size_mb = dest.stat().st_size / 1_048_576
            print(f"  File size: {size_mb:.1f} MB (hash check skipped — no expected hash set)")

        print()

    if not args.no_verify:
        print("Verifying Hailo-8L device:")
        _verify_hailo()
        print()

    if all_ok:
        print("Setup complete.")
        print(f"HEF location: {model_dir}/scdepthv3.hef")
        print()
        print("Enable depth mapping in robot.ini:")
        print("  [depth]")
        print("  enabled = true")
        print("  mode    = fusion")
    else:
        print("Setup completed with errors — check messages above.", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()

"""
robot/stereo_depth.py — Metric stereo depth estimation using OpenCV StereoSGBM.

Loads per-camera calibration files (.npz), builds stereo rectification maps,
and produces float32 depth arrays (metres) from synchronised left/right frame
pairs.  Runs entirely on CPU — no Hailo dependency.

Expected calibration .npz keys
-------------------------------
  camera_matrix  — 3×3 intrinsic matrix
  dist_coeffs    — distortion coefficients (1×5 or 1×8)
  frame_size     — (width, height) at which the calibration was captured
  R              — 3×3 rotation between left and right cameras   (optional)
  T              — 3×1 translation between cameras in metres      (optional)

If R / T are absent the module uses identity / zero, which still produces
depth but with reduced accuracy.  Run tools/calibrate_camera.py with both
cameras to obtain proper stereo extrinsics.
"""

import logging
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np

log = logging.getLogger(__name__)


class StereoDepth:
    """Metric depth from a calibrated stereo camera pair.

    Parameters
    ----------
    left_calib : str
        Path to the left-camera calibration .npz file.
    right_calib : str
        Path to the right-camera calibration .npz file.
    min_disparity : int
        Minimum possible disparity value (usually 0).
    num_disparities : int
        Range of disparity search; must be divisible by 16.
    block_size : int
        Matched block size (3–11, odd).  Larger = smoother but less detail.
    max_depth_m : float
        Depth values beyond this are masked out as invalid.
    """

    def __init__(self,
                 left_calib:      str,
                 right_calib:     str,
                 min_disparity:   int   = 0,
                 num_disparities: int   = 128,
                 block_size:      int   = 9,
                 max_depth_m:     float = 8.0):
        self._max_depth      = max_depth_m
        self._min_disp       = min_disparity
        self._num_disp       = num_disparities
        self._block_size     = block_size
        self._maps_built     = False
        self._map_lx         = None
        self._map_ly         = None
        self._map_rx         = None
        self._map_ry         = None
        self._Q: Optional[np.ndarray]         = None
        self._output_wh: Optional[Tuple[int, int]] = None

        bs = block_size
        self._sgbm = cv2.StereoSGBM_create(
            minDisparity      = min_disparity,
            numDisparities    = num_disparities,
            blockSize         = bs,
            P1                = 8  * 3 * bs * bs,
            P2                = 32 * 3 * bs * bs,
            disp12MaxDiff     = 1,
            uniquenessRatio   = 10,
            speckleWindowSize = 100,
            speckleRange      = 32,
            preFilterCap      = 63,
            mode              = cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self._left_cal  = self._load_calib(left_calib,  'left')
        self._right_cal = self._load_calib(right_calib, 'right')

    # ── calibration loading ───────────────────────────────────────────────────

    @staticmethod
    def _load_calib(path: str, label: str) -> Optional[dict]:
        if not path:
            return None
        p = Path(path)
        if not p.exists():
            log.warning("StereoDepth: %s calibration not found: %s", label, path)
            return None
        try:
            cal = np.load(p)
            return {
                'mtx':  cal['camera_matrix'],
                'dist': cal['dist_coeffs'],
                'size': tuple(int(v) for v in cal['frame_size']),
                'R':    cal['R'] if 'R' in cal else np.eye(3, dtype=np.float64),
                'T':    cal['T'] if 'T' in cal else np.zeros((3, 1), dtype=np.float64),
            }
        except Exception as e:
            log.error("StereoDepth: failed to load %s calibration %s: %s", label, path, e)
            return None

    # ── public interface ──────────────────────────────────────────────────────

    @property
    def available(self) -> bool:
        """True when both calibration files loaded successfully."""
        return self._left_cal is not None and self._right_cal is not None

    def compute(self,
                left:  np.ndarray,
                right: np.ndarray) -> Optional[np.ndarray]:
        """Compute a metric depth map from a synchronised stereo pair.

        Parameters
        ----------
        left, right : np.ndarray (H, W, 3) uint8
            RGB frames of equal size from the left and right cameras.

        Returns
        -------
        np.ndarray (H, W) float32
            Depth in metres; 0.0 = invalid pixel.
        None if calibration is unavailable.
        """
        if not self.available:
            return None

        h, w = left.shape[:2]
        if not self._maps_built or self._output_wh != (w, h):
            self._build_maps(w, h)

        l_rect = cv2.remap(left,  self._map_lx, self._map_ly, cv2.INTER_LINEAR)
        r_rect = cv2.remap(right, self._map_rx, self._map_ry, cv2.INTER_LINEAR)

        l_gray = cv2.cvtColor(l_rect, cv2.COLOR_RGB2GRAY)
        r_gray = cv2.cvtColor(r_rect, cv2.COLOR_RGB2GRAY)

        disp16 = self._sgbm.compute(l_gray, r_gray).astype(np.float32)
        disp   = disp16 / 16.0

        # Reproject disparity to 3-D; Q encodes the stereo geometry
        points = cv2.reprojectImageTo3D(disp, self._Q)
        depth  = points[:, :, 2]   # Z channel = metres

        valid = (disp > 0) & np.isfinite(depth) & (depth > 0.0) & (depth < self._max_depth)
        return np.where(valid, depth, 0.0).astype(np.float32)

    def get_depth_at(self,
                     depth_map: np.ndarray,
                     px: int, py: int) -> Optional[float]:
        """Return metric depth (m) at display-space pixel (px, py), or None."""
        if depth_map is None:
            return None
        h, w = depth_map.shape
        if 0 <= py < h and 0 <= px < w:
            v = float(depth_map[py, px])
            return v if v > 0.0 else None
        return None

    # ── private ───────────────────────────────────────────────────────────────

    def _build_maps(self, w: int, h: int) -> None:
        """Build stereo rectification and disparity-to-depth maps for (w, h)."""
        lc = self._left_cal
        rc = self._right_cal
        target = (w, h)

        l_mtx = _scale_intrinsics(lc['mtx'], lc['size'], target)
        r_mtx = _scale_intrinsics(rc['mtx'], rc['size'], target)

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            l_mtx, lc['dist'],
            r_mtx, rc['dist'],
            target,
            lc['R'], lc['T'],
            flags = cv2.CALIB_ZERO_DISPARITY,
            alpha = 0,
        )
        self._Q = Q
        self._map_lx, self._map_ly = cv2.initUndistortRectifyMap(
            l_mtx, lc['dist'], R1, P1, target, cv2.CV_16SC2)
        self._map_rx, self._map_ry = cv2.initUndistortRectifyMap(
            r_mtx, rc['dist'], R2, P2, target, cv2.CV_16SC2)
        self._output_wh  = target
        self._maps_built = True
        log.info("StereoDepth: rectification maps built for %d×%d", w, h)


# ── helpers ───────────────────────────────────────────────────────────────────

def _scale_intrinsics(mtx: np.ndarray,
                      cal_size: tuple,
                      target: tuple) -> np.ndarray:
    """Scale a 3×3 camera matrix from cal_size to target resolution."""
    sx = target[0] / cal_size[0]
    sy = target[1] / cal_size[1]
    m = mtx.copy().astype(np.float64)
    m[0, 0] *= sx;  m[1, 1] *= sy
    m[0, 2] *= sx;  m[1, 2] *= sy
    return m

"""
robot/depth_estimator.py — Monocular depth estimation via Hailo-8L AI Kit.

Uses scdepthv3, the only monocular depth model with a pre-compiled HEF for the
Hailo-8L (13 TOPS).  Runs at ~145 fps on the Hailo-8L — the Pi 5 CPU handles
stereo depth while the Hailo accelerates monocular depth concurrently.

Output is *relative* depth (closer = higher value, normalised 0–1).  When a
stereo depth map is supplied as an anchor, a per-frame least-squares scale
factor is fitted so the result is expressed in metres.

Graceful degradation
--------------------
  - If hailort is not installed, ``available`` is False and ``infer()``
    returns None without raising.
  - The depth thread in robot_daemon.py falls back to stereo-only mode.
  - If the HEF file is missing, a warning is logged and the estimator is
    silently disabled.
"""

import logging
import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger(__name__)

# ── Hailo platform import (optional) ──────────────────────────────────────────
try:
    from hailo_platform import (
        VDevice, HailoStreamInterface, ConfigureParams,
        HailoSchedulingAlgorithm, FormatType, HEF, InferVStreams,
    )
    _HAILO_AVAILABLE = True
except ImportError:
    _HAILO_AVAILABLE = False
    log.debug("hailort not installed — monocular depth via Hailo disabled")


class DepthEstimator:
    """Monocular depth via scdepthv3 on the Hailo-8L.

    Parameters
    ----------
    model_path : str
        Path to the scdepthv3 .hef file (download via tools/setup_depth_model.py).
    scale_mode : str
        ``'stereo'`` — fit a per-frame scale factor using a stereo depth map.
        ``'none'``   — return raw relative depth (0–1, not metric).
    scale_percentile : int
        Percentile of valid stereo pixels used when fitting the scale factor.
        50 = median; lower values bias toward nearer objects.
    """

    def __init__(self,
                 model_path:       str,
                 scale_mode:       str = 'stereo',
                 scale_percentile: int = 50):
        self._model_path       = model_path
        self._scale_mode       = scale_mode
        self._scale_percentile = scale_percentile
        self._lock             = threading.Lock()
        self._device           = None
        self._network_group    = None
        self._input_vstreams_params  = None
        self._output_vstreams_params = None
        self._input_wh: Optional[tuple] = None   # (W, H) for cv2.resize
        self._available = False

        self._init_hailo()

    # ── initialisation ────────────────────────────────────────────────────────

    def _init_hailo(self) -> None:
        if not _HAILO_AVAILABLE:
            return
        if not Path(self._model_path).exists():
            log.warning(
                "DepthEstimator: HEF not found at %s — run tools/setup_depth_model.py",
                self._model_path,
            )
            return
        try:
            hef    = HEF(self._model_path)
            params = VDevice.create_params()
            self._device = VDevice(params)

            configure_params = ConfigureParams.create_from_hef(
                hef, interface=HailoStreamInterface.PCIe)
            network_groups = self._device.configure(hef, configure_params)
            self._network_group = network_groups[0]

            self._input_vstreams_params = (
                self._network_group.make_input_vstream_params(
                    False, FormatType.FLOAT32,
                    HailoSchedulingAlgorithm.ROUND_ROBIN))
            self._output_vstreams_params = (
                self._network_group.make_output_vstream_params(
                    False, FormatType.FLOAT32,
                    HailoSchedulingAlgorithm.ROUND_ROBIN))

            # Resolve model input resolution from HEF metadata
            info  = hef.get_input_vstream_infos()[0]
            shape = info.shape   # typically (H, W, C) for scdepthv3
            if len(shape) == 3:
                # HEF reports (H, W, C)
                self._input_wh = (int(shape[1]), int(shape[0]))   # (W, H) for cv2
            else:
                log.warning("DepthEstimator: unexpected input shape %s", shape)
                self._input_wh = (320, 256)   # scdepthv3 fallback

            self._available = True
            log.info(
                "DepthEstimator: scdepthv3 loaded from %s, input %dx%d",
                self._model_path, self._input_wh[0], self._input_wh[1],
            )
        except Exception as e:
            log.warning("DepthEstimator: Hailo init failed (%s) — monocular depth disabled", e)
            self._available = False

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def available(self) -> bool:
        """True when the Hailo device and model are ready for inference."""
        return self._available

    def infer(self,
              frame_rgb:    np.ndarray,
              stereo_depth: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Run monocular depth inference on one RGB frame.

        Parameters
        ----------
        frame_rgb : np.ndarray (H, W, 3) uint8
            Full-resolution RGB frame from any single camera.
        stereo_depth : np.ndarray (H, W) float32 or None
            Metric depth map from StereoDepth.compute().  When provided and
            ``scale_mode == 'stereo'``, the output is scaled to metres.

        Returns
        -------
        np.ndarray (H, W) float32
            Depth in metres (if stereo anchor available) or relative [0..1].
        None if Hailo is unavailable or inference fails.
        """
        if not self._available:
            return None

        fh, fw = frame_rgb.shape[:2]
        iw, ih = self._input_wh

        # Resize to model input, convert to float32 [0..1]
        resized = cv2.resize(frame_rgb, (iw, ih), interpolation=cv2.INTER_LINEAR)
        tensor  = (resized.astype(np.float32) / 255.0)[np.newaxis]  # (1, H, W, 3)

        try:
            with self._lock:
                with self._network_group.activate(self._network_group.create_params()):
                    in_name = self._network_group.get_input_vstream_infos()[0].name
                    with InferVStreams(
                        self._network_group,
                        self._input_vstreams_params,
                        self._output_vstreams_params,
                    ) as pipeline:
                        results = pipeline.infer({in_name: tensor})

            raw = results[list(results.keys())[0]][0]   # remove batch dim
        except Exception as e:
            log.warning("DepthEstimator: inference error: %s", e)
            return None

        # Normalise output to [0..1]
        mn, mx    = float(raw.min()), float(raw.max())
        rel_depth = ((raw - mn) / (mx - mn + 1e-8)).astype(np.float32)

        # Resize back to original frame size if needed
        if rel_depth.shape[:2] != (fh, fw):
            rel_depth = cv2.resize(rel_depth, (fw, fh), interpolation=cv2.INTER_LINEAR)

        # ── stereo-guided metric scaling ──────────────────────────────────────
        if self._scale_mode == 'stereo' and stereo_depth is not None:
            scaled = _fit_stereo_scale(rel_depth, stereo_depth, self._scale_percentile)
            if scaled is not None:
                return scaled

        return rel_depth   # relative — caller should check scale_mode

    def stop(self) -> None:
        """Release the Hailo device."""
        if self._device is not None:
            try:
                self._device.release()
            except Exception:
                pass
            self._device    = None
        self._available = False


# ── helpers ───────────────────────────────────────────────────────────────────

def _fit_stereo_scale(rel:    np.ndarray,
                      stereo: np.ndarray,
                      percentile: int) -> Optional[np.ndarray]:
    """Scale relative depth to metric using stereo depth as an anchor.

    Fits a single multiplicative factor: metric ≈ scale × relative.
    Uses pixels where stereo is valid; outliers are suppressed by restricting
    to values below the given percentile of the stereo distribution.

    Returns None if there are too few valid anchor pixels.
    """
    valid = stereo > 0.1
    if valid.sum() < 50:
        return None

    s_vals = stereo[valid].ravel()
    r_vals = rel[valid].ravel()

    # Restrict to stereo depths below the given percentile to reduce outliers
    pct_threshold = float(np.percentile(s_vals, percentile))
    mask = s_vals < pct_threshold * 2.0
    if mask.sum() < 20:
        return None

    scale = float(np.median(s_vals[mask] / (r_vals[mask] + 1e-8)))
    return np.clip(rel * scale, 0.0, 20.0).astype(np.float32)

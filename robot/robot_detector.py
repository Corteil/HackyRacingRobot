"""
robot/robot_detector.py — Racing robot detection via YOLOv8n on Hailo-10H.

Detects other racing robots in camera frames using a custom-trained YOLOv8n
model compiled to HEF format for the Hailo-10H AI HAT+ 2.

Train and compile the model using docs/robot_detector_training.html, then
place the resulting .hef at the path configured under [robot_detector] in
robot.ini.

Output format
-------------
  detect(frame) returns a RobotDetection containing a list of DetectedRobot
  objects, each with pixel bounding-box coordinates and confidence.  The frame
  is annotated in-place with orange bounding boxes when draw=True.

Graceful degradation
--------------------
  - If hailort is not installed, available is False and detect() returns None.
  - If the HEF file is missing, a warning is logged and the detector is
    silently disabled.
  - The camera thread falls back to ArUco-only mode when unavailable.

Hailo output format (hailort 5.x, create_infer_model API)
----------------------------------------------------------
  The model compiled with Hailo DFC for HAILO10H produces two output tensors:

    ne_activation_activation1  float32  (1, N, 1)
        Class confidence per anchor, already sigmoid'd on-chip.  N = 2100
        for a 320×320 input (40×40 + 20×20 + 10×10 anchor grid).

    depth_to_space1            float32  (16, N, 4)
        DFL (Distribution Focal Loss) regression logits.  Axis 0 is the 16
        distribution bins; axis 2 is the 4 ltrb coordinate groups.
        Decode: softmax over axis 0 → weighted sum with [0..15] → ltrb in
        grid-cell units → multiply by stride → pixel distances from anchor.
"""

import logging
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

log = logging.getLogger(__name__)

# ── Hailo platform import (optional) ─────────────────────────────────────────
try:
    from hailo_platform import VDevice, FormatType
    _HAILO_AVAILABLE = True
except ImportError:
    _HAILO_AVAILABLE = False
    log.debug("hailort not installed — robot detector via Hailo disabled")

# ── Drawing constants (RGB order) ─────────────────────────────────────────────
_BOX_COLOUR   = (255, 128,   0)   # orange
_LABEL_COLOUR = (255, 255, 255)   # white
_FONT         = cv2.FONT_HERSHEY_SIMPLEX
_FONT_SCALE   = 0.5
_THICKNESS    = 2

_STRIDES = (8, 16, 32)   # YOLOv8 feature-pyramid strides
_REG_MAX = 16             # DFL distribution bins


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class DetectedRobot:
    """One detected robot in a camera frame."""
    center_x:   int    # pixel x of bounding-box centre
    center_y:   int    # pixel y of bounding-box centre
    x1:         int    # left edge
    y1:         int    # top edge
    x2:         int    # right edge
    y2:         int    # bottom edge
    confidence: float  # 0.0–1.0


@dataclass
class RobotDetection:
    """Snapshot of one detection pass."""
    robots:    List[DetectedRobot] = field(default_factory=list)
    fps:       float               = 0.0
    timestamp: float               = 0.0

    @property
    def count(self) -> int:
        return len(self.robots)

    @property
    def nearest(self) -> Optional[DetectedRobot]:
        """Robot with the largest bounding-box area (assumed nearest)."""
        if not self.robots:
            return None
        return max(self.robots, key=lambda r: (r.x2 - r.x1) * (r.y2 - r.y1))


# ── Anchor grid helpers ───────────────────────────────────────────────────────

def _build_anchor_grid(img_h: int, img_w: int,
                       strides: Tuple[int, ...] = _STRIDES
                       ) -> Tuple[np.ndarray, np.ndarray]:
    """Return (anchor_points, stride_per_anchor) arrays of shape (N, 2) and (N,).

    anchor_points[i] = (cx, cy) in pixels at the model input resolution.
    stride_per_anchor[i] = the stride for anchor i.
    """
    pts, strd = [], []
    for s in strides:
        gh, gw = img_h // s, img_w // s
        for gy in range(gh):
            for gx in range(gw):
                pts.append(((gx + 0.5) * s, (gy + 0.5) * s))
                strd.append(s)
    return (np.array(pts,  dtype=np.float32),
            np.array(strd, dtype=np.float32))


def _dfl_decode(dfl: np.ndarray, strides: np.ndarray) -> np.ndarray:
    """Decode DFL regression tensor to pixel-space x1y1x2y2 boxes.

    Parameters
    ----------
    dfl : (reg_max, N, 4)  — raw DFL logits from the model
    strides : (N,)         — stride for each anchor

    Returns
    -------
    boxes : (N, 4) float32 — ltrb distances in pixels from anchor centre
    """
    # softmax over the reg_max axis (axis 0)
    e = np.exp(dfl - dfl.max(axis=0, keepdims=True))
    probs = e / e.sum(axis=0, keepdims=True)          # (16, N, 4)

    weights = np.arange(_REG_MAX, dtype=np.float32)   # (16,)
    # weighted sum → ltrb in grid-cell units → multiply by stride
    ltrb = (probs * weights[:, None, None]).sum(axis=0)  # (N, 4)
    return ltrb * strides[:, None]                        # (N, 4) pixels


# ── Detector ──────────────────────────────────────────────────────────────────

class RobotDetector:
    """YOLOv8n robot detector running on the Hailo-10H.

    Uses the hailort 5.x create_infer_model / ConfiguredInferModel API.

    Parameters
    ----------
    model_path : str
        Path to the compiled .hef file.
    conf : float
        Confidence threshold (0–1).
    iou : float
        IoU threshold for NMS duplicate suppression.
    draw : bool
        Annotate frames in-place with bounding boxes (default True).
    """

    def __init__(self,
                 model_path: str,
                 conf:       float = 0.45,
                 iou:        float = 0.45,
                 draw:       bool  = True):
        self._model_path = model_path
        self._conf       = conf
        self._iou        = iou
        self._draw       = draw
        self._lock       = threading.Lock()

        self._device          = None
        self._infer_model     = None
        self._configured      = None   # ConfiguredInferModel (kept open)
        self._configured_cm   = None   # the context manager object
        self._input_name      = None
        self._conf_name       = None   # confidence output tensor name
        self._dfl_name        = None   # DFL regression output tensor name
        self._input_wh        = None   # (W, H) for cv2.resize
        self._anchor_pts      = None   # (N, 2) float32
        self._anchor_strides  = None   # (N,)   float32
        self._available       = False

        self._t_prev = time.monotonic()

        self._init_hailo()

    # ── initialisation ────────────────────────────────────────────────────────

    def _init_hailo(self) -> None:
        if not _HAILO_AVAILABLE:
            return
        if not Path(self._model_path).exists():
            log.warning(
                "RobotDetector: HEF not found at %s — "
                "run the training pipeline in docs/robot_detector_training.html",
                self._model_path,
            )
            return
        try:
            params = VDevice.create_params()
            self._device = VDevice(params)
            self._infer_model = self._device.create_infer_model(self._model_path)

            # Request float32 I/O; hailort handles uint8↔float32 conversion
            self._infer_model.input().set_format_type(FormatType.FLOAT32)
            for out in self._infer_model.outputs:
                out.set_format_type(FormatType.FLOAT32)

            self._input_name = self._infer_model.input_names[0]

            # Identify confidence and DFL outputs by shape heuristic:
            #   confidence  → last dim == 1   e.g. (1, 2100, 1)
            #   DFL         → first dim == 16 e.g. (16, 2100, 4)
            for out in self._infer_model.outputs:
                shape = tuple(out.shape)
                if shape[-1] == 1:
                    self._conf_name = out.name
                elif shape[0] == _REG_MAX:
                    self._dfl_name = out.name
            if not self._conf_name or not self._dfl_name:
                raise RuntimeError(
                    f"Cannot identify conf/DFL outputs from shapes: "
                    f"{[(o.name, tuple(o.shape)) for o in self._infer_model.outputs]}"
                )

            inp_shape = tuple(self._infer_model.input().shape)  # (H, W, C)
            ih, iw = inp_shape[0], inp_shape[1]
            self._input_wh = (iw, ih)

            self._anchor_pts, self._anchor_strides = _build_anchor_grid(ih, iw)

            # Enter the configured context once and hold it open for the lifetime
            self._configured_cm = self._infer_model.configure()
            self._configured = self._configured_cm.__enter__()

            self._available = True
            log.info(
                "RobotDetector: loaded %s, input %dx%d, conf=%.2f, iou=%.2f",
                self._model_path, iw, ih, self._conf, self._iou,
            )
        except Exception as e:
            log.warning("RobotDetector: Hailo init failed (%s) — robot detection disabled", e)
            self._available = False

    # ── public API ────────────────────────────────────────────────────────────

    @property
    def available(self) -> bool:
        """True when the Hailo device and model are ready."""
        return self._available

    def detect(self, frame_rgb: np.ndarray) -> Optional[RobotDetection]:
        """Run detection on one RGB frame.

        Parameters
        ----------
        frame_rgb : np.ndarray (H, W, 3) uint8
            Camera frame in RGB format (as produced by _Camera).

        Returns
        -------
        RobotDetection with detected robots (may be empty list).
        None if Hailo is unavailable or inference fails.
        """
        if not self._available:
            return None

        now = time.monotonic()
        fps = 1.0 / max(now - self._t_prev, 1e-6)
        self._t_prev = now

        fh, fw = frame_rgb.shape[:2]
        iw, ih = self._input_wh

        resized = cv2.resize(frame_rgb, (iw, ih), interpolation=cv2.INTER_LINEAR)
        # Model expects uint8 0-255 range; pass as float32 (hailort quantises it)
        tensor = resized.astype(np.float32)

        conf_buf = np.empty(self._infer_model.output(self._conf_name).shape, dtype=np.float32)
        dfl_buf  = np.empty(self._infer_model.output(self._dfl_name).shape,  dtype=np.float32)

        try:
            with self._lock:
                bindings = self._configured.create_bindings(
                    input_buffers  = {self._input_name: tensor},
                    output_buffers = {self._conf_name: conf_buf,
                                      self._dfl_name:  dfl_buf},
                )
                self._configured.run([bindings], timeout=1000)
        except Exception as e:
            log.warning("RobotDetector: inference error: %s", e)
            return None

        robots = self._decode(conf_buf, dfl_buf, fw, fh)

        if self._draw:
            self._annotate(frame_rgb, robots)

        return RobotDetection(robots=robots, fps=round(fps, 1), timestamp=now)

    def stop(self) -> None:
        """Release the Hailo device."""
        if self._configured_cm is not None:
            try:
                self._configured_cm.__exit__(None, None, None)
            except Exception:
                pass
            self._configured_cm = None
            self._configured = None
        if self._device is not None:
            try:
                self._device.release()
            except Exception:
                pass
            self._device = None
        self._available = False

    # ── decoding ─────────────────────────────────────────────────────────────

    def _decode(self,
                conf_raw: np.ndarray,
                dfl_raw:  np.ndarray,
                frame_w:  int,
                frame_h:  int) -> List[DetectedRobot]:
        """Decode model outputs to DetectedRobot list.

        conf_raw : (1, N, 1) — per-anchor confidence, already sigmoid'd
        dfl_raw  : (16, N, 4) — DFL logits
        """
        conf = conf_raw.squeeze()                    # (N,)
        mask = conf >= self._conf
        if not mask.any():
            return []

        iw, ih = self._input_wh

        # Decode DFL → ltrb pixel distances, then filter
        ltrb_all = _dfl_decode(dfl_raw, self._anchor_strides)  # (N, 4)
        ltrb = ltrb_all[mask]                                    # (M, 4)
        pts  = self._anchor_pts[mask]                            # (M, 2) cx,cy
        conf = conf[mask]                                        # (M,)

        # ltrb → xyxy at model input resolution
        x1 = np.clip(pts[:, 0] - ltrb[:, 0], 0, iw)
        y1 = np.clip(pts[:, 1] - ltrb[:, 1], 0, ih)
        x2 = np.clip(pts[:, 0] + ltrb[:, 2], 0, iw)
        y2 = np.clip(pts[:, 1] + ltrb[:, 3], 0, ih)

        # NMS — cv2.dnn.NMSBoxes wants [x, y, w, h] top-left
        boxes  = [(float(a), float(b), float(c - a), float(d - b))
                  for a, b, c, d in zip(x1, y1, x2, y2)]
        scores = conf.tolist()

        indices = cv2.dnn.NMSBoxes(boxes, scores, self._conf, self._iou)
        if len(indices) == 0:
            return []
        flat = indices.flatten() if hasattr(indices, 'flatten') else list(indices)

        # Scale from model input to display frame
        sx = frame_w / iw
        sy = frame_h / ih

        robots = []
        for i in flat:
            bx, by, bw, bh = boxes[i]
            fx1 = max(0, int(bx * sx))
            fy1 = max(0, int(by * sy))
            fx2 = min(frame_w, int((bx + bw) * sx))
            fy2 = min(frame_h, int((by + bh) * sy))
            robots.append(DetectedRobot(
                center_x=(fx1 + fx2) // 2,
                center_y=(fy1 + fy2) // 2,
                x1=fx1, y1=fy1, x2=fx2, y2=fy2,
                confidence=float(scores[i]),
            ))

        return robots

    # ── annotation ───────────────────────────────────────────────────────────

    def _annotate(self, frame: np.ndarray, robots: List[DetectedRobot]) -> None:
        for r in robots:
            cv2.rectangle(frame, (r.x1, r.y1), (r.x2, r.y2), _BOX_COLOUR, _THICKNESS)
            label = f"robot {r.confidence:.2f}"
            (lw, lh), _ = cv2.getTextSize(label, _FONT, _FONT_SCALE, _THICKNESS)
            ly = max(r.y1 - 6, lh + 4)
            cv2.rectangle(frame, (r.x1, ly - lh - 4), (r.x1 + lw + 4, ly), _BOX_COLOUR, -1)
            cv2.putText(frame, label, (r.x1 + 2, ly - 2),
                        _FONT, _FONT_SCALE, _LABEL_COLOUR, 1, cv2.LINE_AA)

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

Hailo output format note
------------------------
  YOLOv8n exported with ultralytics (opset=11, simplify=True) and compiled
  with Hailo DFC produces one output tensor of shape (1, 5, N) or (1, N, 5)
  where N = 8400 anchor proposals and 5 = [cx, cy, w, h, conf].  Coordinates
  are in pixels at the model's input resolution (typically 640×640).

  _decode_outputs() handles both orientations and logs the actual shape on
  the first inference to help diagnose format mismatches.
"""

import logging
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

log = logging.getLogger(__name__)

# ── Hailo platform import (optional) ─────────────────────────────────────────
try:
    from hailo_platform import (
        VDevice, HailoStreamInterface, ConfigureParams,
        HailoSchedulingAlgorithm, FormatType, HEF, InferVStreams,
    )
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


# ── Detector ──────────────────────────────────────────────────────────────────

class RobotDetector:
    """YOLOv8n robot detector running on the Hailo-10H.

    Parameters
    ----------
    model_path : str
        Path to the compiled .hef file (built via tools/robot_detector_training.html).
    conf : float
        Confidence threshold (0–1).  Detections below this are discarded.
        Lower = more sensitive but more false positives.
    iou : float
        IoU threshold for NMS.  Overlapping boxes above this are merged.
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
        self._device     = None
        self._network_group          = None
        self._input_vstreams_params  = None
        self._output_vstreams_params = None
        self._input_wh: Optional[tuple] = None   # (W, H) for cv2.resize
        self._available  = False
        self._shape_logged = False   # log output shape on first inference

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

            info  = hef.get_input_vstream_infos()[0]
            shape = info.shape   # typically (H, W, C)
            if len(shape) == 3:
                self._input_wh = (int(shape[1]), int(shape[0]))   # (W, H) for cv2
            else:
                log.warning("RobotDetector: unexpected input shape %s — defaulting to 640×640", shape)
                self._input_wh = (640, 640)

            self._available = True
            log.info(
                "RobotDetector: loaded %s, input %dx%d, conf=%.2f, iou=%.2f",
                self._model_path, self._input_wh[0], self._input_wh[1],
                self._conf, self._iou,
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

        The frame is annotated in-place when draw=True.
        """
        if not self._available:
            return None

        now = time.monotonic()
        fps = 1.0 / max(now - self._t_prev, 1e-6)
        self._t_prev = now

        fh, fw = frame_rgb.shape[:2]
        iw, ih = self._input_wh

        resized = cv2.resize(frame_rgb, (iw, ih), interpolation=cv2.INTER_LINEAR)
        tensor  = (resized.astype(np.float32) / 255.0)[np.newaxis]   # (1, H, W, 3)

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
        except Exception as e:
            log.warning("RobotDetector: inference error: %s", e)
            return None

        robots = self._decode_outputs(results, fw, fh)

        if self._draw:
            self._annotate(frame_rgb, robots)

        return RobotDetection(robots=robots, fps=round(fps, 1), timestamp=now)

    def stop(self) -> None:
        """Release the Hailo device."""
        if self._device is not None:
            try:
                self._device.release()
            except Exception:
                pass
            self._device = None
        self._available = False

    # ── output decoding ───────────────────────────────────────────────────────

    def _decode_outputs(self,
                        results: dict,
                        frame_w: int,
                        frame_h: int) -> List[DetectedRobot]:
        """Decode Hailo output tensors to a list of DetectedRobot objects.

        Handles the two most common YOLOv8n output layouts from Hailo DFC:
          (batch, 5, N)   — channels-first  [cx, cy, w, h, conf]
          (batch, N, 5)   — channels-last

        For multi-output models the tensors are concatenated along the anchor
        axis before decoding.

        Coordinates are in pixels at the model input resolution and are scaled
        to the display frame size before returning.
        """
        # Log output tensor shapes once to help diagnose format issues
        if not self._shape_logged:
            for name, tensor in results.items():
                log.info("RobotDetector: output '%s' shape %s dtype %s",
                         name, tensor.shape, tensor.dtype)
            self._shape_logged = True

        tensors = list(results.values())

        # ── Consolidate to a single (5, N) tensor ─────────────────────────────
        if len(tensors) == 1:
            raw = tensors[0]
            if raw.ndim == 3:
                raw = raw[0]   # remove batch dim → (5, N) or (N, 5)
        else:
            # Multiple output heads — strip batch dim, normalise to (5, N), concatenate
            parts = []
            for t in tensors:
                if t.ndim == 3:
                    t = t[0]
                if t.ndim == 2 and t.shape[0] < t.shape[1]:
                    parts.append(t)          # already (5, N)
                elif t.ndim == 2:
                    parts.append(t.T)        # was (N, 5) — transpose
                else:
                    log.debug("RobotDetector: unexpected tensor ndim %d — skipping", t.ndim)
            if not parts:
                return []
            raw = np.concatenate(parts, axis=1)

        # Ensure (5, N) — if first dim is larger it's (N, 5), transpose
        if raw.ndim == 2 and raw.shape[0] > raw.shape[1]:
            raw = raw.T

        if raw.shape[0] < 5:
            log.warning("RobotDetector: output shape %s has fewer than 5 channels — "
                        "check model export and compilation", raw.shape)
            return []

        cx   = raw[0]
        cy   = raw[1]
        w    = raw[2]
        h    = raw[3]
        conf = raw[4]   # for a 1-class model; multi-class would need argmax

        # Confidence filter
        mask = conf >= self._conf
        if not mask.any():
            return []

        cx, cy, w, h, conf = cx[mask], cy[mask], w[mask], h[mask], conf[mask]

        # cx,cy,w,h → x1,y1,x2,y2 at model input resolution
        mw, mh = self._input_wh
        x1 = np.clip(cx - w / 2, 0, mw).astype(np.float32)
        y1 = np.clip(cy - h / 2, 0, mh).astype(np.float32)
        x2 = np.clip(cx + w / 2, 0, mw).astype(np.float32)
        y2 = np.clip(cy + h / 2, 0, mh).astype(np.float32)

        # NMS — cv2.dnn.NMSBoxes expects [x, y, w, h] with x,y = top-left
        boxes  = [(float(x), float(y), float(x2-x), float(y2-y))
                  for x, y, x2, y2 in zip(x1, y1, x2, y2)]
        scores = conf.tolist()

        indices = cv2.dnn.NMSBoxes(boxes, scores, self._conf, self._iou)
        if len(indices) == 0:
            return []

        flat = indices.flatten() if hasattr(indices, 'flatten') else list(indices)

        # Scale factors from model input to display frame
        sx = frame_w / mw
        sy = frame_h / mh

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

#!/usr/bin/env python3
"""
aruco_detector.py — ArUco marker detection: tag IDs, positions, distances, bearings.

Detects ArUco markers in a camera frame and reports each tag's pixel geometry
plus, when calibration is available, its metric distance and bearing.

Tag ID convention
-----------------
  Tag ID 0–99   : front face
  Tag ID 100–199: rear face   (ID = front_id + 100)

Gate pairing, post-role decisions (inside/outside), and navigation are the
responsibility of the consuming module (aruco_navigator.py).

Frames are expected in **RGB** format (as produced by robot.py's camera).

Usage::
    from aruco_detector import ArucoDetector

    detector = ArucoDetector()
    state = detector.detect(frame)   # annotates frame in-place; returns ArUcoState
    for tag in state.tags.values():
        print(f"Tag {tag.id}: centre ({tag.center_x}, {tag.center_y}), "
              f"dist={tag.distance}, bearing={tag.bearing}")
"""

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

# ── ArUco dictionary map ──────────────────────────────────────────────────────

ARUCO_DICT: Dict[str, int] = {
    "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":        cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":        cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000":       cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50":         cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":        cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250":        cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000":       cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50":         cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100":        cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250":        cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000":       cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50":         cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100":        cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250":        cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000":       cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# ── Colours (RGB order — robot.py frames are RGB888) ──────────────────────────

_RED   = (255, 0,   0)
_GREEN = (0,   255, 0)
_WHITE = (255, 255, 255)

# ── Drawing constants ─────────────────────────────────────────────────────────

_MARKER_BOX_THICKNESS  = 2
_MARKER_CENTER_RADIUS  = 4
_FONT                  = cv2.FONT_HERSHEY_SIMPLEX
_MARKER_FONT_SCALE     = 0.5
_MARKER_FONT_THICKNESS = 2
_FPS_FONT_SCALE        = 1.0
_FPS_FONT_THICKNESS    = 2
_FPS_POSITION          = (10, 30)


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class ArUcoTag:
    """Single detected ArUco marker with pixel geometry."""
    id:           int
    center_x:     int
    center_y:     int
    area:         int                  # bounding-box area in pixels²
    top_left:     Tuple[int, int]
    top_right:    Tuple[int, int]
    bottom_right: Tuple[int, int]
    bottom_left:  Tuple[int, int]
    distance:     float = None         # metres (None = unknown)
    bearing:      float = None         # degrees from camera centre (None = unknown)


@dataclass
class ArUcoState:
    """Snapshot of one detection pass."""
    tags:      Dict[int, ArUcoTag] = field(default_factory=dict)
    fps:       float               = 0.0
    timestamp: float               = 0.0


def merge_aruco_states(
    state_a: Optional[ArUcoState],
    state_b: Optional[ArUcoState],
) -> Optional[ArUcoState]:
    """Merge ArUco tag detections from two cameras into one ArUcoState.

    Intended for combining front-left and front-right camera detections so the
    navigator sees tags from both fields of view simultaneously.

    Tag deduplication: if the same tag ID appears in both states the tag with
    a known distance is preferred; if both or neither have a distance estimate
    the one with the larger bounding-box area wins (assumed closer / more
    centred in frame).

    Gate pairing is left to the consumer (aruco_navigator.py).

    Returns None if both inputs are None; returns the non-None input if only
    one is provided.
    """
    if state_a is None and state_b is None:
        return None
    if state_a is None:
        return state_b
    if state_b is None:
        return state_a

    # Merge tags: for duplicate IDs prefer the one with distance, then larger area.
    merged_tags: Dict[int, ArUcoTag] = dict(state_a.tags)
    for tag_id, tag_b in state_b.tags.items():
        if tag_id not in merged_tags:
            merged_tags[tag_id] = tag_b
        else:
            tag_a = merged_tags[tag_id]
            a_has_dist = tag_a.distance is not None
            b_has_dist = tag_b.distance is not None
            if b_has_dist and not a_has_dist:
                merged_tags[tag_id] = tag_b
            elif not b_has_dist and not a_has_dist and tag_b.area > tag_a.area:
                merged_tags[tag_id] = tag_b

    return ArUcoState(
        tags=merged_tags,
        fps=(state_a.fps + state_b.fps) / 2.0,
        timestamp=max(state_a.timestamp, state_b.timestamp),
    )


# ── Detector ──────────────────────────────────────────────────────────────────

class ArucoDetector:
    """
    Detects ArUco markers and gate pairs in RGB camera frames.

    Parameters
    ----------
    dict_name  : ArUco dictionary to use (default ``"DICT_4X4_1000"``)
    draw       : Annotate frames in-place (default True)
    show_fps   : Overlay FPS counter (default True)
    calib_file : Path to camera_cal.npz produced by tools/calibrate_camera.py.
                 When supplied, frames are undistorted before detection and
                 pose estimation (distance + bearing) is performed for each
                 marker using solvePnP.
    tag_size   : Physical side length of the printed ArUco marker in metres
                 (default 0.15 m).  Used for pose estimation; ignored when
                 no calib_file is supplied.
    area_k     : Constant for area-based distance fallback (default 0.0 = disabled).
                 Used when calib_file is absent or resolution mismatches.
                 Calibrate by placing the tag at a known distance D (metres),
                 reading the detected area A (pixels²), then setting::
                     area_k = D * sqrt(A)
                 Set via ``[aruco] area_k`` in robot.ini.
    hfov       : Horizontal field of view in degrees (default 0.0 = disabled).
                 Used to convert the tag's pixel x-offset into a bearing angle
                 when solvePnP pose estimation is not available.
                 Set via ``[aruco] hfov`` in robot.ini.
    """

    def __init__(self,
                 dict_name:  str   = "DICT_4X4_1000",
                 draw:       bool  = True,
                 show_fps:   bool  = True,
                 calib_file: str   = None,
                 tag_size:   float = 0.15,
                 area_k:     float = 0.0,
                 hfov:       float = 0.0):
        if dict_name not in ARUCO_DICT:
            raise ValueError(
                f"Unknown ArUco dictionary: {dict_name!r}. "
                f"Valid options: {sorted(ARUCO_DICT)}"
            )
        self._draw     = draw
        self._show_fps = show_fps
        self._area_k   = area_k   # >0 enables area-based distance fallback
        self._hfov     = hfov     # >0 enables pixel bearing fallback (degrees)
        _dictionary    = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])
        _parameters    = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(_dictionary, _parameters)
        self._t_prev   = time.monotonic()

        # 3-D object points for one square marker centred at the origin.
        # Corner order matches OpenCV ArUco: TL, TR, BR, BL.
        half = tag_size / 2.0
        self._obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        # Lens undistortion + pose estimation (both optional, need calib_file)
        self._map1        = None
        self._map2        = None
        self._cam_mtx     = None   # camera intrinsic matrix (loaded from calib)
        self._dist_zero   = np.zeros((4, 1), dtype=np.float32)  # zero after remap
        self._calib_size  = None   # (height, width) the maps were built for
        self._calib_warned = False
        if calib_file is not None:
            try:
                cal = np.load(calib_file)
                self._map1       = cal['map1']
                self._map2       = cal['map2']
                self._cam_mtx    = cal['camera_matrix'].astype(np.float32)
                self._calib_size = (self._map1.shape[0], self._map1.shape[1])
            except Exception as e:
                raise ValueError(f"Cannot load calibration file {calib_file!r}: {e}") from e

    # ── public API ────────────────────────────────────────────────────────────

    def detect(self, frame) -> ArUcoState:
        """
        Detect markers and gate pairs in *frame* (RGB numpy array).

        If ``draw=True``, the frame is annotated in-place:

        - Green bounding box + red centre dot + ID label for each marker
        - Vertical gate-centre line (blue = correct direction, red = reversed)
        - FPS counter in top-left corner

        Returns an :class:`ArUcoState` with all detected tags and gates.
        """
        now  = time.monotonic()
        dt   = now - self._t_prev
        fps  = 1.0 / dt if dt > 0 else 0.0
        self._t_prev = now

        # Undistort frame before detection if calibration was loaded.
        # remap() operates on the raw frame (RGB) and writes back in-place so
        # that any draw annotations also appear on the corrected image.
        # Skip undistortion if the frame size doesn't match the calibration maps
        # (e.g. camera configured at a different resolution than calibration).
        # pose_ok is True only when undistortion was applied — that's when the
        # camera_matrix is valid for solvePnP (no residual lens distortion).
        pose_ok = False
        if self._map1 is not None:
            fh, fw = frame.shape[:2]
            if (fh, fw) == self._calib_size:
                undistorted = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)
                frame[:] = undistorted
                pose_ok = True
            elif not self._calib_warned:
                import warnings
                warnings.warn(
                    f"ArucoDetector: calibration maps are {self._calib_size[1]}×{self._calib_size[0]} "
                    f"but frame is {fw}×{fh} — undistortion skipped. "
                    f"Re-calibrate at {fw}×{fh} or update calib_file.",
                    UserWarning, stacklevel=2,
                )
                self._calib_warned = True

        grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self._detector.detectMarkers(grey)

        tags: Dict[int, ArUcoTag] = {}

        if ids is not None and len(ids) > 0:
            for marker_corner, marker_id in zip(corners, ids.flatten()):
                tag = self._process_marker(frame, marker_corner, int(marker_id), pose_ok)
                tags[tag.id] = tag

        if self._draw and self._show_fps:
            cv2.putText(frame, f"FPS:{fps:.1f}", _FPS_POSITION,
                        _FONT, _FPS_FONT_SCALE, _WHITE, _FPS_FONT_THICKNESS)

        return ArUcoState(tags=tags, fps=round(fps, 1), timestamp=now)

    # ── private helpers ───────────────────────────────────────────────────────

    def _process_marker(self, frame, marker_corner, marker_id: int,
                        pose_ok: bool) -> ArUcoTag:
        pts = marker_corner.reshape((4, 2))
        tl  = (int(pts[0][0]), int(pts[0][1]))
        tr  = (int(pts[1][0]), int(pts[1][1]))
        br  = (int(pts[2][0]), int(pts[2][1]))
        bl  = (int(pts[3][0]), int(pts[3][1]))
        cx  = (tl[0] + br[0]) // 2
        cy  = (tl[1] + br[1]) // 2
        area = abs(tl[0] - br[0]) * abs(tl[1] - br[1])

        # Pose estimation — only valid after undistortion (pose_ok=True).
        # solvePnP returns the translation vector in camera coordinates:
        #   t[0] = right, t[1] = down, t[2] = forward (optical axis).
        # distance = Euclidean distance to marker centre.
        # bearing  = horizontal angle (positive = right of boresight).
        distance: Optional[float] = None
        bearing:  Optional[float] = None
        if pose_ok and self._cam_mtx is not None:
            img_pts = pts.astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                self._obj_pts, img_pts,
                self._cam_mtx, self._dist_zero,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if ok:
                t        = tvec.flatten()
                distance = float(np.linalg.norm(t))
                bearing  = float(math.degrees(math.atan2(t[0], t[2])))

        # ── Fallbacks when solvePnP is unavailable ────────────────────────────
        # Area-based distance: distance ≈ area_k / sqrt(area_px²).
        # Calibrate once: place tag at known distance D, read area A, then
        #   area_k = D * sqrt(A).
        # Pixel bearing: converts x-offset from frame centre to degrees using
        #   the camera's horizontal field of view (hfov).
        # Both are approximate but good enough for navigation without a
        # calibration file.
        if distance is None and self._area_k > 0.0 and area > 0:
            distance = self._area_k / math.sqrt(area)
        if bearing is None and self._hfov > 0.0:
            fw = frame.shape[1]
            bearing = ((cx - fw / 2.0) / fw) * self._hfov

        if self._draw:
            cv2.line(frame, tl, tr, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, tr, br, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, br, bl, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.line(frame, bl, tl, _GREEN, _MARKER_BOX_THICKNESS)
            cv2.circle(frame, (cx, cy), _MARKER_CENTER_RADIUS, _RED, -1)
            label = str(marker_id) if distance is None else f"{marker_id} {distance:.2f}m"
            cv2.putText(frame, label,
                        (tl[0], tl[1] - 15),
                        _FONT, _MARKER_FONT_SCALE, _GREEN, _MARKER_FONT_THICKNESS)

        return ArUcoTag(id=marker_id, center_x=cx, center_y=cy, area=area,
                        top_left=tl, top_right=tr, bottom_right=br, bottom_left=bl,
                        distance=distance, bearing=bearing)


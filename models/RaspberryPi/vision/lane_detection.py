"""
lane_detection.py
-------------------
Classical HSV-based lane detection. Produces the control-facing state
described in documentation/software_architecture.md:

    lane_error   : float  (normalized, -1.0 .. 1.0, negative = lane left of center)
    curvature    : float  (normalized estimate of upcoming turn direction/sharpness)
    confidence   : float  (0.0 .. 1.0)

Design choice: this module does NOT try to fit a full polynomial lane
model or track both lane boundaries independently. It looks for the
drivable (non-boundary) region within the lane ROI and estimates the
centroid of that region at two heights (near/far) -- this is enough to
derive both a lateral error and a coarse curvature estimate, at a
fraction of the computational cost of a full lane-fit pipeline, which
matches the "control-oriented perception" strategy from the
architecture document.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from raspberry_pi.vision.hsv_config import as_numpy_bounds


@dataclass
class LaneEstimate:
    lane_error: float
    curvature: float
    confidence: float


def _roi_slice(frame_height, y_range):
    y0 = int(frame_height * y_range[0])
    y1 = int(frame_height * y_range[1])
    return y0, y1


def _drivable_mask(hsv_roi, boundary_bounds):
    """
    The lane boundary color (typically black/dark track border) is
    thresholded and then INVERTED: what's left is the drivable surface.
    This is more robust than trying to threshold the drivable surface
    color directly, since that color is usually just "not boundary".
    """
    lower, upper = as_numpy_bounds(boundary_bounds)
    boundary_mask = cv2.inRange(hsv_roi, lower, upper)
    drivable_mask = cv2.bitwise_not(boundary_mask)

    kernel = np.ones((5, 5), np.uint8)
    drivable_mask = cv2.morphologyEx(drivable_mask, cv2.MORPH_OPEN, kernel)
    drivable_mask = cv2.morphologyEx(drivable_mask, cv2.MORPH_CLOSE, kernel)
    return drivable_mask


def _row_centroid_x(mask_row: np.ndarray):
    """Return the x-centroid of nonzero pixels in a single mask row, or None."""
    xs = np.nonzero(mask_row)[0]
    if xs.size == 0:
        return None
    return float(xs.mean())


def estimate_lane(frame: np.ndarray, hsv_thresholds: dict,
                   lane_roi_y_range=(0.55, 1.0)) -> LaneEstimate:
    """
    Main entry point used by state_machine/fsm.py during FOLLOW_TRACK.
    """
    h, w = frame.shape[:2]
    y0, y1 = _roi_slice(h, lane_roi_y_range)
    roi_bgr = frame[y0:y1, :]
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    mask = _drivable_mask(roi_hsv, hsv_thresholds["lane_boundary"])
    roi_h = mask.shape[0]

    # Sample two rows: near (bottom of ROI) and far (top of ROI) to
    # derive both lateral error and a coarse curvature estimate.
    near_row = mask[int(roi_h * 0.9), :]
    far_row = mask[int(roi_h * 0.1), :]

    near_cx = _row_centroid_x(near_row)
    far_cx = _row_centroid_x(far_row)

    image_center_x = w / 2.0

    if near_cx is None and far_cx is None:
        return LaneEstimate(lane_error=0.0, curvature=0.0, confidence=0.0)

    # Fall back to whichever row is available if the other is empty
    # (e.g. sharp corner pushing the lane out of one sample row).
    if near_cx is None:
        near_cx = far_cx
    if far_cx is None:
        far_cx = near_cx

    lane_center_x = near_cx  # near row is authoritative for lateral error
    raw_error = image_center_x - lane_center_x
    lane_error = float(np.clip(raw_error / image_center_x, -1.0, 1.0))

    # Curvature proxy: how much the far-row centroid diverges from the
    # near-row centroid, normalized. A straight lane keeps both aligned;
    # an upcoming turn shifts the far centroid sideways first.
    raw_curvature = (far_cx - near_cx) / image_center_x
    curvature = float(np.clip(raw_curvature, -1.0, 1.0))

    # Confidence: fraction of sampled rows that found a drivable region
    # at all. A cheap but effective proxy for "do I trust this frame".
    hits = int(near_row.any()) + int(far_row.any())
    confidence = hits / 2.0

    return LaneEstimate(lane_error=lane_error, curvature=curvature,
                         confidence=confidence)

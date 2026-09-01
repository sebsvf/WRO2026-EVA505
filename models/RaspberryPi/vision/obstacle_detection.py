"""
obstacle_detection.py
------------------------
Detects the WRO Future Engineers traffic pillars (red = keep right,
green = keep left) via HSV color segmentation + contour analysis.

Output matches the control-facing schema from
documentation/software_architecture.md:

    pillar_detected : bool
    pillar_color    : "red" | "green" | None
    pillar_bbox     : (x, y, w, h) in the *pillar ROI* pixel space, or None
    pillar_area     : float  (proxy for distance -- larger area = closer)
    confidence      : float
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from raspberry_pi.vision.hsv_config import as_numpy_bounds


@dataclass
class PillarEstimate:
    pillar_detected: bool
    pillar_color: Optional[str]
    pillar_bbox: Optional[Tuple[int, int, int, int]]
    pillar_area: float
    confidence: float


def _largest_contour_bbox(mask, min_area=150):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return None, 0.0

    x, y, w, h = cv2.boundingRect(largest)
    return (x, y, w, h), float(area)


def estimate_pillar(frame: np.ndarray, hsv_thresholds: dict,
                     pillar_roi_y_range=(0.30, 0.85),
                     min_area=150) -> PillarEstimate:
    h, w = frame.shape[:2]
    y0 = int(h * pillar_roi_y_range[0])
    y1 = int(h * pillar_roi_y_range[1])
    roi_bgr = frame[y0:y1, :]
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    kernel = np.ones((5, 5), np.uint8)

    # Red wraps around hue 0/180 in OpenCV, so it is defined as two
    # ranges in config.yaml (pillar_red + pillar_red_high) that get
    # OR-ed together here.
    lower_r1, upper_r1 = as_numpy_bounds(hsv_thresholds["pillar_red"])
    lower_r2, upper_r2 = as_numpy_bounds(hsv_thresholds["pillar_red_high"])
    red_mask = cv2.bitwise_or(cv2.inRange(roi_hsv, lower_r1, upper_r1),
                               cv2.inRange(roi_hsv, lower_r2, upper_r2))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    lower_g, upper_g = as_numpy_bounds(hsv_thresholds["pillar_green"])
    green_mask = cv2.inRange(roi_hsv, lower_g, upper_g)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    red_bbox, red_area = _largest_contour_bbox(red_mask, min_area)
    green_bbox, green_area = _largest_contour_bbox(green_mask, min_area)

    # If both colors produced a candidate (shouldn't normally happen -
    # only one pillar is expected in the ROI at a time), trust the
    # larger (closer) one.
    if red_bbox is not None and (green_bbox is None or red_area >= green_area):
        bbox, area, color = red_bbox, red_area, "red"
    elif green_bbox is not None:
        bbox, area, color = green_bbox, green_area, "green"
    else:
        return PillarEstimate(pillar_detected=False, pillar_color=None,
                               pillar_bbox=None, pillar_area=0.0,
                               confidence=0.0)

    roi_area = roi_bgr.shape[0] * roi_bgr.shape[1]
    # Confidence grows with relative contour size, saturating at 1.0.
    confidence = float(min(1.0, area / (0.02 * roi_area)))

    return PillarEstimate(pillar_detected=True, pillar_color=color,
                           pillar_bbox=bbox, pillar_area=area,
                           confidence=confidence)

"""Visual parking evidence only. No encoder, odometry or inferred speed."""

from dataclasses import dataclass

import cv2
import numpy as np

from .common import KERNEL, frame_hsv, roi_slice
from .hsv_config import color_mask


@dataclass
class ParkingEstimate:
    parking_lot_detected: bool = False
    parking_lot_position: tuple[int, int] | None = None
    parking_offset: float = 0.0
    confidence: float = 0.0
    marker_count: int = 0
    bottom_fraction: float = 0.0


def estimate_parking_zone(
    frame, hsv_thresholds, min_area=200, *, hsv=None, roi_y_range=(0.35, 1.0)
):
    h, w = frame.shape[:2]
    y0, y1 = roi_slice(h, roi_y_range)
    roi = frame_hsv(frame, hsv)[y0:y1]
    mask = color_mask(roi, hsv_thresholds["parking_marker"])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < max(12, min_area * h * w / (640 * 480)):
            continue
        x, y, bw, bh = cv2.boundingRect(contour)
        candidates.append((area, x, y + y0, bw, bh))
    if not candidates:
        return ParkingEstimate()
    candidates.sort(reverse=True)
    # Two strongest markers define a tentative parking center, not proof of a parked car.
    chosen = candidates[:2]
    cx = float(np.mean([x + bw / 2 for _, x, y, bw, bh in chosen]))
    cy = float(np.mean([y + bh / 2 for _, x, y, bw, bh in chosen]))
    confidence = min(1.0, sum(item[0] for item in chosen) / (0.015 * h * w))
    if len(chosen) < 2:
        confidence = min(confidence, 0.50)
    bottom = max(y + bh for _, x, y, bw, bh in chosen) / h
    return ParkingEstimate(
        True,
        (int(cx), int(cy)),
        float(np.clip((cx - w / 2) / (w / 2), -1, 1)),
        confidence,
        len(chosen),
        bottom,
    )

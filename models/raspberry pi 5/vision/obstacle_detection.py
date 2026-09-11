"""HSV pillar detection, full-frame coordinates and nearest-first selection."""

from dataclasses import dataclass

import cv2
import numpy as np

from .common import KERNEL, frame_hsv, roi_slice
from .hsv_config import color_mask


@dataclass
class PillarEstimate:
    pillar_detected: bool = False
    pillar_color: str | None = None
    pillar_bbox: tuple[int, int, int, int] | None = None
    pillar_area: float = 0.0
    confidence: float = 0.0
    offset: float = 0.0
    center_x: int | None = None


def estimate_pillars(
    frame, hsv_thresholds, pillar_roi_y_range=(0.20, 0.98), min_area=80, *, hsv=None
):
    height, width = frame.shape[:2]
    y0, y1 = roi_slice(height, pillar_roi_y_range)
    roi = frame_hsv(frame, hsv)[y0:y1]
    result = []
    scale = (height * width) / (640 * 480)
    red = color_mask(roi, hsv_thresholds["pillar_red"])
    if "pillar_red_high" in hsv_thresholds:
        red |= color_mask(roi, hsv_thresholds["pillar_red_high"])
    green = color_mask(roi, hsv_thresholds["pillar_green"])
    wall = color_mask(roi, hsv_thresholds["lane_boundary"])
    for color, mask in (("red", red), ("green", green)):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            if area < max(12, min_area * scale) or h < max(8, height * 0.025):
                continue
            aspect = h / w
            if w > width * 0.40 or not 1.15 <= aspect <= 14:
                continue
            solidity = area / (w * h)
            if solidity < 0.45:
                continue
            # Reject colored chairs/signs ABOVE the track wall. Look immediately
            # beside the object because its own pixels occlude the wall behind it.
            margin = max(3, int(width * 0.025))
            nearby = np.concatenate(
                (wall[:, max(0, x - margin) : x], wall[:, x + w : min(width, x + w + margin)]),
                axis=1,
            )
            wall_rows = np.flatnonzero(np.mean(nearby > 0, axis=1) > 0.50) if nearby.size else []
            if len(wall_rows) and y + h < wall_rows[-1] - height * 0.025:
                continue
            confidence = float(
                np.clip(
                    0.45 * solidity
                    + 0.30 * min(1, aspect / 2.5)
                    + 0.25 * min(1, area / max(1, 1400 * scale)),
                    0,
                    1,
                )
            )
            cx = int(x + w / 2)
            result.append(
                PillarEstimate(
                    True,
                    color,
                    (x, y + y0, w, h),
                    float(area),
                    confidence,
                    (cx - width / 2) / (width / 2),
                    cx,
                )
            )
    # Ground contact nearer the bottom is a proximity cue, not a metric distance.
    return sorted(
        result, key=lambda p: (p.pillar_bbox[1] + p.pillar_bbox[3], p.pillar_area), reverse=True
    )


def estimate_pillar(
    frame, hsv_thresholds, pillar_roi_y_range=(0.20, 0.98), min_area=80, *, hsv=None
):
    found = estimate_pillars(frame, hsv_thresholds, pillar_roi_y_range, min_area, hsv=hsv)
    return found[0] if found else PillarEstimate()

"""Multi-row corridor detector; positive error AND curvature mean right.

Requires visible dark boundaries. An all-white image is unknown, not a lane.
Independent samples and contiguous gaps avoid averaging disconnected floor patches.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from .common import KERNEL, frame_hsv, roi_slice
from .hsv_config import color_mask


@dataclass
class LaneEstimate:
    lane_error: float = 0.0
    curvature: float = 0.0
    confidence: float = 0.0


def _corridor(row, expected, min_width):
    dark = row > 0
    # Runs of non-boundary pixels, with endpoints in pixel coordinates.
    edges = np.diff(np.r_[False, ~dark, False].astype(np.int8))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    candidates = []
    for start, end in zip(starts, ends, strict=True):
        width = end - start
        sides = int(start > 0) + int(end < len(row))
        if sides == 0 or width < min_width:
            continue
        center = (start + end - 1) / 2
        # A single wall only provides partial evidence; assume provisional lane width.
        if sides == 1:
            half_width = len(row) * 0.30
            center = start + half_width if start else end - half_width
        score = abs(center - expected) - 0.08 * width
        candidates.append((score, center, sides))
    if not candidates:
        return None
    _, center, sides = min(candidates)
    return float(center), 1.0 if sides == 2 else 0.45


def _wall_profile(dark, hsv_roi):
    """Low camera fallback: a side wall may never reach the lower scan rows.

    Select an opening from the wall/floor contact profile. This is an image
    geometry heuristic, not a calibrated probability or metric depth estimate.
    """
    h, w = dark.shape
    floor = (hsv_roi[-max(3, h // 5) :, :, 2] > 90) & (hsv_roi[-max(3, h // 5) :, :, 1] < 120)
    if float(floor.mean()) < 0.40:
        return LaneEstimate()
    walls = np.zeros_like(dark)
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, bw, bh = cv2.boundingRect(contour)
        # A wall must connect to the image edge; isolated dark furniture is excluded.
        if (
            (x <= 2 or x + bw >= w - 2)
            and bw >= w * 0.20
            and cv2.contourArea(contour) >= w * h * 0.01
        ):
            cv2.drawContours(walls, [contour], -1, 255, -1)
    if not walls.any():
        return LaneEstimate()
    base = np.max(np.where(walls > 0, np.arange(h)[:, None], -1), axis=0).astype(np.float32)
    kernel_width = max(3, (w // 25) | 1)
    base = cv2.blur(base[None, :], (kernel_width, 1))[0]
    contrast = float(np.percentile(base, 80) - np.percentile(base, 20))
    if contrast < h * 0.07:
        return LaneEstimate()  # Flat wall: no evidence of which way to turn.
    openings = base <= np.percentile(base, 25) + h * 0.025
    edges = np.diff(np.r_[False, openings, False].astype(np.int8))
    runs = [
        (b - a, (a + b - 1) / 2)
        for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1), strict=True)
        if b - a >= w * 0.12
    ]
    if not runs:
        return LaneEstimate()
    runs.sort(reverse=True)
    if len(runs) > 1 and runs[1][0] > runs[0][0] * 0.8:
        return LaneEstimate()  # Two comparable openings: don't average through the wall.
    width, center = runs[0]
    error = float(np.clip((center - (w - 1) / 2) / (w / 2), -1, 1))
    confidence = min(0.70, 0.55 + 0.15 * min(1, contrast / (h * 0.3)))
    return LaneEstimate(error, error * 0.5, confidence)


def estimate_lane(frame, hsv_thresholds, lane_roi_y_range=(0.35, 0.95), *, hsv=None):
    h, w = frame.shape[:2]
    y0, y1 = roi_slice(h, lane_roi_y_range)
    roi = frame_hsv(frame, hsv)[y0:y1]
    dark = color_mask(roi, hsv_thresholds["lane_boundary"])
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, KERNEL)
    centers, weights, heights = [], [], []
    expected = (w - 1) / 2
    # Bottom to top: prefer a spatially continuous corridor.
    for y in np.linspace(dark.shape[0] - 2, 1, 9).astype(int):
        row = np.median(dark[max(0, y - 1) : y + 2], axis=0)
        found = _corridor(row, expected, max(5, int(w * 0.12)))
        if found:
            center, weight = found
            if centers and abs(center - expected) > w * 0.40:
                continue
            centers.append(center)
            weights.append(weight)
            heights.append(y)
            expected = center
    fallback = _wall_profile(dark, roi) if sum(weights) / 9 < 0.55 else LaneEstimate()
    if len(centers) < 2:
        return fallback
    near = float(np.median(centers[: min(3, len(centers))]))
    far = float(np.median(centers[-min(3, len(centers)) :]))
    confidence = float(sum(weights) / 9)
    # No nearby sample: don't trust only a distant slit in the wall.
    if max(heights) < dark.shape[0] * 0.5:
        confidence *= 0.5
    estimate = LaneEstimate(
        float(np.clip((near - (w - 1) / 2) / (w / 2), -1, 1)),
        float(np.clip((far - near) / (w / 2), -1, 1)),
        confidence,
    )
    return fallback if fallback.confidence > estimate.confidence else estimate

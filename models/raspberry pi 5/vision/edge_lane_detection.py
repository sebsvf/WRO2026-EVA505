"""Canny/Hough diagnostic, using x(y) to handle vertical walls."""

from dataclasses import dataclass

import cv2
import numpy as np

from .common import validate_frame


@dataclass
class LaneEstimate:
    lane_error: float
    curvature: float
    confidence: float
    left_line: tuple | None
    right_line: tuple | None


def estimate_lane(frame):
    validate_frame(frame)
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
    edges[: int(h * 0.35)] = 0  # Mask AFTER Canny: no artificial ROI edge.
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, 20, minLineLength=max(12, int(h * 0.08)), maxLineGap=int(h * 0.1)
    )
    sides = [[], []]
    if lines is not None:
        for raw in lines.reshape(-1, 4):
            x1, y1, x2, y2 = map(int, raw)
            if abs(y2 - y1) < max(5, abs(x2 - x1) * 0.35):
                continue
            slope = (x2 - x1) / (y2 - y1)
            intercept = x1 - slope * y1
            bottom = slope * (h * 0.85) + intercept
            if not -w * 0.25 <= bottom <= w * 1.25:
                continue
            side = 0 if bottom < w / 2 else 1
            sides[side].append(
                (float(np.hypot(x2 - x1, y2 - y1)), slope, intercept, (x1, y1, x2, y2))
            )
    best = [max(items, key=lambda v: v[0]) if items else None for items in sides]
    left, right = best
    if left is None or right is None:
        return LaneEstimate(
            0,
            0,
            0.3 if left or right else 0,
            left[3] if left else None,
            right[3] if right else None,
        )
    near = [item[1] * h * 0.85 + item[2] for item in best]
    far = [item[1] * h * 0.45 + item[2] for item in best]
    if not w * 0.12 < near[1] - near[0] < w * 1.5 or far[0] >= far[1]:
        return LaneEstimate(0, 0, 0, left[3], right[3])
    center_near, center_far = np.mean(near), np.mean(far)
    return LaneEstimate(
        float(np.clip((center_near - w / 2) / (w / 2), -1, 1)),
        float(np.clip((center_far - center_near) / (w / 2), -1, 1)),
        1.0,
        left[3],
        right[3],
    )

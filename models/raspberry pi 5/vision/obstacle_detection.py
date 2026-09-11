"""
obstacle_detection.py

WRO pillar detector:
red = keep right
green = keep left

HSV segmentation + contour geometry scoring
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import logging

import cv2
import numpy as np

from hsv_config import as_numpy_bounds


logger = logging.getLogger("pillar_detection")


@dataclass
class PillarEstimate:
    pillar_detected: bool
    pillar_color: Optional[str]
    pillar_bbox: Optional[Tuple[int, int, int, int]]
    pillar_area: float
    confidence: float
    offset: float
    center_x: Optional[int]


def _find_best_pillar(mask, y_offset=0, min_area=80):

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    best_candidate = None
    best_score = 0.0

    img_width = mask.shape[1]


    for contour in contours:

        area = cv2.contourArea(contour)

        if area < min_area:
            continue


        x, y, w, h = cv2.boundingRect(contour)


        if w < 5 or h < 20:
            continue


        if w > mask.shape[1] * 0.35:
            continue


        aspect_ratio = h / max(w, 1)


        if aspect_ratio < 2.0 or aspect_ratio > 12:
            continue


        aspect_score = min(
            aspect_ratio / 5.0,
            1.0
        )


        area_score = min(
            area / 2500.0,
            1.0
        )


        position_score = 1.0

        if x <= 15 or x+w >= img_width-15:
            position_score = 0.2


        score = (
            0.5 * aspect_score +
            0.3 * area_score +
            0.2 * position_score
        )


        if score > best_score:

            best_score = score

            best_candidate = (
                x,
                y,
                w,
                h,
                area,
                score
            )


    if best_candidate is None:
        return None, 0.0, 0.0


    x, y, w, h, area, score = best_candidate

    y += y_offset


    return (
        (x, y, w, h),
        float(area),
        float(score)
    )



def estimate_pillar(
        frame: np.ndarray,
        hsv_thresholds: dict,
        pillar_roi_y_range=(0.25, 0.85),
        min_area=80
):

    height, width = frame.shape[:2]


    y0 = int(height * pillar_roi_y_range[0])
    y1 = int(height * pillar_roi_y_range[1])


    roi = frame[y0:y1, :]


    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )


    kernel = np.ones(
        (5,5),
        np.uint8
    )


    # =================
    # RED MASK
    # =================

    r1_low, r1_high = as_numpy_bounds(
        hsv_thresholds["pillar_red"]
    )

    r2_low, r2_high = as_numpy_bounds(
        hsv_thresholds["pillar_red_high"]
    )


    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, r1_low, r1_high),
        cv2.inRange(hsv, r2_low, r2_high)
    )


    red_mask = cv2.morphologyEx(
        red_mask,
        cv2.MORPH_OPEN,
        kernel
    )


    # =================
    # GREEN MASK
    # =================

    g_low, g_high = as_numpy_bounds(
        hsv_thresholds["pillar_green"]
    )


    green_mask = cv2.inRange(
        hsv,
        g_low,
        g_high
    )


    green_mask = cv2.morphologyEx(
        green_mask,
        cv2.MORPH_OPEN,
        kernel
    )


    red_bbox, red_area, red_score = _find_best_pillar(
        red_mask,
        y0,
        min_area
    )


    green_bbox, green_area, green_score = _find_best_pillar(
        green_mask,
        y0,
        min_area
    )


    # =================
    # SELECT BEST
    # =================

    if red_bbox is None and green_bbox is None:

        return PillarEstimate(
            False,
            None,
            None,
            0.0,
            0.0,
            0.0,
            None
        )


    if red_bbox is not None and green_bbox is None:

        bbox = red_bbox
        area = red_area
        confidence = red_score
        color = "red"


    elif green_bbox is not None and red_bbox is None:

        bbox = green_bbox
        area = green_area
        confidence = green_score
        color = "green"


    else:

        if red_score >= green_score:

            bbox = red_bbox
            area = red_area
            confidence = red_score
            color = "red"

        else:

            bbox = green_bbox
            area = green_area
            confidence = green_score
            color = "green"


    x, y, w, h = bbox


    # =================
    # POSITION ERROR
    # =================

    center_x = int(x + w / 2)

    image_center = width / 2


    offset = (
        center_x - image_center
    ) / image_center


    offset = float(
        np.clip(offset, -1.0, 1.0)
    )


    return PillarEstimate(
        True,
        color,
        bbox,
        area,
        float(confidence),
        offset,
        center_x
    )
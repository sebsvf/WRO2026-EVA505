"""
roi.py

Region Of Interest utilities.

The ROI limits the image area before vision processing.

Modules:
- Lane detection:
    Lower track surface.

- Obstacle detection:
    Large trapezoid covering pillar search area.

- Parking detection:
    Wide lower area because parking markers can appear laterally.
"""

import cv2
import numpy as np

# ==========================================================
# POLYGON DEFINITIONS
# ==========================================================


def create_lane_polygon(frame_shape):

    h, w = frame_shape[:2]

    return np.array(
        [
            # upper left
            (int(w * 0.20), int(h * 0.55)),
            # upper right
            (int(w * 0.80), int(h * 0.55)),
            # bottom right
            (w, h),
            # bottom left
            (0, h),
        ],
        dtype=np.int32,
    )


def create_obstacle_polygon(frame_shape):
    """
    ROI for WRO pillars.

    Larger than lane ROI because pillars
    must be detected before reaching them.

    Excludes extreme side regions to avoid
    walls and reflections.
    """

    h, w = frame_shape[:2]

    return np.array(
        [
            # far left upper point
            (int(w * 0.18), int(h * 0.18)),
            # far right upper point
            (int(w * 0.82), int(h * 0.18)),
            # near right
            (int(w * 0.95), int(h * 0.80)),
            # near left
            (int(w * 0.05), int(h * 0.80)),
        ],
        dtype=np.int32,
    )


def create_parking_polygon(frame_shape):
    """
    Parking marker search area.

    Wide because the parking zone can
    appear away from the center.
    """

    h, w = frame_shape[:2]

    return np.array([(0, int(h * 0.35)), (w, int(h * 0.35)), (w, h), (0, h)], dtype=np.int32)


# ==========================================================
# MASK CREATION
# ==========================================================


def create_mask(frame, polygon):
    """
    Creates binary ROI mask.

    Inside polygon:
        255

    Outside:
        0
    """

    mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    cv2.fillPoly(mask, [polygon], 255)

    return mask


def apply_roi(frame, polygon):
    """
    Applies polygon mask to frame.

    Pixels outside ROI become black.
    """

    mask = create_mask(frame, polygon)

    return cv2.bitwise_and(frame, frame, mask=mask)


# ==========================================================
# APPLY FUNCTIONS
# ==========================================================


def apply_lane_roi(frame):

    polygon = create_lane_polygon(frame.shape)

    return apply_roi(frame, polygon)


def apply_obstacle_roi(frame):

    polygon = create_obstacle_polygon(frame.shape)

    return apply_roi(frame, polygon)


def apply_parking_roi(frame):

    polygon = create_parking_polygon(frame.shape)

    return apply_roi(frame, polygon)


# ==========================================================
# DEBUG DRAWING
# ==========================================================


def draw_roi(frame, polygon):

    output = frame.copy()

    cv2.polylines(output, [polygon], True, (0, 255, 0), 2)

    return output


def draw_lane_roi(frame):

    return draw_roi(frame, create_lane_polygon(frame.shape))


def draw_obstacle_roi(frame):

    return draw_roi(frame, create_obstacle_polygon(frame.shape))


def draw_parking_roi(frame):

    return draw_roi(frame, create_parking_polygon(frame.shape))


def draw_all_rois(frame):

    output = frame.copy()

    output = draw_lane_roi(output)
    output = draw_obstacle_roi(output)
    output = draw_parking_roi(output)

    return output

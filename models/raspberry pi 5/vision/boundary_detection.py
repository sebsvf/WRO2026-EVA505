"""
boundary_detection.py

Detects WRO track boundaries (black walls)
and estimates the center of the drivable corridor.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class BoundaryEstimate:
    center_error: float
    confidence: float
    left_boundary: int | None
    right_boundary: int | None



def estimate_boundary(
        frame,
        roi_y_range=(0.25,0.75)
):

    h,w = frame.shape[:2]


    y0 = int(h*roi_y_range[0])
    y1 = int(h*roi_y_range[1])


    roi = frame[y0:y1]


    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )


    # Detect dark borders
    black_mask = cv2.inRange(
        gray,
        0,
        70
    )


    # Remove noise
    kernel=np.ones((5,5),np.uint8)

    black_mask=cv2.morphologyEx(
    black_mask,
    cv2.MORPH_CLOSE,
    kernel
)

    cv2.imshow(
    "BLACK MASK",
    black_mask
)
    cv2.waitKey(1)


    # Look at lower part of ROI
    scan_y=int(black_mask.shape[0]*0.75)


    row=black_mask[scan_y]


    xs=np.where(row>0)[0]


    if len(xs)<2:

        return BoundaryEstimate(
            0.0,
            0.0,
            None,
            None
        )


    # split left/right boundaries

    left_points=xs[xs<w/2]
    right_points=xs[xs>w/2]


    if len(left_points)==0 or len(right_points)==0:

        return BoundaryEstimate(
            0.0,
            0.3,
            None,
            None
        )


    left=int(np.max(left_points))
    right=int(np.min(right_points))


    lane_center=(left+right)/2


    error=(w/2-lane_center)/(w/2)


    confidence=min(
        1.0,
        abs(right-left)/w
    )


    return BoundaryEstimate(
        float(error),
        float(confidence),
        left,
        right
    )
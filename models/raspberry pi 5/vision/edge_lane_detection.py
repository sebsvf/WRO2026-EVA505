"""
edge_lane_detection.py

WRO Future Engineers lane detection.

Approach:
    - Canny edge detection
    - Hough line detection
    - Left/right boundary estimation
    - Lane center estimation

Output:
    lane_error : float (-1 to 1)
                  negative -> lane center is left
                  positive -> lane center is right

    curvature : float
                  rough turning direction

    confidence : float
"""


from dataclasses import dataclass

import cv2
import numpy as np



@dataclass
class LaneEstimate:

    lane_error: float
    curvature: float
    confidence: float

    left_line: tuple | None
    right_line: tuple | None




def _roi_mask(frame):

    """
    Keep only the region where the track exists.

    WRO camera is low mounted,
    so ignore sky/walls.
    """

    h,w = frame.shape[:2]


    mask = np.zeros_like(frame)


    polygon = np.array([
        [
            (0, int(h*0.35)),
            (w, int(h*0.35)),
            (w, h),
            (0, h)
        ]
    ])


    cv2.fillPoly(
        mask,
        polygon,
        (255,255,255)
    )


    return cv2.bitwise_and(
        frame,
        mask
    )




def _line_angle(x1,y1,x2,y2):

    return np.arctan2(
        y2-y1,
        x2-x1
    )





def estimate_lane(frame):


    h,w = frame.shape[:2]


    roi = _roi_mask(frame)


    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )


    blur=cv2.GaussianBlur(
        gray,
        (5,5),
        0
    )


    edges=cv2.Canny(
        blur,
        50,
        150
    )



    lines=cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi/180,
        threshold=20,
        minLineLength=30,
        maxLineGap=80
    )



    debug=frame.copy()



    if lines is None:

        return LaneEstimate(
            0.0,
            0.0,
            0.0,
            None,
            None
        )



    left_candidates=[]
    right_candidates=[]



    for line in lines:

        x1,y1,x2,y2=line[0]


        angle=_line_angle(
            x1,y1,x2,y2
        )


        angle_deg=np.degrees(angle)


        length=np.sqrt(
            (x2-x1)**2+
            (y2-y1)**2
        )


        # ignore horizontal lines
        if abs(angle_deg)<20:
            continue


        # left wall
        if angle_deg < -10:

            left_candidates.append(
                (
                    length,
                    (x1,y1,x2,y2)
                )
            )


        # right wall
        elif angle_deg > 10:

            right_candidates.append(
                (
                    length,
                    (x1,y1,x2,y2)
                )
            )




    left_line=None
    right_line=None



    if left_candidates:

        left_line=max(
            left_candidates,
            key=lambda x:x[0]
        )[1]


    if right_candidates:

        right_line=max(
            right_candidates,
            key=lambda x:x[0]
        )[1]



    # Draw debug

    if left_line:

        cv2.line(
            debug,
            left_line[:2],
            left_line[2:],
            (0,255,0),
            3
        )


    if right_line:

        cv2.line(
            debug,
            right_line[:2],
            right_line[2:],
            (0,0,255),
            3
        )



    cv2.imshow(
        "EDGE",
        edges
    )


    cv2.imshow(
        "HOUGH LINES",
        debug
    )



    if left_line is None or right_line is None:

        return LaneEstimate(
            0.0,
            0.0,
            0.3,
            left_line,
            right_line
        )



    # Estimate where the boundaries reach robot level

    y_target=int(h*0.85)



    def interpolate(line):

        x1,y1,x2,y2=line

        if y2==y1:
            return x1

        return int(
            x1+
            (y_target-y1)
            *
            (x2-x1)
            /
            (y2-y1)
        )



    left_x=interpolate(left_line)
    right_x=interpolate(right_line)



    lane_center=(
        left_x+
        right_x
    )/2



    image_center=w/2



    lane_error=(
        image_center-
        lane_center
    )/image_center



    lane_error=float(
        np.clip(
            lane_error,
            -1,
            1
        )
    )



    # rough curvature

    left_bottom=left_x
    right_bottom=right_x


    width_lane=right_bottom-left_bottom


    curvature=(
        width_lane-w
    )/w



    confidence=0.0


    if left_line:
        confidence+=0.5

    if right_line:
        confidence+=0.5



    return LaneEstimate(
        lane_error,
        float(curvature),
        confidence,
        left_line,
        right_line
    )
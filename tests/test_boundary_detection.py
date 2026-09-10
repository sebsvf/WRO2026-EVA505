import sys
from pathlib import Path
import cv2


ROOT = Path(__file__).resolve().parent.parent


VISION_PATH = (
    ROOT /
    "models" /
    "raspberry pi 5" /
    "vision"
)


sys.path.append(str(VISION_PATH))


from boundary_detection import estimate_boundary


# cambia esta imagen por cualquier pista
IMAGE_PATH = ("tests/datastets/obstacles/images/WIN_20260108_15_33_00_Pro_jpg.rf.924142b36d047a21115834482e0804d6.jpg")


frame = cv2.imread(IMAGE_PATH)

import cv2
import numpy as np


def detect_edges(frame):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )


    blur = cv2.GaussianBlur(
        gray,
        (5,5),
        0
    )


    edges = cv2.Canny(
        blur,
        50,
        150
    )


    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi/180,
        threshold=50,
        minLineLength=40,
        maxLineGap=20
    )


    output = frame.copy()


    if lines is not None:

        for line in lines:

            x1,y1,x2,y2=line[0]

            cv2.line(
                output,
                (x1,y1),
                (x2,y2),
                (0,255,0),
                2
            )


    cv2.imshow(
        "edges",
        edges
    )

    cv2.imshow(
        "lines",
        output
    )


    return output
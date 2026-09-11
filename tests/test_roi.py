import cv2
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent


VISION_PATH = (
    ROOT /
    "models" /
    "raspberry pi 5" /
    "vision"
)


sys.path.append(str(VISION_PATH))


from roi import (
    draw_lane_roi,
    apply_lane_roi,
    draw_obstacle_roi,
    apply_obstacle_roi
)

IMAGE_PATH = ("tests/datastets/obstacles/images 1/WIN_20260108_15_32_06_Pro_jpg.rf.c89a0bd98ce6fa72ebf777397988b1f2.jpg"
)


frame = cv2.imread(IMAGE_PATH)


if frame is None:
    raise FileNotFoundError(
        "No se pudo cargar la imagen"
    )


# Lane
lane_debug = draw_lane_roi(frame)
lane_mask = apply_lane_roi(frame)


# Obstacles
obstacle_debug = draw_obstacle_roi(frame)
obstacle_mask = apply_obstacle_roi(frame)



cv2.imshow(
    "LANE ROI",
    lane_debug
)


cv2.imshow(
    "LANE MASK",
    lane_mask
)


cv2.imshow(
    "OBSTACLE ROI",
    obstacle_debug
)


cv2.imshow(
    "OBSTACLE MASK",
    obstacle_mask
)


cv2.waitKey(0)
cv2.destroyAllWindows()
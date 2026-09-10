import sys
from pathlib import Path
import cv2


ROOT=Path(__file__).resolve().parent.parent


VISION_PATH=(
    ROOT /
    "models" /
    "raspberry pi 5" /
    "vision"
)


sys.path.append(
    str(VISION_PATH)
)


from edge_lane_detection import estimate_lane



IMAGE_PATH=(
"tests/datastets/obstacles/images/WIN_20260108_15_33_16_Pro_jpg.rf.74e2749c3ce40325b67da6f4c89cf98e.jpg"
)



frame=cv2.imread(
    IMAGE_PATH
)


result=estimate_lane(
    frame
)


print(result)


cv2.imshow(
    "original",
    frame
)


cv2.waitKey(0)
cv2.destroyAllWindows()
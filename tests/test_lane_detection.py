import sys
from pathlib import Path
import cv2
import numpy as np


# ==============================
# PATHS
# ==============================

ROOT = Path(__file__).resolve().parent.parent


VISION_PATH = (
    ROOT /
    "models" /
    "raspberry pi 5" /
    "vision"
)


CONFIG_PATH = (
    ROOT /
    "models" /
    "raspberry pi 5" /
    "config" /
    "config.yaml"
)


IMAGE_PATH = (
    "tests/datastets/obstacles/images/WIN_20260108_15_36_00_Pro-2-_jpg.rf.017c9e504754a746ed2efdf418440220.jpg"
)


# ==============================
# IMPORTS
# ==============================

sys.path.append(str(VISION_PATH))


from lane_detection import estimate_lane
from hsv_config import load_hsv_thresholds


# ==============================
# LOAD IMAGE
# ==============================

frame = cv2.imread(IMAGE_PATH)


if frame is None:
    raise Exception(
        f"No se pudo abrir la imagen: {IMAGE_PATH}"
    )


print("Imagen cargada correctamente")
print("Resolución:", frame.shape)


# ==============================
# LOAD CONFIG
# ==============================

thresholds = load_hsv_thresholds(
    str(CONFIG_PATH)
)


# ==============================
# CREATE MASKS
# ==============================

hsv = cv2.cvtColor(
    frame,
    cv2.COLOR_BGR2HSV
)


height, width = frame.shape[:2]


# mismo ROI que lane_detection
y0 = int(height * 0.55)
y1 = int(height * 1.0)


roi_hsv = hsv[y0:y1, :]


# negro/borde de pista
lower = np.array(
    [0, 0, 0]
)

upper = np.array(
    [180, 120, 120]
)


boundary_mask = cv2.inRange(
    roi_hsv,
    lower,
    upper
)


drivable_mask = cv2.bitwise_not(
    boundary_mask
)


# ==============================
# RUN DETECTOR
# ==============================

result = estimate_lane(
    frame,
    thresholds
)


print("\nRESULTADO:")
print(result)


# ==============================
# DEBUG IMAGE
# ==============================

debug = frame.copy()


cv2.rectangle(
    debug,
    (0, y0),
    (width, y1),
    (0,255,0),
    2
)


# ==============================
# SHOW
# ==============================

cv2.imshow(
    "LANE DETECTION",
    debug
)


cv2.imshow(
    "BOUNDARY MASK",
    boundary_mask
)


cv2.imshow(
    "DRIVABLE MASK",
    drivable_mask
)


cv2.waitKey(0)

cv2.destroyAllWindows()
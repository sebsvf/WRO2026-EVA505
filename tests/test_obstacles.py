import sys
from pathlib import Path
import cv2


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

IMAGE_PATH = ("tests/datastets/obstacles/images 1/WIN_20260108_15_31_15_Pro_jpg.rf.7046dbe756176b67329f0ca8c4ab2adf.jpg")


# ==============================
# IMPORTS
# ==============================

sys.path.append(str(VISION_PATH))

from obstacle_detection import estimate_pillar
from hsv_config import load_hsv_thresholds


# ==============================
# LOAD IMAGE
# ==============================

frame = cv2.imread(str(IMAGE_PATH))

import numpy as np

hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


# ==============================
# RED MASK
# ==============================

# Rojo tiene dos rangos en HSV (porque cruza 0/180)
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])


red_mask = cv2.bitwise_or(
    cv2.inRange(hsv, lower_red1, upper_red1),
    cv2.inRange(hsv, lower_red2, upper_red2)
)


# ==============================
# GREEN MASK
# ==============================

lower_green = np.array([45, 80, 60])
upper_green = np.array([85, 255, 255])


green_mask = cv2.inRange(
    hsv,
    lower_green,
    upper_green
)


# ==============================
# MORPHOLOGY (igual que detector)
# ==============================

kernel = np.ones((7,7), np.uint8)

red_mask = cv2.morphologyEx(
    red_mask,
    cv2.MORPH_CLOSE,
    kernel
)

green_mask = cv2.morphologyEx(
    green_mask,
    cv2.MORPH_CLOSE,
    kernel
)


# ==============================
# SHOW MASKS
# ==============================

cv2.imshow(
    "RED MASK",
    red_mask
)

cv2.imshow(
    "GREEN MASK",
    green_mask
)

if frame is None:
    raise Exception(
        f"No se pudo abrir la imagen: {IMAGE_PATH}"
    )


print("Imagen cargada correctamente")
print("Resolución:", frame.shape)


# ==============================
# LOAD HSV CONFIG
# ==============================

thresholds = load_hsv_thresholds(
    str(CONFIG_PATH)
)


# ==============================
# RUN DETECTOR
# ==============================

result = estimate_pillar(
    frame,
    thresholds
)


print("\nRESULTADO:")
print(result)

if result.pillar_detected:

    x, y, w, h = result.pillar_bbox

    cv2.rectangle(
        frame,
        (x, y),
        (x+w, y+h),
        (0,255,0),
        3
    )

    cv2.putText(
        frame,
        result.pillar_color,
        (x, y-10),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0,255,0),
        2
    )


# ==============================
# SHOW IMAGE
# ==============================

cv2.imshow(
    "Original",
    frame
)

cv2.waitKey(0)
cv2.destroyAllWindows()
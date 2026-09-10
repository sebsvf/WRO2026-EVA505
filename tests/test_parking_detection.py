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


CONFIG_PATH = (
    ROOT /
    "models" /
    "raspberry pi 5" /
    "config" /
    "config.yaml"
)


IMAGE_PATH = (
    "tests/datastets/obstacles/images 2/img_27_jpg.rf.4fc2ba0373ffc215b862d67c5612f1bb.jpg"
)


sys.path.append(
    str(VISION_PATH)
)


from parking_detection import estimate_parking_zone
from hsv_config import load_hsv_thresholds



frame = cv2.imread(
    IMAGE_PATH
)


if frame is None:
    raise Exception(
        f"No se pudo cargar imagen: {IMAGE_PATH}"
    )


thresholds = load_hsv_thresholds(
    str(CONFIG_PATH)
)


result = estimate_parking_zone(
    frame,
    thresholds
)


print("\nRESULTADO:")
print(result)



if result.parking_lot_detected:

    cx, cy = result.parking_lot_position


    cv2.circle(
        frame,
        (cx, cy),
        8,
        (0,255,0),
        -1
    )


    cv2.putText(
        frame,
        "PARKING",
        (cx, cy-15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0,255,0),
        2
    )



cv2.imshow(
    "PARKING TEST",
    frame
)


cv2.waitKey(0)
cv2.destroyAllWindows()
"""
hsv_config.py
--------------
Loading/saving of HSV threshold sets and a small interactive tuning
helper. Keeping this separate from lane_detection.py / obstacle_detection.py
means the thresholds can be recalibrated on competition day (different
mat, different lighting) without touching any detection logic.
"""

import yaml
import numpy as np


def load_hsv_thresholds(config_path="raspberry_pi/config/config.yaml"):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    return cfg["hsv_thresholds"]


def save_hsv_thresholds(thresholds, config_path="raspberry_pi/config/config.yaml"):
    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)
    cfg["hsv_thresholds"] = thresholds
    with open(config_path, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def as_numpy_bounds(bounds: dict):
    """Convert a {'lower': [...], 'upper': [...]} dict to np.uint8 arrays."""
    return (np.array(bounds["lower"], dtype=np.uint8),
            np.array(bounds["upper"], dtype=np.uint8))


def interactive_tune(camera, label, config_path="raspberry_pi/config/config.yaml"):
    """
    Minimal OpenCV trackbar-based tuner. Run this manually on the
    Raspberry Pi (with a display or VNC) the day of the competition to
    re-tune a single color range against the actual mat and lighting.

    Usage:
        python -m raspberry_pi.vision.hsv_config <label>

    Press 's' to save the current sliders into config.yaml, 'q' to quit
    without saving.
    """
    import cv2

    def _nothing(_):
        pass

    window = f"HSV tuning: {label}"
    cv2.namedWindow(window)
    for name, default in [("H min", 0), ("S min", 0), ("V min", 0),
                           ("H max", 180), ("S max", 255), ("V max", 255)]:
        cv2.createTrackbar(name, window, default, 255 if "H" not in name else 180, _nothing)

    while True:
        frame = camera.capture_frame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([cv2.getTrackbarPos("H min", window),
                           cv2.getTrackbarPos("S min", window),
                           cv2.getTrackbarPos("V min", window)])
        upper = np.array([cv2.getTrackbarPos("H max", window),
                           cv2.getTrackbarPos("S max", window),
                           cv2.getTrackbarPos("V max", window)])

        mask = cv2.inRange(hsv, lower, upper)
        preview = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow(window, preview)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            thresholds = load_hsv_thresholds(config_path)
            thresholds[label] = {"lower": lower.tolist(), "upper": upper.tolist()}
            save_hsv_thresholds(thresholds, config_path)
            print(f"Saved '{label}' = lower={lower.tolist()} upper={upper.tolist()}")
            break
        elif key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    import sys
    from raspberry_pi.camera.camera import Camera

    if len(sys.argv) != 2:
        print("Usage: python -m raspberry_pi.vision.hsv_config <threshold_label>")
        sys.exit(1)

    cam = Camera()
    cam.start()
    interactive_tune(cam, sys.argv[1])
    cam.stop()

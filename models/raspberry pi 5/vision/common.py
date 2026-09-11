import cv2
import numpy as np

KERNEL = np.ones((3, 3), np.uint8)


def validate_frame(frame):
    if (
        not isinstance(frame, np.ndarray)
        or frame.dtype != np.uint8
        or frame.ndim != 3
        or frame.shape[2] != 3
        or min(frame.shape[:2]) < 8
    ):
        raise ValueError("Expected nonempty BGR uint8 image (at least 8x8)")


def roi_slice(height, y_range):
    if (
        len(y_range) != 2
        or not all(np.isfinite(v) for v in y_range)
        or not 0 <= y_range[0] < y_range[1] <= 1
    ):
        raise ValueError("ROI must satisfy 0 <= top < bottom <= 1")
    y0, y1 = int(height * y_range[0]), int(height * y_range[1])
    if y1 - y0 < 3:
        raise ValueError("ROI must span at least three rows")
    return y0, y1


def frame_hsv(frame, hsv=None):
    validate_frame(frame)
    if hsv is None:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if hsv.shape != frame.shape or hsv.dtype != np.uint8:
        raise ValueError("HSV frame shape/type mismatch")
    return hsv

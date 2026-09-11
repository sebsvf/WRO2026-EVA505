
import cv2

from .common import frame_hsv, roi_slice
from .hsv_config import color_mask


def estimate_section_marker(frame, thresholds, *, hsv=None, y_range=(0.65, 0.90)):
    h, w = frame.shape[:2]
    y0, y1 = roi_slice(h, y_range)
    roi = frame_hsv(frame, hsv)[y0:y1]
    candidates = []
    for color in ("blue", "orange"):
        mask = color_mask(roi, thresholds[f"section_{color}"])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, bw, bh = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            # Floor stripes span the image; thin upright colored objects do not.
            if bw >= w * 0.25 and bw >= bh * 2 and area >= w * h * 0.001:
                candidates.append((y + bh, area, color))
    return max(candidates)[2] if candidates else None

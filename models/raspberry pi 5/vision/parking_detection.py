"""
parking_detection.py
-----------------------
MVP parking-zone detection and blind-approach dead reckoning.

STATUS: experimental / least mature module in this repository. Vision-
based parking marker detection is implemented and usable as-is; the
dead-reckoning fallback for the final blind approach (once the marker
leaves the camera's field of view) uses a simple constant-velocity
distance model and WILL need on-track tuning of PARKING_APPROACH_DISTANCE_MM
and the wheelbase-derived turning radius in control/steering_controller.py.

Output schema:
    parking_lot_detected : bool
    parking_lot_position : (x, y) centroid in full-frame pixel coords, or None
    parking_offset        : float, normalized lateral offset (like lane_error)
    confidence            : float
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from raspberry_pi.vision.hsv_config import as_numpy_bounds

# TODO: confirm against the actual parking marker width in the current
# WRO Future Engineers field spec before relying on this for the blind
# reverse distance estimate.
KNOWN_MARKER_WIDTH_MM = 200.0


@dataclass
class ParkingEstimate:
    parking_lot_detected: bool
    parking_lot_position: Optional[Tuple[int, int]]
    parking_offset: float
    confidence: float


def estimate_parking_zone(frame: np.ndarray, hsv_thresholds: dict,
                           min_area=200) -> ParkingEstimate:
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower, upper = as_numpy_bounds(hsv_thresholds["parking_marker"])
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return ParkingEstimate(False, None, 0.0, 0.0)

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return ParkingEstimate(False, None, 0.0, 0.0)

    x, y, bw, bh = cv2.boundingRect(largest)
    cx, cy = x + bw / 2.0, y + bh / 2.0

    image_center_x = w / 2.0
    offset = float(np.clip((image_center_x - cx) / image_center_x, -1.0, 1.0))

    confidence = float(min(1.0, area / (0.03 * w * h)))

    return ParkingEstimate(True, (int(cx), int(cy)), offset, confidence)


def estimate_distance_mm(bbox_width_px: float, frame_width_px: int,
                          horizontal_fov_deg: float = 62.2) -> float:
    """
    Very rough monocular distance estimate from apparent marker width,
    using the known real-world marker width and the camera's horizontal
    field of view (62.2 deg is the Raspberry Pi Camera V2.1 datasheet
    value). This is only accurate enough to decide "close enough to
    start the blind reverse maneuver" -- not for precise positioning.
    """
    if bbox_width_px <= 0:
        return float("inf")
    focal_px = (frame_width_px / 2.0) / np.tan(np.radians(horizontal_fov_deg / 2.0))
    return (KNOWN_MARKER_WIDTH_MM * focal_px) / bbox_width_px


class BlindApproachEstimator:
    """
    Once the parking marker leaves the FOV during the final reverse,
    this tracks distance traveled via encoder ticks (reported by the
    ESP32 over UART) to estimate progress through the blind portion of
    the maneuver. See documentation/software_architecture.md Sec 4.9.
    """

    def __init__(self, wheel_radius_mm: float = 32.5, ticks_per_rev: int = 20):
        # TODO: confirm ticks_per_rev against the Pololu encoder's actual
        # counts-per-revolution specification.
        self.wheel_radius_mm = wheel_radius_mm
        self.ticks_per_rev = ticks_per_rev
        self._start_ticks = None

    def start(self, current_ticks: int):
        self._start_ticks = current_ticks

    def distance_traveled_mm(self, current_ticks: int) -> float:
        if self._start_ticks is None:
            return 0.0
        delta_ticks = current_ticks - self._start_ticks
        revs = delta_ticks / self.ticks_per_rev
        return revs * 2 * np.pi * self.wheel_radius_mm

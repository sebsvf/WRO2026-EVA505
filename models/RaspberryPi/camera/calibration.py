import logging
import time

logger = logging.getLogger("calibration")


def lock_exposure_and_white_balance(camera, settle_time_s=2.0):
    """
    Let auto-exposure/AWB converge for `settle_time_s`, then read back
    whatever picamera2 landed on and lock it via set_manual_controls().

    Returns the (exposure_us, awb_gains) tuple that was applied, so the
    caller can persist it into config.yaml for the next run if desired.
    """
    logger.info("Letting auto-exposure/AWB converge for %.1fs...",
                settle_time_s)
    time.sleep(settle_time_s)

    exposure_us = None
    awb_gains = None
    if camera._picam2 is not None:  # not in simulated mode
        metadata = camera._picam2.capture_metadata()
        exposure_us = metadata.get("ExposureTime")
        awb_gains = metadata.get("ColourGains")

    camera.set_manual_controls(exposure_us=exposure_us, awb_gains=awb_gains)
    logger.info("Locked exposure=%s us, awb_gains=%s", exposure_us, awb_gains)
    return exposure_us, awb_gains


def verify_hsv_thresholds(frame, hsv_thresholds, min_pixels=25):
    """
    Sanity check performed during CALIBRATION: confirm that the
    configured HSV ranges actually match *something* in the current
    frame for each expected color (lane boundary, red pillar, green
    pillar). This does not guarantee correctness, but it catches the
    common failure mode of thresholds tuned on a different mat/lighting
    that now match nothing at all.

    Returns a dict {label: bool} indicating which colors were found.
    """
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = {}
    for label, bounds in hsv_thresholds.items():
        lower = np.array(bounds["lower"], dtype=np.uint8)
        upper = np.array(bounds["upper"], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        found = int(np.count_nonzero(mask)) >= min_pixels
        results[label] = found
        if not found:
            logger.warning(
                "HSV threshold '%s' matched no pixels in the calibration "
                "frame -- recalibrate with vision/hsv_config.py before "
                "running the FSM.", label)
    return results

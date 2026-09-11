"""Validated HSV intervals and config-relative calibration storage."""

import os
import tempfile
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "config.yaml"


@lru_cache(maxsize=128)
def _bounds(lower, upper):
    lo, hi = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if lo.shape != (3,) or hi.shape != (3,):
        raise ValueError("HSV bounds must each have three elements")
    for bound in (lo, hi):
        if (
            not np.all(np.isfinite(bound))
            or np.any(bound != np.floor(bound))
            or np.any(bound < 0)
            or np.any(bound > [179, 255, 255])
        ):
            raise ValueError("HSV range: H 0..179, S/V 0..255")
    if np.any(lo > hi):
        raise ValueError("HSV lower must not exceed upper")
    lo, hi = lo.astype(np.uint8), hi.astype(np.uint8)
    lo.flags.writeable = hi.flags.writeable = False
    return lo, hi


def as_numpy_bounds(bounds):
    return _bounds(tuple(bounds["lower"]), tuple(bounds["upper"]))


def color_mask(hsv, bounds):
    ranges = [bounds] if isinstance(bounds, dict) else bounds
    if not isinstance(ranges, (list, tuple)) or not ranges:
        raise ValueError("Missing HSV intervals")
    result = np.zeros(hsv.shape[:2], np.uint8)
    for interval in ranges:
        lower, upper = as_numpy_bounds(interval)
        cv2.bitwise_or(result, cv2.inRange(hsv, lower, upper), dst=result)
    return result


def load_hsv_thresholds(config_path=DEFAULT_CONFIG):
    cfg = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    thresholds = cfg["hsv_thresholds"]
    for bounds in thresholds.values():
        for interval in [bounds] if isinstance(bounds, dict) else bounds:
            as_numpy_bounds(interval)
    return thresholds


def save_hsv_thresholds(thresholds, config_path=DEFAULT_CONFIG):
    path = Path(config_path).resolve()
    for bounds in thresholds.values():
        for interval in [bounds] if isinstance(bounds, dict) else bounds:
            as_numpy_bounds(interval)
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    cfg["hsv_thresholds"] = thresholds
    fd, temporary = tempfile.mkstemp(dir=path.parent, suffix=".yaml.tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(cfg, stream, sort_keys=False, allow_unicode=True)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def interactive_tune(camera, label, config_path=DEFAULT_CONFIG):
    """Requires opencv-python GUI build instead of opencv-python-headless."""
    thresholds = load_hsv_thresholds(config_path)
    bounds = thresholds[label]
    if not isinstance(bounds, dict):
        raise ValueError("Tune each split interval separately")
    window = f"HSV: {label}"
    names = ["H min", "S min", "V min", "H max", "S max", "V max"]
    cv2.namedWindow(window)
    try:
        for index, (name, value) in enumerate(
            zip(names, bounds["lower"] + bounds["upper"], strict=True)
        ):
            cv2.createTrackbar(name, window, value, 179 if index % 3 == 0 else 255, lambda _: None)
        while True:
            frame = camera.capture_frame()
            values = [cv2.getTrackbarPos(name, window) for name in names]
            candidate = {"lower": values[:3], "upper": values[3:]}
            try:
                mask = color_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), candidate)
            except ValueError:
                mask = np.zeros(frame.shape[:2], np.uint8)
            cv2.imshow(window, cv2.bitwise_and(frame, frame, mask=mask))
            key = cv2.waitKey(20) & 255
            if key == ord("s"):
                thresholds[label] = candidate
                save_hsv_thresholds(thresholds, config_path)
                break
            if key == ord("q"):
                break
    finally:
        cv2.destroyWindow(window)

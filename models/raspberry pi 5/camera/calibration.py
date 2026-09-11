
import logging
import math
import time

import cv2
import numpy as np


logger = logging.getLogger("calibration")


def lock_exposure_and_white_balance(camera, settle_time_s=2.0):

    settle_time_s = float(settle_time_s)

    if not math.isfinite(settle_time_s) or settle_time_s < 0:
        raise ValueError("settle_time_s debe ser finito y >= 0")

    camera.set_auto_controls()

    if camera.simulated:
        logger.warning("Calibración omitida: cámara simulada")
        return None, None

    logger.info(
        "Esperando %.1f s para el ajuste automático de AE/AWB",
        settle_time_s,
    )
    time.sleep(settle_time_s)

    metadata = camera.capture_metadata()
    required = ("ExposureTime", "AnalogueGain", "ColourGains")
    missing = [key for key in required if metadata.get(key) is None]

    if missing:
        raise RuntimeError(
            "No se puede calibrar; faltan metadatos: "
            + ", ".join(missing)
        )

    exposure_us = int(metadata["ExposureTime"])
    analogue_gain = float(metadata["AnalogueGain"])
    awb_gains = tuple(float(v) for v in metadata["ColourGains"])

    camera.set_manual_controls(
        exposure_us=exposure_us,
        analogue_gain=analogue_gain,
        awb_gains=awb_gains,
    )

    logger.info(
        "Bloqueo solicitado: exposición=%s us, ganancia=%s, AWB=%s",
        exposure_us,
        analogue_gain,
        awb_gains,
    )

    return exposure_us, awb_gains


def _validate_hsv_bounds(bounds):
    """Valida antes de convertir a uint8 para evitar desbordamientos."""
    lower = np.asarray(bounds["lower"], dtype=float)
    upper = np.asarray(bounds["upper"], dtype=float)

    if lower.shape != (3,) or upper.shape != (3,):
        raise ValueError("Cada límite HSV debe contener tres valores")

    for value in (lower, upper):
        if (
            not np.all(np.isfinite(value))
            or np.any(value != np.floor(value))
            or np.any(value < 0)
            or np.any(value > np.array([179, 255, 255]))
        ):
            raise ValueError(
                "HSV inválido: H debe estar en 0–179; S y V en 0–255"
            )

    if np.any(lower > upper):
        raise ValueError(
            "lower debe ser <= upper. "
            "Para intervalos separados usa una lista de rangos"
        )

    return lower.astype(np.uint8), upper.astype(np.uint8)


def verify_hsv_thresholds(
    frame,
    hsv_thresholds,
    min_pixels=25,
    roi_mask=None,
):

    if (
        not isinstance(frame, np.ndarray)
        or frame.dtype != np.uint8
        or frame.ndim != 3
        or frame.shape[2] != 3
        or frame.size == 0
    ):
        raise ValueError("frame debe ser una imagen BGR uint8 válida")

    if not isinstance(min_pixels, int) or min_pixels <= 0:
        raise ValueError("min_pixels debe ser un entero positivo")

    if roi_mask is not None:
        if (
            not isinstance(roi_mask, np.ndarray)
            or roi_mask.shape != frame.shape[:2]
            or roi_mask.dtype != np.uint8
        ):
            raise ValueError(
                "roi_mask debe ser uint8 y tener el tamaño del frame"
            )

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    results = {}

    for label, bounds in hsv_thresholds.items():
        ranges = [bounds] if isinstance(bounds, dict) else bounds

        if not isinstance(ranges, (list, tuple)) or not ranges:
            raise ValueError(f"Rangos HSV inválidos para '{label}'")

        mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        for interval in ranges:
            lower, upper = _validate_hsv_bounds(interval)
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(hsv, lower, upper),
            )

        if roi_mask is not None:
            mask[roi_mask == 0] = 0

        count = cv2.countNonZero(mask)
        results[label] = count >= min_pixels

        if not results[label]:
            logger.warning(
                "Color '%s': %d píxeles, mínimo requerido %d. "
                "Comprueba que el color esté visible antes de recalibrar.",
                label,
                count,
                min_pixels,
            )

    return results